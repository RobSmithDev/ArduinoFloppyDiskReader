#include "BulkWriter.h"

#include "ArduinoInterface.h"

#include <iostream>
#include <filesystem>
#include <thread>

namespace fs = std::filesystem;


#ifdef _WIN32
#include <conio.h>
#include <string>
#else
#include <stdio.h>
#include <termios.h>

// Reuse from Main.cpp
void initTermios(int echo);
void resetTermios(void);
char _getChar();
std::wstring atw(const std::string& str);

#endif

using namespace ArduinoFloppyReader;

// Read an ADF file and write it to disk
int BulkWriter::adf2Disk(const std::wstring& filename, bool verify) {
	bool hdMode = false;

	// Detect disk speed
	const ArduinoFloppyReader::FirmwareVersion v = m_adfWriter.getFirwareVersion();

	// Disabled this for the moment, seems to be failing from time to time.
	//if (((v.major == 1) && (v.minor >= 9)) || (v.major > 1)) {
	//	if (m_adfWriter.GuessDiskDensity(hdMode) != ArduinoFloppyReader::ADFResult::adfrComplete) {
	//		printf("Unable to work out the density of the disk inserted.\n");
	//		return 1;
	//	}
	//}

	ADFResult result = m_adfWriter.ADFToDisk(filename, hdMode, verify, true, false, true, [this](const int currentTrack, const DiskSurface currentSide, bool isVerifyError, const CallbackOperation operation) ->WriteResponse {
		if (isVerifyError) {
			char input;
			do {
				printf("\rDisk write verify error on track %i, %s side. [R]etry, [S]kip, [A]bort?                                   ", currentTrack, (currentSide == DiskSurface::dsUpper) ? "Upper" : "Lower");
#ifdef _WIN32
				input = toupper(_getch());
#else
				input = toupper(_getChar());
#endif	
			} while ((input != 'R') && (input != 'S') && (input != 'A'));

			switch (input) {
			case 'R': return WriteResponse::wrRetry;
			case 'I': return WriteResponse::wrSkipBadChecksums;
			case 'A': return WriteResponse::wrAbort;
			}
		}
		printf("\r  Writing Track %i, %s side     ", currentTrack, (currentSide == DiskSurface::dsUpper) ? "Upper" : "Lower");
#ifndef _WIN32
		fflush(stdout);
#endif		
		return WriteResponse::wrContinue;
		});

	switch (result) {
	case ADFResult::adfrComplete:
		//printf("\rADF file written to disk                                                           "); 
		break;
	case ArduinoFloppyReader::ADFResult::adfrExtendedADFNotSupported:	printf("\rExtended ADF files are not currently supported.                                    "); break;
	case ArduinoFloppyReader::ADFResult::adfrMediaSizeMismatch:			if (hdMode)
		printf("\rDisk in drive was detected as HD, but a DD ADF file supplied.                      "); else
		printf("\rDisk in drive was detected as DD, but an HD ADF file supplied.                     ");
		break;
	case ADFResult::adfrFirmwareTooOld:             printf("\rCannot write this file, you need to upgrade the firmware first.                    "); break;
	case ADFResult::adfrCompletedWithErrors:		printf("\rADF file written to disk but there were errors during verification                 "); break;
	case ADFResult::adfrAborted:					printf("\rWriting ADF file to disk                                                           "); break;
	case ADFResult::adfrFileError:					printf("\rError opening ADF file.                                                            "); break;
	case ADFResult::adfrDriveError:					printf("\rError communicating with the Arduino interface.                                    ");
		printf("\n%s                                                  ", m_adfWriter.getLastError().c_str()); break;
	case ADFResult::adfrDiskWriteProtected:			printf("\rError, disk is write protected!                                                    "); break;
	default:										printf("\rAn unknown error occured                                                           "); break;
	}

	if (result != ADFResult::adfrComplete)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

// Iterate through a directory
void BulkWriter::writeDirectory(const std::wstring& port, const std::wstring& directory, const bool inHDMode, bool verify) {

	std::string const& adfExtension = ".adf";

	if (!fs::exists(directory))
	{
		std::wcout << "Directory not found " << directory << std::endl;
		return;
	}

	// Check if there are .adf files in the folder.
	std::vector<fs::directory_entry> listOfAdfs;

	auto directoryIterator = fs::directory_iterator(directory);

	for (const auto& entry : directoryIterator)
	{
		if (entry.is_regular_file() && entry.path().extension() == adfExtension)
		{
			listOfAdfs.push_back(entry);
		}
	}

	if (!m_adfWriter.openDevice(port)) {
		printf("\rError opening COM port: %s  ", m_adfWriter.getLastError().c_str());
		return;
	}

	bool isFirstDisk = true;

	std::wcout << "Writing " << listOfAdfs.size() << " adf files in folder : " << directory << std::endl << std::endl;

	for (const auto& entry : listOfAdfs)
	{
		if (!isFirstDisk)
		{
			std::cout << "\rRemove disk, and insert next disk.        ";
			while (m_adfWriter.DiskPresent())
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(500));
			}
		}
		else
		{
			if (!m_adfWriter.DiskPresent())
			{
				std::cout << "Insert first disk.";
			}
			isFirstDisk = false;
		}

		bool isWriteProtected = false;
		bool isDiskPresent = false;
		bool writeProtectWarningGiven = false;

		bool checkForNonWriteProtectedDisk = true;

		while (checkForNonWriteProtectedDisk)
		{
			isDiskPresent = m_adfWriter.DiskPresent();
			isWriteProtected = m_adfWriter.WriteProtected();

			if (isDiskPresent && !isWriteProtected)
			{
				// Everything ok, get ready to start writing.
				checkForNonWriteProtectedDisk = false;
			}
			else
			{
				if (isDiskPresent && isWriteProtected)
				{
					if (!writeProtectWarningGiven)
					{
						std::cout << "\rDisk is write protected, insert not write protected disk.         ";
						writeProtectWarningGiven = true;
					}
				}

				std::this_thread::sleep_for(std::chrono::milliseconds(2000));
			}
		}

		if (entry.is_regular_file() && entry.path().extension() == adfExtension)
		{
			std::cout << "\r-> " << entry.path().filename() << std::endl;

			if (entry.exists())
			{
				bool writtenSuccesfullyOrAborted = false;

				do {
#ifdef _WIN32
					int result = adf2Disk(entry.path(), verify);
#else
					int result = adf2Disk(atw(entry.path()), verify);
#endif
					if (result)
					{
						std::cout << "Something went wrong. [R]etry, [S]kip, [A]bort?";

						char input;

						do {
#ifdef _WIN32
							input = toupper(_getch());
#else
							input = toupper(_getChar());
#endif	
						} while ((input != 'R') && (input != 'S') && (input != 'A'));

						switch (input) {
						case 'R':
							break;
						case 'S':
							writtenSuccesfullyOrAborted = true;
							;
						case 'A':
							m_adfWriter.closeDevice();
							return;
						}
						}
					else
					{
						writtenSuccesfullyOrAborted = true;
					}

					} while (!writtenSuccesfullyOrAborted);
				}
			else
			{
				std::cout << "Error opening " << entry.path().filename() << std::endl;
			}
			}
		}

	//TODO: add some better handling for this => possibly opening and closing for every disk.
	m_adfWriter.closeDevice();

	}