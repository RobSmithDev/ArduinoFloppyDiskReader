/* ArduinoFloppyReader (and writer)
*
* Copyright (C) 2017-2021 Robert Smith (@RobSmithDev)
* https://amiga.robsmithdev.co.uk
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU Library General Public
* License as published by the Free Software Foundation; either
* version 3 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Library General Public License for more details.
*
* You should have received a copy of the GNU Library General Public
* License along with this program; if not see http://www.gnu.org/licenses/
*/

////////////////////////////////////////////////////////////////////////////////////////////
// Example console application for reading and writing floppy disks to and from ADF files //
////////////////////////////////////////////////////////////////////////////////////////////

#include "ADFWriter.h"
#include "ArduinoInterface.h"
#ifdef _WIN32
#include <conio.h>
#else
#include <stdio.h>
#include <termios.h>

#ifndef _wcsupr
#include <wctype.h>
void _wcsupr(wchar_t* str) {
	while (*str) {
		*str = towupper(*str);
		str++;
	}
}
#endif
static struct termios old, current;

/* Initialize new terminal i/o settings */
void initTermios(int echo) 
{
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  current = old; /* make new settings same as old settings */
  current.c_lflag &= ~ICANON; /* disable buffered i/o */
  if (echo) {
      current.c_lflag |= ECHO; /* set echo mode */
  } else {
      current.c_lflag &= ~ECHO; /* set no echo mode */
  }
  tcsetattr(0, TCSANOW, &current); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void) 
{
  tcsetattr(0, TCSANOW, &old);
}

char _getChar() 
{
  char ch;
  initTermios(0);
  ch = getchar();
  resetTermios();
  return ch;
}

std::wstring atw(const std::string& str) {
	std::wstring ws(str.size(), L' '); 
	ws.resize(::mbstowcs((wchar_t*)ws.data(), str.c_str(), str.size())); 
	return ws;
}
#endif

using namespace ArduinoFloppyReader;

ADFWriter writer;


// Read an ADF file and write it to disk
void adf2Disk(const std::wstring& filename, bool verify) {
	printf("\nWrite disk from ADF mode\n\n");
	if (!verify) printf("WARNING: It is STRONGLY recommended to write with verify support turned on.\r\n\r\n");

	ADFResult result = writer.ADFToDisk(filename,verify,true, [](const int currentTrack, const DiskSurface currentSide, bool isVerifyError) ->WriteResponse {
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
		printf("\rWriting Track %i, %s side     ", currentTrack, (currentSide == DiskSurface::dsUpper) ? "Upper" : "Lower");
#ifndef _WIN32
		fflush(stdout);		
#endif		
		return WriteResponse::wrContinue;
	});

	switch (result) {
	case ADFResult::adfrComplete:					printf("\rADF file written to disk                                                           "); break;
	case ADFResult::adfrCompletedWithErrors:		printf("\rADF file written to disk but there were errors during verification                 "); break;
	case ADFResult::adfrAborted:					printf("\rWriting ADF file to disk                                                           "); break;
	case ADFResult::adfrFileError:					printf("\rError opening ADF file.                                                            "); break;
	case ADFResult::adfrDriveError:					printf("\rError communicating with the Arduino interface.                                    "); 
													printf("\n%s                                                  ", writer.getLastError().c_str()); break;
	case ADFResult::adfrDiskWriteProtected:			printf("\rError, disk is write protected!                                                    "); break;
	default:										printf("\rAn unknown error occured                                                           "); break;
	}
}

// Stolen from https://stackoverflow.com/questions/11635/case-insensitive-string-comparison-in-c
// A wide-string case insensative compare of strings
bool iequals(const std::wstring& a, const std::wstring& b) {
	return std::equal(a.begin(), a.end(),b.begin(), b.end(),[](wchar_t a, wchar_t b) {
			return tolower(a) == tolower(b);
	});
}
bool iequals(const std::string& a, const std::string& b) {
	return std::equal(a.begin(), a.end(),b.begin(), b.end(),[](char a, char b) {
			return tolower(a) == tolower(b);
	});
}

// Read a disk and save it to ADF or SCP files
void disk2ADF(const std::wstring& filename) {
	const wchar_t* extension = wcsrchr(filename.c_str(), L'.');
	bool isADF = true;

	if (extension) {
		extension++;
		isADF = !iequals(extension, L"SCP");
	}

	if (isADF) printf("\nCreate ADF from disk mode\n\n"); else printf("\nCreate SCP file from disk mode\n\n");

	// Get the current firmware version.  Only valid if openDevice is successful
	const ArduinoFloppyReader::FirmwareVersion v = writer.getFirwareVersion();
	if ((v.major == 1) && (v.minor < 8)) {
		if (!isADF) {
			printf("This requires firmware V1.8 or newer.\n");
			return;
		}
		else {
			// improved disk timings in 1.8, so make them aware
			printf("Rob strongly recommends updating the firmware on your Arduino to at least V1.8.\n");
			printf("That version is even better at reading old disks.\n");
		}
	}

	auto callback = [isADF](const int currentTrack, const DiskSurface currentSide, const int retryCounter, const int sectorsFound, const int badSectorsFound) ->WriteResponse {
		if (retryCounter > 20) {
			char input;
			do {
				printf("\rDisk has checksum errors/missing data.  [R]etry, [I]gnore, [A]bort?                                      ");
#ifdef _WIN32
				input = toupper(_getch());
#else
				input = toupper(_getChar());
#endif	
			} while ((input != 'R') && (input != 'I') && (input != 'A'));
			switch (input) {
			case 'R': return WriteResponse::wrRetry;
			case 'I': return WriteResponse::wrSkipBadChecksums;
			case 'A': return WriteResponse::wrAbort;
			}
		}
		if (isADF) {
			printf("\rReading Track %i, %s side (retry: %i) - Got %i/11 sectors (%i bad found)   ", currentTrack, (currentSide == DiskSurface::dsUpper) ? "Upper" : "Lower", retryCounter, sectorsFound, badSectorsFound);
		}
		else {
			printf("\rReading Track %i, %s side   ", currentTrack, (currentSide == DiskSurface::dsUpper) ? "Upper" : "Lower");
		}
#ifndef _WIN32
		fflush(stdout);		
#endif		
		return WriteResponse::wrContinue;
	};

	ADFResult result;
	
	if (isADF) result = writer.DiskToADF(filename, 80, callback); else result = writer.DiskToSCP(filename, 80, 3, callback);

	switch (result) {
	case ADFResult::adfrComplete:					printf("\rFile created successfully.                                                     "); break;
	case ADFResult::adfrAborted:					printf("\rFile aborted.                                                                  "); break;
	case ADFResult::adfrFileError:					printf("\rError creating file.                                                           "); break;
	case ADFResult::adfrFileIOError:				printf("\rError writing to file.                                                         "); break;
	case ADFResult::adfrFirmwareTooOld:			    printf("\rThis requires firmware V1.8 or newer.                                          "); break;
	case ADFResult::adfrCompletedWithErrors:		printf("\rFile created with partial success.                                             "); break;
	case ADFResult::adfrDriveError:					printf("\rError communicating with the Arduino interface.                                ");
													printf("\n%s                                                  ", writer.getLastError().c_str()); break;
	default: 										printf("\rAn unknown error occured.                                                      "); break;
	}
}

// Run the diagnostics module
void runDiagnostics(const std::wstring& port) {
	printf("\rRunning diagnostics on COM port: %ls\n",port.c_str());

	writer.runDiagnostics(port, [](bool isError, const std::string message)->void {
		if (isError)
			printf("DIAGNOSTICS FAILED: %s\n",message.c_str());
		else 
			printf("%s\n", message.c_str());
	}, [](bool isQuestion, const std::string question)->bool {
		if (isQuestion) 
			printf("%s [Y/N]: ", question.c_str());
		else 
			printf("%s [Enter/ESC]: ", question.c_str());

		char c;
		do {
#ifdef _WIN32
			c = toupper(_getch());
#else
			c = toupper(_getChar());
#endif	
		} while ((c != 'Y') && (c != 'N') && (c != '\n') && (c != '\r') && (c != '\x1B'));
		printf("%c\n", c);

		return (c == 'Y') || (c == '\n') || (c == '\r') || (c == '\x1B');
	});

	writer.closeDevice();
}

#ifdef _WIN32
int wmain(int argc, wchar_t* argv[], wchar_t *envp[])
#else
int main(int argc, char* argv[], char *envp[])
#endif
{
	printf("Arduino Amiga ADF & SCP Floppy Disk Reader/Writer V2.5, Copyright (C) 2017-2021 Robert Smith\r\n");
	printf("Full sourcecode and documentation at https://amiga.robsmithdev.co.uk\r\n");
	printf("This is free software licenced under the GNU General Public Licence V3\r\n\r\n");

	if (argc < 3) {
		printf("Usage:\r\n\n");
		printf("To read a disk to an ADF file:\r\n");
		printf("ArduinoFloppyReader <COMPORT> OutputFilename.ADF [READ]\r\n\r\n");
		printf("To read a disk to an SCP file:\r\n");
		printf("ArduinoFloppyReader <COMPORT> OutputFilename.SCP [READ]\r\n\r\n");
		printf("To write an ADF file to disk:\r\n");
		printf("ArduinoFloppyReader <COMPORT> InputFilename.ADF WRITE [VERIFY]\r\n\r\n");
		printf("To start interface diagnostics:\r\n");
		printf("ArduinoFloppyReader <COMPORT> DIAGNOSTIC\r\n\r\n");
		printf("Detected Serial Devices:\r\n");

		std::vector<std::wstring> portList;
		ArduinoFloppyReader::ArduinoInterface::enumeratePorts(portList);
		for (const std::wstring& port : portList)
			printf("    %ls\n", port.c_str());
		printf("\r\n");

		return 0;
	}
	
#ifdef _WIN32	
	bool writeMode = (argc > 3) && (iequals(argv[3], L"WRITE"));
	bool verify = (argc > 4) && (iequals(argv[4], L"VERIFY"));
	if (argc >= 2) {
		std::wstring port = argv[1];
		std::wstring filename = argv[2];
		int i = _wtoi(argv[1]);
		if (i) port = L"COM" + std::to_wstring(i); else port = argv[1];
#else
	bool writeMode = (argc > 3) && (iequals(argv[3], "WRITE"));
	bool verify = (argc > 4) && (iequals(argv[4], "VERIFY"));
	if (argc >= 2) {
		std::wstring port = atw(argv[1]);
		std::wstring filename = atw(argv[2]);
#endif
		if (iequals(filename, L"DIAGNOSTIC")) {
			runDiagnostics(port);
		} else
		if (!writer.openDevice(port)) {
			printf("\rError opening COM port: %s  ", writer.getLastError().c_str());
		}
		else {
			if (writeMode) adf2Disk(filename.c_str(), verify); else disk2ADF(filename.c_str());
			writer.closeDevice();
		}
	}
	
	printf("\r\n\r\n");
	
    return 0;
}

