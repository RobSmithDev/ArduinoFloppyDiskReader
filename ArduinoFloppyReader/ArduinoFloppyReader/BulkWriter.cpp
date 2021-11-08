#include "BulkWriter.h"
//#include <synchapi.h>

#include "ArduinoInterface.h"
#ifdef _WIN32
#include <conio.h>
#include <string>
#include <iostream>
#include <filesystem>
namespace fs = std::filesystem;
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
    }
    else {
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

namespace fs = std::filesystem;

using namespace ArduinoFloppyReader;

//// Read an ADF file and write it to disk
int BulkWriter::adf2Disk(const std::wstring& filename, const bool inHDMode, bool verify) {
    printf("\nWrite disk from ADF mode\n\n");
    if (!verify) printf("WARNING: It is STRONGLY recommended to write with verify support turned on.\r\n\r\n");

    ADFResult result = m_adfWriter.ADFToDisk(filename, inHDMode, verify, true, false, true, [](const int currentTrack, const DiskSurface currentSide, bool isVerifyError, const CallbackOperation operation) ->WriteResponse {
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
	   printf("\n%s                                                  ", m_adfWriter.getLastError().c_str()); break;
    case ADFResult::adfrDiskWriteProtected:			printf("\rError, disk is write protected!                                                    "); break;
    default:										printf("\rAn unknown error occured                                                           "); break;
    }

    return 0;
}

// Iterate through a directory
void BulkWriter::writeDirectory(const std::wstring& port, const std::wstring& directory, const bool inHDMode, bool verify) {

    std::string const& adfExtension = ".adf";

    if (!fs::exists(directory))
    {
	   std::wcout << "Directory not found " << directory << std::endl;
	   return;
    }

    if (!m_adfWriter.openDevice(port)) {
	   printf("\rError opening COM port: %s  ", m_adfWriter.getLastError().c_str());
	   return;
    }


    bool isFirstDisk = true;

    std::wcout << "Writing all adf files in folder: " << directory << std::endl;

    for (const auto& entry : fs::directory_iterator(directory))
    {
	   if (!isFirstDisk)
	   {
		  std::cout << std::endl;
		  std::cout << "Remove disk, and insert next disk." << std::endl;

		  while (m_adfWriter.DiskPresent())
		  {
			 Sleep(500);
		  }

	   }
	   else
	   {
		  isFirstDisk = false;
	   }

	   std::cout << "Insert not write protected disk." << std::endl;

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
				    std::cout << "Disk is write protected." << std::endl;
				    writeProtectWarningGiven = true;
				}
			 }
			 Sleep(2000);
		  }
	   }


	   if (entry.is_regular_file() && entry.path().extension() == adfExtension)
	   {
		  std::cout << "Writing : " << std::endl;
		  std::cout << entry.path().filename() << std::endl;

		  if (entry.exists())
		  {
			 adf2Disk(entry.path(), inHDMode, verify);
		  }
		  else
		  {
			 std::cout << "Error opening " << entry.path().filename() << std::endl;
		  }

	   }
    }

    std::wcout << "Finished writing files in folder: " << directory << std::endl;

    //TODO: add some better handling for this => possibly opening and closing for every disk.
    m_adfWriter.closeDevice();

}