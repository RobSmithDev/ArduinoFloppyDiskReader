/* ArduinoFloppyReader (and writer)
*
* Copyright (C) 2017-2020 Robert Smith (@RobSmithDev)
* http://amiga.robsmithdev.co.uk
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

#include "stdafx.h"
#include <windows.h>
#include "..\lib\ADFWriter.h"
#include <conio.h>

using namespace ArduinoFloppyReader;

ADFWriter writer;


// Read an ADF file and write it to disk
void adf2Disk(wchar_t* argv[], bool verify) {
	printf("\nWrite disk from ADF mode\n\n");
	if (!verify) printf("WARNING: It is STRONGLY recommended to write with verify support turned on.\r\n\r\n");

	ADFResult result = writer.ADFToDisk(argv[2],true, verify, [](const int currentTrack, const DiskSurface currentSide, bool isVerifyError) ->WriteResponse {
		if (isVerifyError) {
			char input;
			do {
				printf("\rDisk write verify error on track %i, %s side. [R]etry, [S]kip, [A]bort?                                   ", currentTrack, (currentSide == DiskSurface::dsUpper) ? "Upper" : "Lower");
				input = toupper(getchar());
			} while ((input != 'R') && (input != 'S') && (input != 'A'));
			
			switch (input) {
			case 'R': return WriteResponse::wrRetry;
			case 'I': return WriteResponse::wrSkipBadChecksums;
			case 'A': return WriteResponse::wrAbort;
			}
		}
		printf("\rWriting Track %i, %s side     ", currentTrack, (currentSide == DiskSurface::dsUpper) ? "Upper" : "Lower");
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
	}
}

// Read a disk and save it to ADF files
void disk2ADF(wchar_t* argv[]) {
	printf("\nCreate ADF from disk mode\n\n");

	ADFResult result = writer.DiskToADF(argv[2], 80, [](const int currentTrack, const DiskSurface currentSide, const int retryCounter, const int sectorsFound, const int badSectorsFound) ->WriteResponse {
		if (retryCounter > 20) {
			char input;
			do {
				printf("\rDisk has checksum errors/missing data.  [R]etry, [I]gnore, [A]bort?                                      ");
				input = toupper(getchar());
			} while ((input != 'R') && (input != 'I') && (input != 'A'));
			switch (input) {
			case 'R': return WriteResponse::wrRetry;
			case 'I': return WriteResponse::wrSkipBadChecksums;
			case 'A': return WriteResponse::wrAbort;
			}
		}
		printf("\rReading Track %i, %s side (retry: %i) - Got %i/11 sectors (%i bad found)   ", currentTrack, (currentSide == DiskSurface::dsUpper) ? "Upper" : "Lower", retryCounter, sectorsFound, badSectorsFound);
		return WriteResponse::wrContinue;
	});

	switch (result) {
	case ADFResult::adfrComplete:					printf("\rADF file created with valid checksums.                                             "); break;
	case ADFResult::adfrAborted:					printf("\rADF file aborted.                                                                  "); break;
	case ADFResult::adfrFileError:					printf("\rError creating ADF file.                                                           "); break;
	case ADFResult::adfrFileIOError:				printf("\rError writing to ADF file.                                                         "); break;
	case ADFResult::adfrCompletedWithErrors:		printf("\rADF file created with partial success.                                             "); break;
	case ADFResult::adfrDriveError:					printf("\rError communicating with the Arduino interface.                                    ");
													printf("\n%s                                                  ", writer.getLastError().c_str()); break;
	}
}

// Run the diagnostics module
void runDiagnostics(int comPort) {
	printf("\rRunning diagnostics on COM port: %i\n",comPort);

	writer.runDiagnostics(comPort, [](bool isError, const std::string message)->void {
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
			c = _getch();
			c=toupper(c);
		} while ((c != 'Y') && (c != 'N') && (c != '\n') && (c != '\r') && (c != '\x1B'));
		printf("%c\n", c);

		return (c == 'Y') || (c == '\n') || (c == '\r') || (c == '\x1B');
	});

	writer.closeDevice();
}

int wmain(int argc, wchar_t* argv[], wchar_t *envp[])
{
	printf("Arduino Amiga ADF Floppy disk Reader/Writer, Copyright (C) 2017-2018 Robert Smith\r\n");
	printf("Full sourcecode and documentation at http://amiga.robsmithdev.co.uk\r\n");
	printf("This is free software licenced under the GNU General Public Licence V3\r\n\r\n");

	if (argc < 3) {
		printf("Usage:\r\n\n");
		printf("To read a disk to an ADF file:\r\n");
		printf("ArduinoFloppyReader <COMPORT> OutputFilename.ADF [READ]\r\n\r\n");
		printf("To write an ADF file to disk:\r\n");
		printf("ArduinoFloppyReader <COMPORT> InputFilename.ADF WRITE [VERIFY]\r\n\r\n");
		printf("To start interface diagnostics:\r\n");
		printf("ArduinoFloppyReader DIAGNOSTIC <COMPORT>\r\n\r\n");
		return 0;
	}

	bool writeMode = false;
	bool verify = false;
	if (argc > 3) {
		_wcsupr(argv[3]);
		writeMode = wcscmp(argv[3], L"WRITE") == 0;
	}
	if (argc > 4) {
		_wcsupr(argv[4]);
		verify = wcscmp(argv[4], L"VERIFY") == 0;
	}

	_wcsupr(argv[1]);
	if (wcscmp(argv[1], L"DIAGNOSTIC") == 0) {
		runDiagnostics(_wtoi(argv[2]));
	}
	else {

		if (!writer.openDevice(_wtoi(argv[1]))) {
			printf("\rError opening COM port: %s  ", writer.getLastError().c_str());
		}
		else {

			if (writeMode) adf2Disk(argv, verify); else disk2ADF(argv);

			writer.closeDevice();
		}
	}
	
	getchar();
    return 0;
}

