/* ArduinoFloppyReader
*
* Copyright (C) 2017 Robert Smith (@RobSmithDev)
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

//////////////////////////////////////////////////////////////////////////////////////////
// Example console application for reading floppy drives                                //
//////////////////////////////////////////////////////////////////////////////////////////


#include "stdafx.h"
#include <windows.h>
#include "..\lib\ADFWriter.h"

using namespace ArduinoFloppyReader;

ADFWriter writer;

int wmain(int argc, wchar_t* argv[], wchar_t *envp[])
{
	printf("Arduino Amiga Floppydisk Reader, Copyright (C) 2017 Robert Smith\r\n");
	printf("Full sourcecode and documentation at http://amiga.robsmithdev.co.uk\r\n");
	printf("This is free software licenced under the GNU General Public Licence V3\r\n\r\n");

	if (argc < 3) {
		printf("Usage:\r\n");
		printf("ArduinoFloppyReader COMPORT OutputFilename.ADF\r\n\r\n");
		return 0;
	}
	
	writer.openDevice(_wtoi(argv[1]));
	if (writer.analyseDisk([](int progress) -> bool {
			printf("\rAnalysing Disk. %i %% complete", progress);
			return true;
		}
	) == AnalysisResult::arComplete) 	
	{

		ADFResult result = writer.writeADF(argv[2], 80, [](const int currentTrack, const DiskSurface currentSide, const int retryCounter, const int sectorsFound, const int badSectorsFound) ->WriteResponse {
				if (retryCounter > 15) {
					char input;
					do {
						printf("\rDisk has checksum errors/missing data.  [R]etry, [I]gnore, [A]bort?                                      ");
						input = toupper(getchar());
					} while ((input != 'R') && (input != 'I') && (input != 'A'));
					switch (input) {
						case 'R': break;
						case 'I': return WriteResponse::wrSkipBadChecksums;
						case 'A': return WriteResponse::wrAbort;
					}
				}
				printf("\rReading Track %i, %s side (retry: %i) - Got %i/11 sectors (%i bad found)   ", currentTrack, (currentSide == DiskSurface::dsUpper) ? "Upper" : "Lower", retryCounter, sectorsFound, badSectorsFound);
				return WriteResponse::wrContinue;
			});

		switch (result) {
		case adfrComplete:					printf("\rADF file created with valid checksums.                                             "); break;
		case adfrAborted:					printf("\rADF file aborted.                                                                  "); break;
		case adfrFileError:					printf("\rError creating ADF file.                                                           "); break;
		case adfrFileIOError:				printf("\rError writing to ADF file.                                                         "); break;
		case adfrCompletedWithErrors:		printf("\rADF file created with partial success.                                             "); break;
		case adfrDriveError:				printf("\rError communicating with the Arduino interface.                                    "); break;
		}
	}
	else printf("\r\n\r\nError reading disk!\r\n");
	writer.closeDevice();

	
	getchar();
    return 0;
}

