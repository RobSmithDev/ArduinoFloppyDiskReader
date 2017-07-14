/* ArduinoFloppyReader
*
* Copyright (C) 2017 Robert Smith (@RobSmithDev)
* http://amiga.robsmithdev.co.uk
*
* This library is free software; you can redistribute it and/or
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
* License along with this library; if not, see http://www.gnu.org/licenses/
*/

//////////////////////////////////////////////////////////////////////////////////////////
// Class that reads reads the MFM data, decodes it, and then writes an ADF file to disk //
//////////////////////////////////////////////////////////////////////////////////////////
//
// Purpose:
// This class reads the raw data from the device and attemps to check for errors.
// Once all errors have been corrected the class will save the tracks to disk as an ADF file
// 
// The MFM decoding algorithm and information regarding finding the start of a sector
// were taken from the excellent documentation by Laurent Clévy at http://lclevy.free.fr/adflib/adf_info.html
// Also credits to Keith Monahan https://www.techtravels.org/tag/mfm/ regarding a bug in the MFM sector start data

#pragma once
#include <functional>
#include "ArduinoInterface.h"
#include "phaseAnalyser.h"

// Number of read retries performed per disk phase check
#define RETRYS_PER_PHASE     5

namespace ArduinoFloppyReader {

	// Optional how to respond to the callback from the writeADF command.
	enum WriteResponse {
								wrContinue,				// Continue working as normal
								wrAbort,				// Abort thr process and stop
								wrSkipBadChecksums,		// Request that the process ignores bad checksums (not recommended unless the retryCounter gets beyond RETRYS_PER_PHASE*16)
							};

	enum ADFResult {
						adfrComplete,					// Process completed successfully
						adfrAborted,					// Process was aborted
						adfrFileError,					// Error opening the file to write to
						adfrFileIOError,				// Error writing to the ADF file
						adfrCompletedWithErrors,		// Completed but with sector errors
						adfrDriveError					// Something wrong with reading the disk
					};

	enum AnalysisResult {
			                 arComplete,				// Anaysis is complete and ready for use
							 arFailed,                  // Anaysis failed to read a disk
							 arAborted,                 // Analysis was aborted
							 arDriveError               // Something wrong talking to thre drive
						};

	// Main writer class
	class ADFWriter {
	private:
		// The Arduino device
		ArduinoInterface m_device;

		// Phase corrector library
		PhaseAnalyser m_phase;

	public: 
		// Open the device we want to use.  Returns TRUE if it worked
		bool openDevice(const unsigned int comPort);

		// Close the device when we've finished
		void closeDevice();

		// Analyse phases, this is optional but massivly improves reading speed.  The callback provides progres between 0 and 100%.  Returning FALSE stops the process
		AnalysisResult analyseDisk(std::function<bool(int progress)> callback);

		// Reads the disk and write the data to the ADF file supplied.  The callback is for progress, and you can returns FALSE to abort the process
		// It is HIGHLY recommended to use analyseDisk() first
		// numTracks is the number of tracks to read.  Usually 80 (0..79), sometimes track 80 and 81 are needed
		ADFResult writeADF(const std::wstring outputFile, const unsigned int numTracks, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const int retryCounter, const int sectorsFound, const int badSectorsFound)> callback);
	};



};
