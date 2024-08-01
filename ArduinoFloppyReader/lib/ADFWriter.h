/* DrawBridge - aka ArduinoFloppyReader (and writer)
*
* Copyright (C) 2017-2024 Robert Smith (@RobSmithDev)
* https://amiga.robsmithdev.co.uk
*
* This file is multi-licensed under the terms of the Mozilla Public
* License Version 2.0 as published by Mozilla Corporation and the
* GNU General Public License, version 2 or later, as published by the
* Free Software Foundation.
*
* MPL2: https://www.mozilla.org/en-US/MPL/2.0/
* GPL2: https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html
*
*/

//////////////////////////////////////////////////////////////////////////////////////////
// Class that reads reads the MFM data, decodes it, and then writes an ADF file to disk //
//////////////////////////////////////////////////////////////////////////////////////////
//
// Purpose:
// This class reads the raw data from the device and attemps to check for errors.
// Once all errors have been corrected the class will save the tracks to disk as an ADF file
//
// The class also handles writing an ADF file back to disk and optionally verifying the write.
// 
// The MFM decoding algorithm and information regarding finding the start of a sector
// were taken from the excellent documentation by Laurent Clévy at http://lclevy.free.fr/adflib/adf_info.html
// Also credits to Keith Monahan https://www.techtravels.org/tag/mfm/ regarding a bug in the MFM sector start data
//
// V2.8

#pragma once
#include <functional>

#include "RotationExtractor.h"
#include "ArduinoInterface.h"

#define MFM_MASK    0x55555555L		
#define AMIGA_WORD_SYNC  0x4489							 // Disk SYNC code for the Amiga start of sector
#define SECTOR_BYTES	512								 // Number of bytes in a decoded sector
#define NUM_SECTORS_PER_TRACK_DD 11						 // Number of sectors per track
#define NUM_SECTORS_PER_TRACK_HD 22						  // Same but for HD disks
#define RAW_SECTOR_SIZE (8+56+SECTOR_BYTES+SECTOR_BYTES)      // Size of a sector, *Including* the sector sync word longs
#define ADF_TRACK_SIZE_DD (SECTOR_BYTES*NUM_SECTORS_PER_TRACK_DD)   // Bytes required for a single track dd
#define ADF_TRACK_SIZE_HD (SECTOR_BYTES*NUM_SECTORS_PER_TRACK_HD)   // Bytes required for a single track hd


	// Buffer to hold raw data for just a single sector
typedef unsigned char RawEncodedSector[RAW_SECTOR_SIZE];
typedef unsigned char RawDecodedSector[SECTOR_BYTES];
typedef RawDecodedSector RawDecodedTrackDD[NUM_SECTORS_PER_TRACK_DD];
typedef RawDecodedSector RawDecodedTrackHD[NUM_SECTORS_PER_TRACK_HD];
typedef unsigned char RawMFMData[SECTOR_BYTES + SECTOR_BYTES];
namespace ArduinoFloppyReader {





	// Optional how to respond to the callback from the writeADF command.
	enum class WriteResponse {
								wrContinue,				// Continue working as normal
								wrAbort,				// Abort thr process and stop
								wrSkipBadChecksums,		// Request that the process ignores bad checksums (not recommended unless the retryCounter gets beyond RETRYS_PER_PHASE*16)
								wrRetry                 // Retry!
							};

	enum class ADFResult {
						adfrComplete,					// Process completed successfully
						adfrAborted,					// Process was aborted
						adfrFileError,					// Error opening the file to write to
						adfrFileIOError,				// Error writing to the ADF file
						adfrCompletedWithErrors,		// Completed but with sector errors
						adfrDiskWriteProtected,         // Disk is write protected!
						adfrDriveError,					// Something wrong with reading the disk
						adfrFirmwareTooOld,				// Firmware is too old
						adfrMediaSizeMismatch,			// HD/DD mismatch
						adfrExtendedADFNotSupported,	// Extended ADFs not currently supported
						adfrBadSCPFile,                 // Incompatiable IPF library
						adfrIPFLibraryNotAvailable      // The IPF library was not found
					};

	enum class AnalysisResult {
			                 arComplete,				// Anaysis is complete and ready for use
							 arFailed,                  // Anaysis failed to read a disk
							 arAborted,                 // Analysis was aborted
							 arDriveError               // Something wrong talking to thre drive
						};

	enum class CallbackOperation {
							coStarting,
							coReading,
							coWriting,
							coVerifying,
							coRetryReading,
							coRetryWriting,
							coReVerifying,
							coReadingFile
						};

	// Main writer class
	class ADFWriter {
	private:
		// The Arduino device
		ArduinoInterface m_device;

	public:  
		ADFWriter();
		~ADFWriter();

		// Open the device we want to use.  Returns TRUE if it worked
		bool openDevice(const std::wstring& portName);

		// Close the device when we've finished
		void closeDevice();

		// Get the current firmware version.  Only valid if openDevice is successful
		const FirmwareVersion getFirwareVersion() const;

		std::string getLastError() { return m_device.getLastErrorStr(); };
		DiagnosticResponse getLastErrorCode() { return m_device.getLastError(); };

		ADFResult runClean(std::function<bool(const uint16_t position, const uint16_t maxPos)>  onProgress);

		// Reads the disk and write the data to the ADF file supplied.  The callback is for progress, and you can returns FALSE to abort the process
		// numTracks is the number of tracks to read.  Usually 80 (0..79), sometimes track 80 and 81 are needed
		ADFResult DiskToADF(const std::wstring& outputFile, const bool inHDMode, const unsigned int numTracks, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const int retryCounter, const int sectorsFound, const int badSectorsFound, const int maxSectors, const CallbackOperation operation)> callback);

		// Reads the disk and write the data to the SCP file supplied.  The callback is for progress, and you can returns FALSE to abort the process
		// numTracks is the number of tracks to read.  Usually 80 (0..79), sometimes track 80 and 81 are needed. revolutions is hwo many revolutions of the disk to save (1-5)
		// SCP files are a low level flux record of the disk and usually can backup copy protected disks to.  Without special hardware they can't usually be written back to disks.
		ADFResult DiskToSCP(const std::wstring& outputFile, bool isHDMode, const unsigned int numTracks, const unsigned char revolutions, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const int retryCounter, const int sectorsFound, const int badSectorsFound, const int maxSectors, const CallbackOperation operation)> callback, bool useNewFluxReader = false);

		// Writes an ADF file back to a floppy disk.  Return FALSE in the callback to abort this operation.  If verify is set then the track isread back and and sector checksums are checked for 11 valid sectors
		ADFResult ADFToDisk(const std::wstring& inputFile, const bool inHDMode, bool verify, bool usePrecompMode, bool eraseFirst, bool writeFromIndex, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const bool isVerifyError, const CallbackOperation operation) > callback);

		// Writes an IMG, IMA or ST file to disk. Return FALSE in the callback to abort this operation.  If verify is set then the track isread back and and sector checksums are checked for 11 valid sectors
		ADFResult sectorFileToDisk(const std::wstring& inputFile, const bool inHDMode, bool verify, bool usePrecompMode, bool eraseFirst, bool useAtariSTTiming, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const bool isVerifyError, const CallbackOperation operation) > callback);

		// Attempt to read a PC or Atari ST disk as a disk sector-based file
		ADFResult diskToIBMST(const std::wstring& outputFile, const bool inHDMode, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const int retryCounter, const int sectorsFound, const int badSectorsFound, const int maxSectors, const CallbackOperation operation)> callback);

		// Writes an SCP file back to a floppy disk.  Return FALSE in the callback to abort this operation.  
		ADFResult SCPToDisk(const std::wstring& inputFile, bool extraErases, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const bool isVerifyError, const CallbackOperation operation) > callback);

		// Writes an IPF file back to a floppy disk.  Return FALSE in the callback to abort this operation.  
		ADFResult IPFToDisk(const std::wstring& inputFile, bool extraErases, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const bool isVerifyError, const CallbackOperation operation) > callback);


		// Run diagnostics on the system.  You do not need to call openDevice first.  Return TRUE if everything passed
		bool runDiagnostics(const std::wstring& portName, std::function<void(bool isError, const std::string message)> messageOutput, std::function<bool(bool isQuestion, const std::string question)> askQuestion);
		
		// Attempt to work out what the density of the currently inserted disk is
		ADFResult GuessDiskDensity(bool& isHD);
	};


};

void encodeSector(const unsigned int trackNumber, const ArduinoFloppyReader::DiskSurface surface, bool isHD, const unsigned int sectorNumber, const RawDecodedSector& input, RawEncodedSector& encodedSector, unsigned char& lastByte);

