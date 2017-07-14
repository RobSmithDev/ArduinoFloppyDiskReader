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

#include "stdafx.h"
#include <windows.h>
#include <vector>
#include "ADFWriter.h"
#include "phaseAnalyser.h"
#include "ArduinoInterface.h"


using namespace ArduinoFloppyReader;


#define MFM_MASK    0x55555555		
#define AMIGA_WORD_SYNC  0x4489         // Disk SYNC code for the Amiga start of sector
#define TRACK_SIZE (56+512+512)			// Size of a sector, NOT including the sector sync word longs

// Buffer to hold raw data for just a single sector
typedef unsigned char RawSector[TRACK_SIZE];

// Structure to hold data while we decode it
typedef struct alignas(8)  {
	unsigned char trackFormat;        // This will be 0xFF for Amiga
	unsigned char trackNumber;        // Current track number (this is actually (tracknumber*2) + side
	unsigned char sectorNumber;       // The sector we just read (0 to 11)
	unsigned char sectorsRemaining;   // How many more sectors remain until the gap (0 to 10)

	unsigned long sectorLabel[4];     // OS Recovery Data, we ignore this

	DWORD headerChecksum;			  // Read from the header, header checksum
	DWORD dataChecksum;			      // Read from the header, data checksum

	DWORD headerChecksumCalculated;   // The header checksum we calculate
	DWORD dataCheckdumCalculated;     // The data checksum we calculate

	unsigned char data[512];          // decoded sector data
} DecodedSector;

// To hold a list of valid and checksum failed sectors
struct DecodedTrack {
	// A list of valid sectors where the checksums are OK
	std::vector<DecodedSector> validSectors;
	// A list of sectors found with invalid checksums.  These are used if ignore errors is triggered
	std::vector<DecodedSector> invalidSectors;
};


// MFM decoding algorithm
// *input;	MFM coded data buffer (size == 2*data_size) 
// *output;	decoded data buffer (size == data_size) 
// Returns the checksum calculated over the data
unsigned long decodeMFMdata(const unsigned long* input, unsigned long* output, const unsigned int data_size) {
	unsigned long odd_bits, even_bits;
	unsigned long chksum = 0L;
	unsigned int count;

	// the decoding is made here long by long : with data_size/4 iterations 
	for (count = 0; count < data_size / 4; count++) {
		odd_bits = *input;					// longs with odd bits 
		even_bits = *(unsigned long*)(((unsigned char*)input) + data_size);   // longs with even bits - located 'data_size' bytes after the odd bits

		chksum ^= odd_bits;              // XOR Checksum
		chksum ^= even_bits;

		*output = (even_bits & MFM_MASK) | ((odd_bits & MFM_MASK) << 1);
		input++;      /* next 'odd' long and 'even bits' long  */
		output++;     /* next location of the future decoded long */
	}
	return chksum &= MFM_MASK;
}

// Copys the data from inTrack into outTrack but fixes the bit/byte alignment so its aligned on the start of a byte 
void alignSectorToByte(const RawTrackData& inTrack, int byteStart, int bitStart, RawSector& outSector) {
	unsigned char byteOut = 0;
	unsigned int byteOutPosition = 0;

	// The position supplied is the last bit of the track sync.  
	bitStart--;   // goto the next bit
	if (bitStart < 0) {
		// Could do a MEMCPY here, but for now just let the code below run
		bitStart = 7;
		byteStart++;
	}

	// Bit counter output
	unsigned int counter = 0;

	// This is mis-aligned.  So we need to shift the data into byte boundarys
	for (;;) {
		for (int bitCounter = bitStart; bitCounter >= 0; bitCounter--) {
			byteOut <<= 1;
			if (inTrack[byteStart%RAW_TRACKDATA_LENGTH] & (1 << bitCounter)) byteOut |= 1;

			if (++counter >= 8) {
				outSector[byteOutPosition] = byteOut;
				byteOutPosition++;
				if (byteOutPosition >= TRACK_SIZE) return;
				counter = 0;
			}
		}

		// Move along and reset
		byteStart++;
		bitStart = 7;
	}
}

// Attempt to repair the MFM data.  Returns TRUE if errors are detected
bool repairMFMData(unsigned char* data, const unsigned int dataLength) {
	bool errors = false;
	// Only certain bit-patterns are allowed.  So if we come across an invalid one we will try to repair it.  
	// You cannot have two '1's together, and a max of three '0' in a row
	// Allowed combinations:  (note the SYNC WORDS and TRACK START are designed to break these rules, but we should encounter them)
	// 
	//	00010
	//	00100
	//	00101
	//	01000
	//	01001
	//	01010
	//	10001
	//	10010
	//	10100
	//	10101
	//

	unsigned char testByte = 0;
	int counter = 0;
	for (unsigned int position = 0; position < dataLength; position++) {
		for (unsigned char bitIndex = 0; bitIndex < 8; bitIndex++) {
			testByte <<= 1;   // shift off one bit to make room for the new bit
			if (*data & (1 << bitIndex)) {
				// Make sure two '1's dont come in together as this is not allowed! This filters out a lot of BAD combinations
				if ((testByte & 0x2) != 0x2) {
					testByte |= 1;
				} 
				else {
					errors = true;
				}
			}

			// We're only interested in the last so bany bits, and only when we have received that many
			if (++counter > 4) {
				switch (testByte & 0x1F) {
					// These are the only possible invalid combinations left
				case 0x00:
				case 0x01:
				case 0x10:
					// No idea how to repair these	
					errors = true;
					break;
				}
			}


		}
	}

	return errors;
	 
}


// Extract and convert the sector.  This may be a duplicate so we may reject it
void decodeSector(const RawSector& rawSector, const unsigned int trackNumber, const DiskSurface surface, DecodedTrack& decodedTrack) {
	DecodedSector sector;

	// Easier to operate on
	unsigned char* sectorData = (unsigned char*)rawSector;

	// Try to repair any errors on old disks
	if (repairMFMData(sectorData, TRACK_SIZE)) {
		OutputDebugString(L"MFM Errors Detected\n");
	}

	// Read the first 4 bytes (8).  This  is the track header data	
	sector.headerChecksumCalculated = decodeMFMdata((unsigned long*)sectorData, (unsigned long*)&sector, 4);
	// Decode the label data and update the checksum
	sector.headerChecksumCalculated ^= decodeMFMdata((unsigned long*)(sectorData + 8), (unsigned long*)&sector.sectorLabel[0], 16);
	// Get the checksum for the header
	decodeMFMdata((unsigned long*)(sectorData + 40), (unsigned long*)&sector.headerChecksum, 4);  // (computed on mfm longs, longs between offsets 8 and 44 == 2 * (1 + 4) longs)
	// If the header checksum fails we just cant trust anything we received, so we just drop it
	if (sector.headerChecksum != sector.headerChecksumCalculated) {
		return;
	}

	// Check if the header contains valid fields
	if (sector.trackFormat != 0xFF) 
		return;  // not valid
	if (sector.sectorNumber > 10) 
		return;
	if (sector.trackNumber > 162) 
		return;   // 81 tracks * 2 for both sides
	if (sector.sectorsRemaining > 11) 
		return;  // this isnt possible either

	// And is it from the track we expected?
	const unsigned char targetTrackNumber = (trackNumber << 1) | ((surface == DiskSurface::dsUpper) ? 1 : 0);

	if (sector.trackNumber != targetTrackNumber) return; // this'd be weird

	// Get the checksum for the data
	decodeMFMdata((unsigned long*)(sectorData + 48), (unsigned long*)&sector.dataChecksum, 4);

	

	// Lets see if we already have this one
	const int searchSector = sector.sectorNumber;
	auto index = std::find_if(decodedTrack.validSectors.begin(), decodedTrack.validSectors.end(), [searchSector](const DecodedSector& sector) -> bool {
		return (sector.sectorNumber == searchSector);
	});

	// We already have it as a GOOD VALID sector, so skip, we don't need it.
	if (index != decodedTrack.validSectors.end()) return;

	// Decode the data and receive it's checksum
	sector.dataCheckdumCalculated = decodeMFMdata((unsigned long*)(sectorData + 56), (unsigned long*)&sector.data[0], 512); // (from 64 to 1088 == 2*512 bytes)
	
	// See if this already exists in our invalid sectors list
	index = std::find_if(decodedTrack.invalidSectors.begin(), decodedTrack.invalidSectors.end(), [searchSector](const DecodedSector& sector) -> bool {
		return (sector.sectorNumber == searchSector);
	});

	// Is the data valid?
	if (sector.dataChecksum != sector.dataCheckdumCalculated) {
		// If we havent got this one as a bad one, so we'll keep a copy of it
		if (index == decodedTrack.invalidSectors.end()) decodedTrack.invalidSectors.push_back(sector);
	}
	else {
		// Its a good sector, and we dont have it yet
		decodedTrack.validSectors.push_back(sector);

		// Lets delete it from invalid sectors list
		if (index != decodedTrack.invalidSectors.end()) decodedTrack.invalidSectors.erase(index);
	}
}

// Find sectors within raw data read from the drive by searching bit-by-bit for the SYNC bytes
void findSectors(const RawTrackData& track, unsigned int trackNumber, DiskSurface side, WORD trackSync, DecodedTrack& decodedTrack) {
	// Work out what we need to search for which is 2AAAAAAAsyncsync
	//const unsigned long long search = (trackSync | (((DWORD)trackSync) << 16)) | (((long long)0x2AAAAAAA) << 32);

	// Work out what we need to search for which is syncsync
	const unsigned long search = (trackSync | (((DWORD)trackSync) << 16));

	// Prepare our test buffer
	unsigned long decoded = 0;

	// Keep runnign until we run out of data
	unsigned int byteIndex = 0;

	// run the entire track length
	while (byteIndex<RAW_TRACKDATA_LENGTH) {

		// Check each bit, the "decoded" variable slowly slides left providing a 64-bit wide "window" into the bitstream
		for (int bitIndex = 7; bitIndex >= 0; bitIndex--) {
			decoded <<= 1;   // shift off one bit to make room for the new bit
			if (track[byteIndex] & (1 << bitIndex)) decoded |= 1;

			// Have we matched the sync words and the start words? - Early Amiga had a bug with the 0xAAAAAAAA sync pattern
			if ((decoded ) == search) {
				RawSector alignedSector;
				// We extract ALL of the track data from this BIT to byte align it properly, then pass it onto the code to read the sector
				alignSectorToByte(track, byteIndex, bitIndex, alignedSector);

				// Now see if there's a valid sector there				
				decodeSector(alignedSector, trackNumber, side, decodedTrack);

				// We know the size of this buffer, so we can skip by exactly this amount
				byteIndex += TRACK_SIZE; // skip this many bytes as we know this is part of the track!
				if (byteIndex >= RAW_TRACKDATA_LENGTH) break;
			}
		}
		byteIndex++;
	}
}

// Merges any invalid sectors into the valid ones as a last resort
void mergeInvalidSectors(DecodedTrack& track) {
	// So the invalid sectors list do not overlap with the valid ones, so we can just add them on the end
	for (const DecodedSector& sector : track.invalidSectors)
		track.validSectors.push_back(sector);

	// Clear the list
	track.invalidSectors.clear();
}




// Open the device we want to use.  Returns TRUE if it worked
bool ADFWriter::openDevice(const unsigned int comPort) {
	m_phase.reset();
	if (m_device.openPort(comPort) != InterfaceResult::irOK) return false;
	Sleep(100);
	return m_device.enableReading(true, true)==InterfaceResult::irOK;
}

// Close the device when we've finished
void ADFWriter::closeDevice() {
	m_device.closePort();
}

// Analyse phases, this is optional but massivly improves reading speed.  The callback provides progres between 0 and 100%.  Returning FALSE stops the process
AnalysisResult ADFWriter::analyseDisk(std::function<bool(int progress)> callback) {
	if (!m_device.isOpen()) return AnalysisResult::arDriveError;

	// Reset the phase monitor system
	m_phase.reset();

	unsigned int currentTrack = 0;
	// We try reading track 0 with each phase and then sort by which gave us the best results
	if (m_device.selectTrack(currentTrack) != InterfaceResult::irOK) return AnalysisResult::arDriveError;

	// To hold a raw track
	RawTrackData data;
	DecodedTrack track;
	int sectorsFound = 0;

	// Calc all phases
	for (int phaseIndex = 0; phaseIndex < NUMBER_OF_PHASES; phaseIndex++) {

		unsigned int counter = 0;
		const char currentPhase = m_phase.getPhaseAtIndex(phaseIndex);

		// For one track, on both sides
		for (int surfaceIndex = 0; surfaceIndex < 2; surfaceIndex++) {
			const DiskSurface surface = (surfaceIndex == 1) ? DiskSurface::dsUpper : DiskSurface::dsLower;

			if (callback)
				if (!callback((((phaseIndex << 1) | surfaceIndex) * 100) / (NUMBER_OF_PHASES * 2))) return AnalysisResult::arAborted;

			// Switch surface
			if (!m_device.selectSurface(surface)) return AnalysisResult::arDriveError;

			track.invalidSectors.clear();
			track.validSectors.clear();

			// Read from the disk and search for sectors
			if (m_device.readCurrentTrack(currentPhase, data)) {
				if (m_device.trackContainsData(data)) {
					findSectors(data, 0, surface, AMIGA_WORD_SYNC, track);
					sectorsFound += track.validSectors.size();
				}
				else {
					// We'll skip to the next track and try again
					if (surfaceIndex == 0) {
						phaseIndex = -1;
						surfaceIndex = 2;
						counter = 0;
						currentTrack++;
						if (currentTrack > 79) return AnalysisResult::arFailed;
						if (m_device.selectTrack(currentTrack) != InterfaceResult::irOK) return AnalysisResult::arDriveError;
						break;
					}
				}
			} 
			else {
				// Something went wrong
				return AnalysisResult::arDriveError;
			}

			// Count only valid sectord
			counter += track.validSectors.size();
		}

		// Keep score
		m_phase.submitStatistics(currentPhase, counter);
	}
	
	return sectorsFound > 4 ? AnalysisResult::arComplete : AnalysisResult::arFailed;
}

// Reads the disk and write the data to the ADF file supplied.  The callback is for progress, and you can returns FALSE to abort the process
// It is HIGHLY recommended to use analyseDisk() first
ADFResult ADFWriter::writeADF(const std::wstring outputFile, const unsigned int numTracks, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const int retryCounter, const int sectorsFound, const int badSectorsFound)> callback) {
	if (!m_device.isOpen()) return ADFResult::adfrDriveError;

	// Higher than this is not supported
	if (numTracks>82) return ADFResult::adfrDriveError; 

	// Attempt ot open the file
	HANDLE hADFFile = CreateFile(outputFile.c_str(), GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, 0, 0);
	if (hADFFile ==INVALID_HANDLE_VALUE) return ADFResult::adfrFileError;

	// To hold a raw track
	RawTrackData data;
	DecodedTrack track;
	bool includesBadSectors = false;

	// Do all tracks
	for (unsigned int currentTrack = 0; currentTrack < numTracks; currentTrack++) {

		// Select the track we're working on
		if (!m_device.selectTrack(currentTrack)) {
			CloseHandle(hADFFile);
			return ADFResult::adfrCompletedWithErrors;
		}

		// Now select the side
		for (unsigned int surfaceIndex = 0; surfaceIndex < 2; surfaceIndex++) {
			// Surface 
			const DiskSurface surface = (surfaceIndex == 1) ? DiskSurface::dsUpper : DiskSurface::dsLower;

			// Change the surface we're looking at
			if (!m_device.selectSurface(surface)) {
				CloseHandle(hADFFile);
				return ADFResult::adfrCompletedWithErrors;
			}

			// Reset the sectors list
			track.invalidSectors.clear();
			track.validSectors.clear();

			int failCounter = 0;
			int phaseIndex = 0;

			// Optomise order based on history so far
			m_phase.reflectOrder();

			// Extract phase code
			char currentPhase;
			int failureTotal = 0;
			bool ignoreChecksums = false;
			unsigned int totalFails = 0;

			// Repeat until we have all 11 sectors
			while (track.validSectors.size() < 11) {

				if (callback)
					switch (callback(currentTrack, surface, failureTotal/8, track.validSectors.size(), track.invalidSectors.size())) {
					case WriteResponse::wrContinue: break;  // do nothing
					case WriteResponse::wrAbort:    CloseHandle(hADFFile);
						return ADFResult::adfrAborted;

					case WriteResponse::wrSkipBadChecksums: ignoreChecksums = true; failureTotal = 0; break;
					}

				currentPhase = m_phase.getPhaseAtIndex(phaseIndex);

				// Read and process
				if (m_device.readCurrentTrack(currentPhase, data) == InterfaceResult::irOK) {
					unsigned int sectorsBefore = track.validSectors.size();
					findSectors(data, currentTrack, surface, AMIGA_WORD_SYNC, track);
					sectorsBefore = track.validSectors.size() - sectorsBefore;
					// Keep phase anayser ontop! - but we award based on total sectors so far!
					if (sectorsBefore) m_phase.submitStatistics(currentPhase, track.validSectors.size());
				}
				else {
					CloseHandle(hADFFile);
					return ADFResult::adfrDriveError;
				}

				// Every 5 fails, switch disk phase
				failureTotal++;
				if (failCounter++ > 5) {
					// If we fail, we penalise the phase by what was found
					m_phase.submitStatistics(currentPhase, track.validSectors.size());
					phaseIndex = (phaseIndex + 1) % NUMBER_OF_PHASES;
					failCounter = 0;

					// Full phase cycle.
					if (phaseIndex == 0) {
						totalFails++;
						// Failed three times? Ok, lets see hwta we can salvage
						// We switch off the motor, and then switch it basck on, sometimes this jults the drive into getting something we missed
						if (!m_device.enableReading(false, false)) {
							CloseHandle(hADFFile);
							m_device.enableReading(false);
							return ADFResult::adfrDriveError;
						}
						// Wait for a period
						Sleep(100);
						// Turn it back on again
						if (!m_device.enableReading(true, false)) {
							CloseHandle(hADFFile);
							m_device.enableReading(false);
							return ADFResult::adfrDriveError;
						}
					}

					// If the user wants to skip invalid sectors and save them
					if ((ignoreChecksums) || (totalFails>2)) {
						includesBadSectors |= track.invalidSectors.size() > 0;
						mergeInvalidSectors(track);
					}
				}
			}

			// Sort the sectors in order
			std::sort(track.validSectors.begin(), track.validSectors.end(), [](const DecodedSector & a, const DecodedSector & b) -> bool {
				return a.sectorNumber < b.sectorNumber;
			});

			// Now write all of them to disk
			DWORD written;
			for (unsigned int sector = 0; sector < 11; sector++)
				if (!WriteFile(hADFFile, track.validSectors[sector].data, 512, &written, 0)) {
					CloseHandle(hADFFile);
					return ADFResult::adfrFileIOError;
				}
		}
	}

	CloseHandle(hADFFile);

	return includesBadSectors ? ADFResult::adfrCompletedWithErrors : ADFResult::adfrComplete;
}
