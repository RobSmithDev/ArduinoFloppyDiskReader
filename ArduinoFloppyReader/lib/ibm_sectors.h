/* Taken from DiskFlashback, Copyright (C) 2021-2024 Robert Smith (@RobSmithDev)
 * https://robsmithdev.co.uk/diskflashback
 *
 * This file is multi-licensed under the terms of the Mozilla Public
 * License Version 2.0 as published by Mozilla Corporation and the
 * GNU General Public License, version 2 or later, as published by the
 * Free Software Foundation.
 *
 * MPL2: https://www.mozilla.org/en-US/MPL/2.0/
 * GPL2: https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html
 *
 * This file is maintained at https://github.com/RobSmithDev/DiskFlashback
 */

#pragma once


#include <stdint.h>
#include <unordered_map>
#include <map>
#include <vector>

namespace IBM {

	static const uint32_t MaxTrackSize = (0x3A00 * 2);

	typedef std::vector<uint8_t> RawDecodedSector;

	// Structure to hold data while we decode it
	typedef struct {
		uint32_t numErrors;					// Number of decoding errors found
		RawDecodedSector data;          // decoded sector data
	} DecodedSector;

	// To hold a list of valid and checksum failed sectors
	struct DecodedTrack {
		// A map of sector number to valid sectors 
		std::map<int, DecodedSector> sectors;

		uint32_t sectorsWithErrors = 0;
	};


	// Feed in Track 0, sector 0 and this will try to extract the number of sectors per track, or 0 on error
	bool getTrackDetails_IBM(const uint8_t* sector, uint32_t& serialNumber, uint32_t& numHeads, uint32_t& totalSectors, uint32_t& sectorsPerTrack, uint32_t& bytesPerSector);
	
	// Encode the track supplied into a raw MFM bit-stream
	uint32_t encodeSectorsIntoMFM_IBM(const bool isHD, const bool forceAtariTiming, DecodedTrack* decodedTrack, const uint32_t trackNumber, uint32_t mfmBufferSizeBytes, void* trackData);

	// Searches for sectors - you can re-call this and it will update decodedTrack rather than replace it
	// nonstandardTimings is set to true if this uses non-standard timings like those used by Atari etc
	void findSectors_IBM(const uint8_t* track, const uint32_t dataLengthInBits, const bool isHD, const uint32_t trackNumber, const uint32_t expectedNumSectors, DecodedTrack& decodedTrack, bool& nonstandardTimings);

};