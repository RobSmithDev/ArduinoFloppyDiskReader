/* ArduinoFloppyReader (and writer)
*
* Copyright (C) 2017-2022 Robert Smith (@RobSmithDev)
* https://amiga.robsmithdev.co.uk
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
// The class also handles writing an ADF file back to disk and optionally verifying the write.
// 
// The MFM decoding algorithm and information regarding finding the start of a sector
// were taken from the excellent documentation by Laurent Cl�vy at http://lclevy.free.fr/adflib/adf_info.html
// Also credits to Keith Monahan https://www.techtravels.org/tag/mfm/ regarding a bug in the MFM sector start data
//

#include <vector>
#include <algorithm>
#include <assert.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <string>
#include <codecvt>
#include <thread>

// This gets around an issue with the windows header files defining max
const long long StreamMax = std::numeric_limits<std::streamsize>::max();

#include "ADFWriter.h"
#include "ArduinoInterface.h"
#include "RotationExtractor.h"
#ifndef _WIN32
#include <string.h>
#include <cstring>
#endif

#ifdef _WIN32
#include <Windows.h>
#pragma comment(lib,"Ws2_32.lib")
#else
#include <algorithm>
#include <netdb.h>
#endif 

#include "pll.h"

//#include "ftdi.h"
#include "capsapi/Comtype.h"
#include "capsapi/CapsAPI.h"
#include "capsapi/CapsPlug.h"

#include <math.h>


using namespace ArduinoFloppyReader;

#ifndef OUTPUT_TIME_IN_NS
WARNING: OUTPUT_TIME_IN_NS MUST BE DEFINED IN REVOLUTIONEXTRACTOR.H FOR THIS CLASS TO WORK CORRECTLY
#endif


#define MFM_MASK    0x55555555L		
#define AMIGA_WORD_SYNC  0x4489							 // Disk SYNC code for the Amiga start of sector
#define SECTOR_BYTES	512								 // Number of bytes in a decoded sector
#define NUM_SECTORS_PER_TRACK_DD 11						 // Number of sectors per track
#define NUM_SECTORS_PER_TRACK_HD 22						  // Same but for HD disks
#define RAW_SECTOR_SIZE (8+56+SECTOR_BYTES+SECTOR_BYTES)      // Size of a sector, *Including* the sector sync word longs
#define ADF_TRACK_SIZE_DD (SECTOR_BYTES*NUM_SECTORS_PER_TRACK_DD)   // Bytes required for a single track dd
#define ADF_TRACK_SIZE_HD (SECTOR_BYTES*NUM_SECTORS_PER_TRACK_HD)   // Bytes required for a single track hd


const char* TEST_BYTE_SEQUENCE = "amiga.robsmithdev.co.uk";

// Buffer to hold raw data for just a single sector
typedef unsigned char RawEncodedSector[RAW_SECTOR_SIZE];
typedef unsigned char RawDecodedSector[SECTOR_BYTES];
typedef RawDecodedSector RawDecodedTrackDD[NUM_SECTORS_PER_TRACK_DD];
typedef RawDecodedSector RawDecodedTrackHD[NUM_SECTORS_PER_TRACK_HD];
typedef unsigned char RawMFMData[SECTOR_BYTES + SECTOR_BYTES];

// When workbench formats a disk, it write 13630 bytes of mfm data to the disk.  So we're going to write this amount, and then we dont need an erase first
typedef struct alignas(1) {
	unsigned char filler1[1654];  // Padding at the start of the track.  This will be set to 0xaa
	// Raw sector data
	RawEncodedSector sectors[NUM_SECTORS_PER_TRACK_DD];   // 11968 bytes
	// Blank "Filler" gap content. (this may get overwritten by the sectors a little)
	unsigned char filler2[8];
} FullDiskTrackDD;

typedef struct alignas(1) {
	unsigned char filler1[1654];  // Padding at the start of the track.  This will be set to 0xaa
	// Raw sector data
	RawEncodedSector sectors[NUM_SECTORS_PER_TRACK_HD];   // 11968*2 bytes
	// Blank "Filler" gap content. (this may get overwritten by the sectors a little)
	unsigned char filler2[8];
} FullDiskTrackHD;

// Structure to hold data while we decode it
typedef struct alignas(8)  {
	unsigned char trackFormat;        // This will be 0xFF for Amiga
	unsigned char trackNumber;        // Current track number (this is actually (tracknumber*2) + side
	unsigned char sectorNumber;       // The sector we just read (0 to 11)
	unsigned char sectorsRemaining;   // How many more sectors remain until the gap (0 to 10)

	uint32_t sectorLabel[4];     // OS Recovery Data, we ignore this

	uint32_t headerChecksum;	  // Read from the header, header checksum
	uint32_t dataChecksum;		  // Read from the header, data checksum

	uint32_t headerChecksumCalculated;   // The header checksum we calculate
	uint32_t dataChecksumCalculated;     // The data checksum we calculate

	RawDecodedSector data;          // decoded sector data

	RawMFMData rawSector;   // raw track data, for analysis of invalid sectors
} DecodedSector;

// To hold a list of valid and checksum failed sectors
struct DecodedTrack {
	// A list of valid sectors where the checksums are OK
	std::vector<DecodedSector> validSectors;
	// A list of sectors found with invalid checksums.  These are used if ignore errors is triggered
	// We keep copies of each one so we can perform a statistical analysis to see if we can get a working one based on which bits are mostly set the same
	std::vector<DecodedSector> invalidSectors[NUM_SECTORS_PER_TRACK_HD];
};


// MFM decoding algorithm
// *input;	MFM coded data buffer (size == 2*data_size) 
// *output;	decoded data buffer (size == data_size) 
// Returns the checksum calculated over the data
uint32_t decodeMFMdata(const uint32_t* input, uint32_t* output, const unsigned int data_size) {
	uint32_t odd_bits, even_bits;
	uint32_t chksum = 0L;
	unsigned int count;

	// the decoding is made here long by long : with data_size/4 iterations 
	for (count = 0; count < data_size / 4; count++) {
		odd_bits = *input;					// longs with odd bits 
		even_bits = *(uint32_t*)(((unsigned char*)input) + data_size);   // longs with even bits - located 'data_size' bytes after the odd bits

		chksum ^= odd_bits;              // XOR Checksum
		chksum ^= even_bits;

		*output = ((even_bits & MFM_MASK) | ((odd_bits & MFM_MASK) << 1));
		input++;      /* next 'odd' long and 'even bits' long  */
		output++;     /* next location of the future decoded long */
	}
	return chksum & MFM_MASK;
}

// MFM encoding algorithm part 1 - this just writes the actual data bits in the right places
// *input;	RAW data buffer (size == data_size) 
// *output;	MFM encoded buffer (size == data_size*2) 
// Returns the checksum calculated over the data
uint32_t encodeMFMdataPart1(const uint32_t* input, uint32_t* output, const unsigned int data_size) {
	uint32_t chksum = 0L;
	unsigned int count;

	uint32_t* outputOdd = output;
	uint32_t* outputEven = (uint32_t*)(((unsigned char*)output) + data_size);

	// Encode over two passes.  First split out the odd and even data, then encode the MFM values, the /4 is because we're working in longs, not bytes
	for (count = 0; count < data_size / 4; count++) {
		*outputEven = *input & MFM_MASK;
		*outputOdd = ((*input)>>1) & MFM_MASK;
		outputEven++;
		outputOdd++;
		input++;
	}
	
	// Checksum calculator
	// Encode over two passes.  First split out the odd and even data, then encode the MFM values, the /4 is because we're working in longs, not bytes
	for (count = 0; count < (data_size / 4) * 2; count++) {
		chksum ^= *output;
		output++;
	}

	return chksum & MFM_MASK;
}

// Copys the data from inTrack into outTrack but fixes the bit/byte alignment so its aligned on the start of a byte 
void alignSectorToByte(const unsigned char* inTrack, const int dataLength, int byteStart, int bitStart, RawEncodedSector& outSector) {
	unsigned char byteOut = 0;
	unsigned int byteOutPosition = 0;

	// Bit counter output
	unsigned int counter = 0;

	// The position supplied is the last bit of the track sync.  
	bitStart--;   // goto the next bit
	if (bitStart < 0) {
		// Could do a MEMCPY here, but for now just let the code below run
		bitStart = 7;
		byteStart++;
	}
	byteStart -= 8;   // wind back 8 bytes

	// This is mis-aligned.  So we need to shift the data into byte boundarys
	for (;;) {
		for (int bitCounter = bitStart; bitCounter >= 0; bitCounter--) {
			byteOut <<= 1;
			if (inTrack[byteStart % dataLength] & (1 << bitCounter)) byteOut |= 1;

			if (++counter >= 8) {
				outSector[byteOutPosition] = byteOut;
				byteOutPosition++;
				if (byteOutPosition >= RAW_SECTOR_SIZE) return;
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
	// Allowed combinations:  (note the SYNC WORDS and TRACK START are designed to break these rules, but we shouldn't encounter them)
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
		// Fixed: This was the wrong way around
		for (int bitIndex = 7; bitIndex >= 0; bitIndex--) {
			testByte <<= 1;   // shift off one bit to make room for the new bit
			if (*data & (1 << bitIndex)) {
				// Make sure two '1's dont come in together as this is not allowed! This filters out a lot of BAD combinations
				if ((testByte & 0x2) != 0x2) {
					testByte |= 1;
				} 
				else {
					// We detected two '1's in a row, which isnt allowed.  From reading this most likely means this was a weak bit, so we change it to zero.
					errors = true;
				}
			} 

			// We're only interested in the last so many bits, and only when we have received that many
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
		data++;
	}

	return errors;
	 
}

// Looks at the history for this sector number and creates a new sector where the bits are set to whatever occurs more.  We then do a checksum and if it succeeds we use it
bool attemptFixSector(const DecodedTrack& decodedTrack, DecodedSector& outputSector) {
	int sectorNumber = outputSector.sectorNumber;

	if (decodedTrack.invalidSectors[sectorNumber].size() < 2) return false;

	typedef struct {
		int zeros = 0;
		int ones = 0;
	} SectorCounter[8];

	SectorCounter sectorSum[SECTOR_BYTES + SECTOR_BYTES];

	memset(sectorSum, 0, sizeof(sectorSum));

	// Calculate the number of '1's and '0's in each block
	for (const DecodedSector& sec : decodedTrack.invalidSectors[sectorNumber]) 
		for (int byteNumber = 0; byteNumber < SECTOR_BYTES + SECTOR_BYTES; byteNumber++) 
			for (int bit = 0; bit <= 7; bit++) 
				if (sec.rawSector[byteNumber] & (1 << bit))
					sectorSum[byteNumber][bit].ones++; else sectorSum[byteNumber][bit].zeros++;

	// Now create a sector based on this data
	memset(outputSector.rawSector, 0, sizeof(outputSector.rawSector));
	for (int byteNumber = 0; byteNumber < SECTOR_BYTES + SECTOR_BYTES; byteNumber++)
		for (int bit = 0; bit <= 7; bit++)
			if (sectorSum[byteNumber][bit].ones >= sectorSum[byteNumber][bit].zeros)
				outputSector.rawSector[byteNumber] |= (1 << bit);

	return true;
}

// Extract and convert the sector.  This may be a duplicate so we may reject it.  Returns TRUE if it was valid, or false if not
bool decodeSector(const RawEncodedSector& rawSector, const unsigned int trackNumber, bool isHD, const DiskSurface surface, DecodedTrack& decodedTrack, bool ignoreHeaderChecksum, int& lastSectorNumber) {
	DecodedSector sector;

	lastSectorNumber = -1;
	memcpy(sector.rawSector, rawSector, sizeof(RawMFMData));

	// Easier to operate on
	unsigned char* sectorData = (unsigned char*)rawSector;
 
	// Read the first 4 bytes (8).  This  is the track header data	
	sector.headerChecksumCalculated = decodeMFMdata((uint32_t*)(sectorData + 8), (uint32_t*)&sector, 4);
	// Decode the label data and update the checksum
	sector.headerChecksumCalculated ^= decodeMFMdata((uint32_t*)(sectorData + 16), (uint32_t*)&sector.sectorLabel[0], 16);
	// Get the checksum for the header
	decodeMFMdata((uint32_t*)(sectorData + 48), (uint32_t*)&sector.headerChecksum, 4);  // (computed on mfm longs, longs between offsets 8 and 44 == 2 * (1 + 4) longs)
	// If the header checksum fails we just cant trust anything we received, so we just drop it
	if ((sector.headerChecksum != sector.headerChecksumCalculated) && (!ignoreHeaderChecksum)) {
		return false;
	}

	// Check if the header contains valid fields
	if (sector.trackFormat != 0xFF) 
		return false;  // not valid
	if (sector.sectorNumber > (isHD ? 21 : 10))
		return false;
	if (sector.trackNumber > 166) 
		return false;   // 83 tracks * 2 for both sides
	if (sector.sectorsRemaining > (isHD ? 22 : 11))
		return false;  // this isnt possible either
	if (sector.sectorsRemaining < 1)
		return false;  // or this

	// And is it from the track we expected?
	const unsigned char targetTrackNumber = (trackNumber << 1) | ((surface == DiskSurface::dsUpper) ? 1 : 0);

	if (sector.trackNumber != targetTrackNumber) return false; // this'd be weird

	// Get the checksum for the data
	decodeMFMdata((uint32_t*)(sectorData + 56), (uint32_t*)&sector.dataChecksum, 4);
	

	// Lets see if we already have this one
	const int searchSector = sector.sectorNumber;
	auto index = std::find_if(decodedTrack.validSectors.begin(), decodedTrack.validSectors.end(), [searchSector](const DecodedSector& sector) -> bool {
		return (sector.sectorNumber == searchSector);
	});

	// We already have it as a GOOD VALID sector, so skip, we don't need it.
	if (index != decodedTrack.validSectors.end()) return true;

	// Decode the data and receive it's checksum
	sector.dataChecksumCalculated = decodeMFMdata((uint32_t*)(sectorData + 64), (uint32_t*)&sector.data[0], SECTOR_BYTES); // (from 64 to 1088 == 2*512 bytes)

	lastSectorNumber = sector.sectorNumber;

	// Is the data valid?
	if (sector.dataChecksum != sector.dataChecksumCalculated) {
		// Keep a copy
		decodedTrack.invalidSectors[sector.sectorNumber].push_back(sector);
		return false;
	}
	else {
		// Its a good sector, and we dont have it yet
		decodedTrack.validSectors.push_back(sector);

		// Lets delete it from invalid sectors list
		decodedTrack.invalidSectors[sector.sectorNumber].clear();


		return true;
	}
}

// Encode a sector into the correct format for disk
void encodeSector(const unsigned int trackNumber, const DiskSurface surface, bool isHD, const unsigned int sectorNumber, const RawDecodedSector& input, RawEncodedSector& encodedSector, unsigned char& lastByte) {
	// Sector Start
	encodedSector[0] = (lastByte & 1) ? 0x2A : 0xAA;
	encodedSector[1] = 0xAA;
	encodedSector[2] = 0xAA;
	encodedSector[3] = 0xAA;
	// Sector Sync
	encodedSector[4] = 0x44;
	encodedSector[5] = 0x89;
	encodedSector[6] = 0x44;
	encodedSector[7] = 0x89;

	// MFM Encoded header
	DecodedSector header;
	memset(&header, 0, sizeof(header));

	header.trackFormat = 0xFF;
	header.trackNumber = (trackNumber << 1) | ((surface == DiskSurface::dsUpper) ? 1 : 0);
	header.sectorNumber = sectorNumber; 
	header.sectorsRemaining = (isHD ? NUM_SECTORS_PER_TRACK_HD : NUM_SECTORS_PER_TRACK_DD) - sectorNumber;  //1..11
	
	
	header.headerChecksumCalculated = encodeMFMdataPart1((const uint32_t*)&header, (uint32_t*)&encodedSector[8], 4);
	// Then theres the 16 bytes of the volume label that isnt used anyway
	header.headerChecksumCalculated ^= encodeMFMdataPart1((const uint32_t*)&header.sectorLabel, (uint32_t*)&encodedSector[16], 16);
	// Thats 40 bytes written as everything doubles (8+4+4+16+16). - Encode the header checksum
	encodeMFMdataPart1((const uint32_t*)&header.headerChecksumCalculated, (uint32_t*)&encodedSector[48], 4);
	// And move on to the data section.  Next should be the checksum, but we cant encode that until we actually know its value!
	header.dataChecksumCalculated = encodeMFMdataPart1((const uint32_t*)&input, (uint32_t*)&encodedSector[64], SECTOR_BYTES);
	// And add the checksum
	encodeMFMdataPart1( (const uint32_t*)&header.dataChecksumCalculated, (uint32_t*)&encodedSector[56], 4);

	// Now fill in the MFM clock bits
	bool lastBit = encodedSector[7] & (1 << 0);
	bool thisBit = lastBit;

	// Clock bits are bits 7, 5, 3 and 1
	// Data is 6, 4, 2, 0
	for (int count = 8; count < RAW_SECTOR_SIZE; count++) {
		for (int bit = 7; bit >= 1; bit -= 2) {
			lastBit = thisBit;			
			thisBit = encodedSector[count] & (1 << (bit-1));
	
			if (!(lastBit || thisBit)) {
				// Encode a 1!
				encodedSector[count] |= (1 << bit);
			}
		}
	}

	lastByte = encodedSector[RAW_SECTOR_SIZE - 1];
}

// Find sectors within raw data read from the drive by searching bit-by-bit for the SYNC bytes
void findSectors(const unsigned char* track, bool isHD, unsigned int trackNumber, DiskSurface side, unsigned short trackSync, DecodedTrack& decodedTrack, bool ignoreHeaderChecksum) {
	// Work out what we need to search for which is syncsync
	const uint32_t search = (trackSync | (((uint32_t)trackSync) << 16));

	// Prepare our test buffer
	uint32_t decoded = 0;

	// Keep runnign until we run out of data
	unsigned int byteIndex = 0;

	int nextTrackBitCount = 0;

	const unsigned int dataLength = isHD ? RAW_TRACKDATA_LENGTH_HD : RAW_TRACKDATA_LENGTH_DD;
	const int maxSectors = isHD ? NUM_SECTORS_PER_TRACK_HD : NUM_SECTORS_PER_TRACK_DD;

	// run the entire track length
	while (byteIndex < dataLength) {

		// Check each bit, the "decoded" variable slowly slides left providing a 32-bit wide "window" into the bitstream
		for (int bitIndex = 7; bitIndex >= 0; bitIndex--) {
			decoded <<= 1;   // shift off one bit to make room for the new bit

			if (track[byteIndex] & (1 << bitIndex)) decoded |= 1;

			// Have we matched the sync words correctly
			++nextTrackBitCount;
			int lastSectorNumber = -1;
			if (decoded == search) {
				RawEncodedSector alignedSector;
				
				// We extract ALL of the track data from this BIT to byte align it properly, then pass it onto the code to read the sector (from the start of the sync code)
				alignSectorToByte(track, dataLength, byteIndex, bitIndex, alignedSector);

				// Now see if there's a valid sector there.  We now only skip the sector if its valid, incase rogue data gets in there
				if (decodeSector(alignedSector, trackNumber, isHD, side, decodedTrack, ignoreHeaderChecksum, lastSectorNumber)) {
					// We know the size of this buffer, so we can skip by exactly this amount
					byteIndex += RAW_SECTOR_SIZE - 8; // skip this many bytes as we know this is part of the track! minus 8 for the SYNC
					if (byteIndex >= dataLength) break;
					// We know that 8 bytes from here should be another track. - we allow 1 bit either way for slippage, but this allows an extra check incase the SYNC pattern is damaged
					nextTrackBitCount = 0;
				}
				else {

					// Decode failed.  Lets try a "homemade" one
					DecodedSector newTrack;
					if ((lastSectorNumber >= 0) && (lastSectorNumber < maxSectors)) {
						newTrack.sectorNumber = lastSectorNumber;
						if (attemptFixSector(decodedTrack, newTrack)) {
							memcpy(newTrack.rawSector, alignedSector, sizeof(newTrack.rawSector));
							// See if our makeshift data will decode or not
							if (decodeSector(alignedSector, trackNumber, isHD, side, decodedTrack, ignoreHeaderChecksum, lastSectorNumber)) {
								// We know the size of this buffer, so we can skip by exactly this amount
								byteIndex += RAW_SECTOR_SIZE - 8; // skip this many bytes as we know this is part of the track! minus 8 for the SYNC
								if (byteIndex >= dataLength) break;
							}
						}
					}
					if (decoded == search) nextTrackBitCount = 0;
				}
			}
		}
		byteIndex++;
	}
}

// Merges any invalid sectors into the valid ones as a last resort
void mergeInvalidSectors(DecodedTrack& track, bool isHD) {
	const int maxSectors = isHD ? NUM_SECTORS_PER_TRACK_HD : NUM_SECTORS_PER_TRACK_DD;

	for (unsigned char sector = 0; sector < maxSectors; sector++) {
		if (track.invalidSectors[sector].size()) {
			// Lets try to make the best sector we can
			DecodedSector sec = track.invalidSectors[sector][0];
			// Repair maybe!?
			attemptFixSector(track, sec);

			track.validSectors.push_back(sec);
		}
		track.invalidSectors[sector].clear();
	}
}

ADFWriter::ADFWriter() {
#ifdef _WIN32
	// Start winsock
	WSADATA wsadata;
	WSAStartup(MAKEWORD(2, 0), &wsadata);
#endif
}

ADFWriter::~ADFWriter() {
#ifdef _WIN32
	WSACleanup();
#endif
	// Incase it was used
	CapsExit();
}

// Open the device we want to use.  Returns TRUE if it worked
bool ADFWriter::openDevice(const std::wstring& portName) {
	if (m_device.openPort(portName) != DiagnosticResponse::drOK) return false;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	if (m_device.enableReading(true, true) != DiagnosticResponse::drOK) {
		m_device.closePort();
		return false;
	}
	return true;
}

// Close the device when we've finished
void ADFWriter::closeDevice() {
	m_device.closePort();
}

// Run diagnostics on the system.  You do not need to call openDevice first.  Return TRUE if everything passed
bool ADFWriter::runDiagnostics(const std::wstring& portName, std::function<void(bool isError, const std::string message)> messageOutput, std::function<bool(bool isQuestion, const std::string question)> askQuestion) {
	std::stringstream msg;
	if (!messageOutput) return false;
	if (!askQuestion) return false;


	unsigned int major=0, minor=0, rev=0;

	msg << "Attempting to open and use ";
	char convert[256];
#ifdef _WIN32	
	sprintf_s(convert, "%ls", portName.c_str());
#else
	sprintf(convert, "%ls", portName.c_str());
#endif
	msg << convert << " without CTS";
	messageOutput(false,msg.str());

	// Step 1 is to check the com port stuff
	DiagnosticResponse r = m_device.openPort(portName, false);
	
	// Check response
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true,m_device.getLastErrorStr());
		messageOutput(true, "Please check Arduino power and the TXD/RXD on the USB->Serial board are connected to the right pins on the Arduino");
		return false;
	}

	// Step 2: This has been setup with CTS stuff disabled, but the port opened.  So lets validate CTS *is* working as we expect it to
	messageOutput(false, "Testing CTS pin");
	r = m_device.testCTS();
	
	// Check response
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		messageOutput(true, "Please check the following PINS on the Arduino: A2");
		return false;
	}
	
	// Step 3: CTS is working correctly, so re-open the port in normal mode
	messageOutput(false, "CTS OK.  Reconnecting with CTS enabled");
	m_device.closePort();
	r = m_device.openPort(portName, true);

	// Check response
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		return false;
	}

	FirmwareVersion version = m_device.getFirwareVersion();
	if ((version.major < 1) || (version.major > 9)) {
		messageOutput(true, "Error reading firmware version. Something is wrong with the communication between the PC and the Arduino.");
		return false;
	}

	char buffer[250];
#ifdef _WIN32	
	sprintf_s(buffer, "Board is running firmware version %i.%i.%i %s%s\n", version.major, version.minor, version.buildNumber, (version.deviceFlags1 & FLAGS_FIRMWARE_BETA)?"beta ":"", version.fullControlMod ? " (modded for DiskChange support)" : "");
#else	
	sprintf(buffer, "Board is running firmware version %i.%i.%i %s%s\n", version.major, version.minor, version.buildNumber, (version.deviceFlags1 & FLAGS_FIRMWARE_BETA) ? "beta " : "", version.fullControlMod ? " (modded for DiskChange support)" : "");
#endif
	messageOutput(false, buffer);

	
	messageOutput(false, "Fetching latest firmware version...");

	// Fetch version from 'A' record in the DNS record
	hostent* address = gethostbyname("drawbridge-firmware-amiga.robsmithdev.co.uk");
	if ((address) && (address->h_addrtype == AF_INET)) {
		if (address->h_addr_list[0] != 0) {
			in_addr add = *((in_addr*)address->h_addr_list[0]);
#ifdef _WIN32			
			major = add.S_un.S_un_b.s_b1;
			minor = add.S_un.S_un_b.s_b2;
			rev = add.S_un.S_un_b.s_b3;
#else
			uint32_t bytes = htonl(add.s_addr);
			major = bytes >> 24;
			minor = (bytes >> 16) & 0xFF;
			rev = (bytes >> 8) & 0xFF;
#endif			
		}
	}
	if (major) {
		if ((major > version.major) ||
			((major == version.major) && (minor > version.minor)) ||
			((major == version.major) && (minor == version.minor) && (rev > version.buildNumber))) {
			messageOutput(false, "");
			messageOutput(false, ">>> WARNING: This firmware is out of date - Please update it!");
			char latestFirmwareVersion[128] = { 0 };
#ifdef _WIN32
			sprintf_s(latestFirmwareVersion, ">>> Latest version is V%i.%i.%i", major, minor, rev);
#else
			sprintf(latestFirmwareVersion, ">>> Latest version is V%i.%i.%i", major, minor, rev);
#endif
			messageOutput(false, latestFirmwareVersion);
			messageOutput(false, "");
		} else 
			messageOutput(false,"Firmware is up-to-date");
	} else 
	if ((version.major == 1) && (version.minor < 9)) {
		messageOutput(false, "This firmware is out of date.  Please update it!");
	}

	if ((version.major > 1) || ((version.major == 1) && (version.minor >= 9))) {
		messageOutput(false, "Features Set: ");

		if (version.fullControlMod || (version.deviceFlags1 & FLAGS_DISKCHANGE_SUPPORT)) messageOutput(false, "      o Disk Change Pin Support");
		if (version.deviceFlags1 & FLAGS_DRAWBRIDGE_PLUSMODE) messageOutput(false, "      o DrawBridge PLUS"); else messageOutput(false, "      o DrawBridge Classic");
		if (version.deviceFlags1 & FLAGS_DENSITYDETECT_ENABLED) messageOutput(false, "      o HD/DD Detect Enabled");
		if (version.deviceFlags1 & FLAGS_SLOWSEEKING_MODE) messageOutput(false, "      o Slow Seeking Mode Enabled");
		if (version.deviceFlags1 & FLAGS_INDEX_ALIGN_MODE) messageOutput(false, "      o Force Index Alignment on Writes Enabled");
		if (version.deviceFlags1 & FLAGS_FLUX_READ) messageOutput(false, "      o Accurate Flux Read Mode"); else
		if (version.deviceFlags1 & FLAGS_HIGH_PRECISION_SUPPORT) messageOutput(false, "      o Higher Accuracy SCP Mode");
		if (version.deviceFlags1 & FLAGS_FIRMWARE_BETA) messageOutput(false, "      o Beta Firmware");
	}

	if ((version.major > 1) || ((version.major == 1) && (version.minor >= 8))) {
		messageOutput(false, "Testing USB->Serial transfer speed (read)");
		if (m_device.testTransferSpeed() != DiagnosticResponse::drOK) {
			messageOutput(false, "The USB->Serial adapter in use is not suitable.");
			messageOutput(false, "Arduino UNO: The on-board adapter is not able to sustain large data transfers");
			messageOutput(false, "Arduino Pro mini: The USB-Serial board is not able to sustain large data transfers");
			messageOutput(false, "Arduino Nano: The USB-Serial on this board is not fast enough to sustain large data transfers");
			messageOutput(false, "");
			messageOutput(false, "If you are using any of the devices with a CH340 converter then swap it for an FTDI one.");
			messageOutput(false, "If you still have problems after switching, connect the Arduino using a shorter cable");
			messageOutput(false, "and if possible directly (ie: not via a USB hub)");
			messageOutput(true, "Diagnostics failed.");
			return false;
		}
		messageOutput(false, "Read speed test passed.  USB to serial converter is functioning correctly!");
	}


	if ((version.major > 1) || ((version.major == 1) && (version.minor >= 8))) {
		messageOutput(false, "Testing write-protect signal");
		for (;;) {
		
			if (m_device.checkIfDiskIsWriteProtected(true) == DiagnosticResponse::drWriteProtected) break;

			if (!askQuestion(false, "Inserted disk is not write protected. If it is, then check Arduino Pin A1. Please insert a write protected AmigaDOS disk in the drive")) {
				messageOutput(true, "Diagnostics aborted");
				return false;
			}
		}
	}

	// Request a disk
	if (!askQuestion(false, "Please insert a *write protected* disk in the drive that was ideally not written using this tool.\r\nUse a disk that you don't mind being erased.\nThis disk must contain data/formatted as an AmigaDOS disk")) {
		messageOutput(true, "Diagnostics aborted");
		return false;
	}

	// Functions to test
	messageOutput(false, "Enabling the drive (please listen and watch the drive)");
	r = m_device.enableReading(true, false);
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		return false;
	}

	// Ask the user for verification of the result
	if (!askQuestion(true,"Did the floppy disk start spinning, and did the drive LED switch on?")) {
		messageOutput(true, "Please check the drive has power and the following PINS on the Arduino: 5 and 11");
		messageOutput(true, "It's possible you may also need need to provide a seperate 5V power supply for this drive.");
		return false;
	}

	// Check INDEX as its required for PPM calculation
	messageOutput(false, "Checking for INDEX pulse from drive");
	r = m_device.testIndexPulse();
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		messageOutput(true, "Please check that a disk is inserted and the following PINS on the Arduino: 2");
		return false;
	}

	// RPM
	if ((version.major > 1) || ((version.major == 1) && (version.minor >= 9))) {
		messageOutput(false, "Measuring Drive RPM...");
		float rpm = 0;
		r = m_device.measureDriveRPM(rpm);
		if (r == DiagnosticResponse::drNoDiskInDrive) {
			messageOutput(false, "WARNING: Disk not detected in drive.");
		}
		else {
			if (r != DiagnosticResponse::drOK) {
				messageOutput(true, m_device.getLastErrorStr());
				return false;
			}

#ifdef _WIN32	
			sprintf_s(buffer, "Drive RPM measured as %0.2f RPM\n", rpm);
#else	
			sprintf(buffer, "Drive RPM measured as %0.2f RPM\n", rpm);
#endif
			messageOutput(false, buffer);
			if ((rpm < 280) || (rpm > 320)) {
				messageOutput(true, "Drive RPM should be 300, your drive is out of acceptable tolerance!");
				messageOutput(true, "I recommend trying again with a different disk.");
				messageOutput(true, "If this problem persists then you either have a bad batch of disks");
				messageOutput(true, "that are slowing the drive motor down, or the drive motor needs re-calibrating.");
				messageOutput(true, "This can also be a sign that the drive has just worn out and should not be used.");
				return false;
			}
			else {
				if ((rpm < 295) || (rpm > 305)) {
					messageOutput(false, "WARNING: Drive RPM should be 300, your drive is a little out of tune.");
					messageOutput(false, "I recommend trying again with a different disk!");
					messageOutput(false, "If this problem persists then you either have a bad batch of disks");
					messageOutput(false, "or you need to re-calibrate your drive motor.");
				}
			}
		}


		messageOutput(false, "Checking board type...");
		bool isPlus = false;
		r = m_device.guessPlusMode(isPlus);
		if (r != DiagnosticResponse::drOK) {
			messageOutput(true, m_device.getLastErrorStr());
			messageOutput(true, "Failed checking the board type.  Check firmware.");
			return false;
		}

		// See what type the board claims to be
		bool isPlusReally = false;
		r = m_device.eeprom_IsDrawbridgePlusMode(isPlusReally);
		if (r != DiagnosticResponse::drOK) {
			messageOutput(true, m_device.getLastErrorStr());
			messageOutput(true, "Failed checking the board type setting.  Check firmware.");
			return false;
		}

		if (isPlus != isPlusReally) {
			if (isPlus) {
				messageOutput(false, "Board was detected as DrawBridge PLUS but is configured as DrawBridge Classic.");
				if (askQuestion(true, "Board was detected as DrawBridge PLUS but is configured as DrawBridge Classic.\nWould you like the board configuring as DrawBridge Plus?")) {
					if (m_device.eeprom_SetDrawbridgePlusMode(true) != DiagnosticResponse::drOK) {
						messageOutput(true, m_device.getLastErrorStr());
						messageOutput(true, "Failed setting the board type setting.  Check firmware.");
						return false;
					}
				}
			}
			else {
				messageOutput(false, "Board was detected as DrawBridge Classic but is configured as DrawBridge PLUS.");
				if (askQuestion(true, "Board was detected as DrawBridge Classic but is configured as DrawBridge PLUS.\nWould you like the board configuring as DrawBridge Classic?")) {
					if (m_device.eeprom_SetDrawbridgePlusMode(false) != DiagnosticResponse::drOK) {
						messageOutput(true, m_device.getLastErrorStr());
						messageOutput(true, "Failed setting the board type setting.  Check firmware.");
						return false;
					}
				}
			}
		}
		else {
			messageOutput(false, "Board EEPROM setting matches detected board type.");
		}
	}
	
	// Now see if we can find track 0
	messageOutput(false, "Asking the Arduino to find Track 0");
	r = m_device.findTrack0();
	if (r == DiagnosticResponse::drRewindFailure) {
		messageOutput(true, "Unable to find track 0");
		if (askQuestion(true,"Could you hear the drive head moving?")) {
			messageOutput(true, "Please check the following PINS on the Arduino: 6, 8");
		}
		else {
			messageOutput(true, "Please check the following PINS on the Arduino: 7");
			messageOutput(true, "It's possible you may also need need to provide a seperate 5V power supply for this drive.");
		}
		return false;
	}
	else
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		return false;
	}

	// Track 0 found.  Lets see if we can seek to track 70
	messageOutput(false, "Track 0 was found.  Asking the Arduino to find Track 70");
	r = m_device.selectTrack(70);
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		messageOutput(true, "There is also a strong possability that you may also need need to provide a seperate 5V power supply for this drive.");
		return false;
	}

	if (!askQuestion(true,"Could you hear the head moving quite a distance?")) {
        messageOutput(true, "As we successfully found track 0, please check the following PINS on the Arduino: 6, 7");
		messageOutput(true, "It's possible you may also need need to provide a seperate 5V power supply for this drive.");
		return false;
	}

	// So we can open the drive and move the head.  We're going to turn the drive off
	r = m_device.enableReading(false, false);
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		return false;
	}


	// Test DiskChange
	if (version.fullControlMod) {
		// Goto track 3, just like the Amiga
		m_device.selectTrack(3);

		std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
		messageOutput(false, "Testing Disk Change pin.");
		messageOutput(false, "*** Please remove disk from drive *** (you have 30 seconds)");
		while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < 30000) {
			std::this_thread::sleep_for(std::chrono::milliseconds(250));
			if (m_device.checkForDisk(true) == DiagnosticResponse::drNoDiskInDrive) break;
		}

		if (m_device.isDiskInDrive()) {
			messageOutput(true, "Disk change is NOT working correctly");
			messageOutput(true, "Please check the following pins on the Arduino: 10, 11");
			return false;
		}

		start = std::chrono::steady_clock::now();
		messageOutput(false, "*** Please re-insert disk into drive *** (you have 30 seconds)");
		while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < 30000) {
			std::this_thread::sleep_for(std::chrono::milliseconds(2000));
			m_device.selectTrack(2);
			if (m_device.isDiskInDrive()) break;
			std::this_thread::sleep_for(std::chrono::milliseconds(2000));
			m_device.selectTrack(3);
			if (m_device.isDiskInDrive()) break;
		}

		if (m_device.isDiskInDrive()) {
			messageOutput(false, "Disk change is working correctly");
		}
		else {
			messageOutput(true, "Disk change is NOT working correctly");
			messageOutput(true, "Please check the following pins on the Arduino: 10, 11");
			return false;
		}		

	}

	messageOutput(false, "Starting drive, and seeking to track 40.");
	// Re-open the drive
	r = m_device.enableReading(true, true);
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		return false;
	}

	// Goto 40, this is where the root block is, although we wont be decoding it
	r = m_device.selectTrack(40);
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		return false;
	}

	bool isHDDrive = false;

	// See what type of disk it is
	if ((version.major > 1) || ((version.major == 1) && (version.minor >= 9))) {
		bool ishd = false;
		if (m_device.checkDiskCapacity(ishd) == DiagnosticResponse::drOK) {
			isHDDrive = ishd;
			if (ishd) messageOutput(false, "Disk inserted was detected as HIGH DENSITY"); else messageOutput(false, "Disk inserted was detected as DOUBLE DENSITY");
			if (m_device.setDiskCapacity(isHDDrive) != DiagnosticResponse::drOK) {
				messageOutput(true, "Unable to change density mode of the drive");
				messageOutput(true, m_device.getLastErrorStr());
				return false;
			}
		}
	}

	messageOutput(false, "Checking for DATA from drive");
	r = m_device.testDataPulse();
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		messageOutput(true, "Please check that a disk is inserted and following PINS on the Arduino: 4 (and the 1K resistor)");
		return false;
	}

	r = m_device.selectSurface(ArduinoFloppyReader::DiskSurface::dsUpper);
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		return false;
	}

	messageOutput(false, "Attempting to read a track from the UPPER side of the disk");
	ArduinoFloppyReader::RawTrackDataHD* data = (ArduinoFloppyReader::RawTrackDataHD*)malloc(sizeof(ArduinoFloppyReader::RawTrackDataHD));
	int counter;
	bool tracksFound = false;
	const unsigned int readSize = isHDDrive ? sizeof(ArduinoFloppyReader::RawTrackDataHD) : sizeof(ArduinoFloppyReader::RawTrackDataDD);
	const unsigned int maxSectorsPerTrack = isHDDrive ? NUM_SECTORS_PER_TRACK_HD : NUM_SECTORS_PER_TRACK_DD;

	if (!data) {
		messageOutput(true, "Error, out of memory");
		return false;
	}

	bool badRead = false; 

	for (int a = 0; a < 10; a++) {
		r = m_device.readCurrentTrack(data, readSize, true);
		if (r != DiagnosticResponse::drOK) {
			messageOutput(true, m_device.getLastErrorStr());
			free(data);
			return false;
		}

		// Data read.  See if any tracks were detected
		DecodedTrack trk1;
		findSectors(*data, readSize, 40, ArduinoFloppyReader::DiskSurface::dsUpper, AMIGA_WORD_SYNC, trk1, true);
		
		counter=0; 
		for (unsigned int sec=0; sec< maxSectorsPerTrack; sec++)
			if (trk1.invalidSectors[sec].size()) counter++;

		if (counter + trk1.validSectors.size() > 0) {
			messageOutput(false, "Tracks found!");
			tracksFound = true;
			break;
		}
		// Nothing found?
		DecodedTrack trk2;
		findSectors(*data, readSize, 40, ArduinoFloppyReader::DiskSurface::dsLower, AMIGA_WORD_SYNC, trk2, true);
		
		counter = 0;
		for (unsigned int sec = 0; sec< maxSectorsPerTrack; sec++)
			if (trk2.invalidSectors[sec].size()) counter++;

		if (counter + trk2.validSectors.size() > 0) {
			messageOutput(false, "Tracks found but on the wrong side.  Please check the following PINS on the Arduino: 9");
			free(data);
			return false;
		}
		if (tracksFound) break;
	}

	if (!tracksFound) {
		badRead = true;

		messageOutput(true, "Failed to detect valid data on the current disk on Track 40, Upper Side");
		if (!askQuestion(true, "Failed to detect valid data on Track 40, upper side.\nIf you are sure the disk is OK then there may be a drive head alignment issue.\nThis can be confirmed by performing the write test.\n\nDo you want to continue?")) {
			free(data);
			return false;
		}
	}

	messageOutput(false, "Attempting to read a track from the LOWER side of the disk");
	r = m_device.selectSurface(ArduinoFloppyReader::DiskSurface::dsLower);
	if (r != DiagnosticResponse::drOK) {
		free(data);
		messageOutput(true, m_device.getLastErrorStr());
		return false;
	}
	tracksFound = false;

	for (int a = 0; a < 10; a++) {
		r = m_device.readCurrentTrack(data, readSize, true);
		if (r != DiagnosticResponse::drOK) {
			free(data);
			messageOutput(true, m_device.getLastErrorStr());
			return false;
		}

		// Data read.  See if any tracks were detected
		DecodedTrack trk1;
		findSectors(*data, readSize, 40, ArduinoFloppyReader::DiskSurface::dsLower, AMIGA_WORD_SYNC, trk1, true);

		counter = 0;
		for (unsigned int sec = 0; sec< maxSectorsPerTrack; sec++)
			if (trk1.invalidSectors[sec].size()) counter++;

		if (counter + trk1.validSectors.size() > 0) {
			messageOutput(false, "Tracks found!");
			tracksFound = true;
			break;
		}
		// Nothing found?
		DecodedTrack trk2;
		findSectors(*data, readSize, 40, ArduinoFloppyReader::DiskSurface::dsUpper, AMIGA_WORD_SYNC, trk2, true);

		counter = 0;
		for (unsigned int sec = 0; sec< maxSectorsPerTrack; sec++)
			if (trk2.invalidSectors[sec].size()) counter++;

		if (counter + trk2.validSectors.size() > 0) {
			messageOutput(false, "Tracks found but on the wrong side.  Please check the following PINS on the Arduino: 9");
			free(data);
			return false;
		}
		if (tracksFound) break;
	}

	if (!tracksFound) {
		badRead = true;

		messageOutput(true, "Failed to detect valid data on the current disk on Track 40, Lower Side");
		if (!askQuestion(true, "Failed to detect valid data on Track 40, lower side.\nIf you are sure the disk is OK then there may be a drive head alignment issue.\nThis can be confirmed by performing the write test.\n\nDo you want to continue?")) {
			free(data);
			return false;
		}
	}

	if (!badRead) messageOutput(false, "Reading was successful!"); else messageOutput(false, "Reading errors occured");

	// Turn the drive off again
	r = m_device.enableReading(false,false);
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		free(data);
		return false;
	}

	// Now ask.
	if (!askQuestion(true,"Would you like to test writing to a disk?  Please insert a WRITE ENABLED disk that you *do not mind* being overwritten")) {
		messageOutput(true, "Diagnostic aborted.");
		free(data);
		return false;
	}

	// Try to enable the write head
	do {
		r = m_device.enableWriting(true, true);

		if (r == DiagnosticResponse::drWriteProtected) {
			if (!askQuestion(false,"Please write-enable the disk and try again.")) {
				messageOutput(true, "Diagnostic aborted.");
				free(data);
				return false;
			}
		} else
		if (r != DiagnosticResponse::drOK) {
			messageOutput(true, m_device.getLastErrorStr());
			free(data);
			return false; 
		}

	} while (r == DiagnosticResponse::drWriteProtected);
	// Writing is enabled.

	if ((version.major > 1) || ((version.major == 1) && (version.minor >= 8))) {
		for (;;) {

			if (m_device.checkIfDiskIsWriteProtected(true) == DiagnosticResponse::drOK) break;

			if (!askQuestion(false, "Inserted disk is write protected. If it isn't, then check Arduino Pin A1. Please insert a write *enabled* disk in the drive")) {
				messageOutput(true, "Diagnostics aborted");
				free(data);
				return false;
			}
		}
	}

	
	// Goto 41, we'll write some stuff
	r = m_device.selectTrack(41);
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		free(data);
		return false;
	}

	
	// We're gonna write a test track out
	RawDecodedSector track[NUM_SECTORS_PER_TRACK_HD];
	FullDiskTrackDD disktrackDD;
	FullDiskTrackHD disktrackHD;
	memset(disktrackDD.filler1, 0xAA, sizeof(disktrackDD.filler1));  // Pad with "0"s which as an MFM encoded byte is 0xAA
	memset(disktrackDD.filler2, 0xAA, sizeof(disktrackDD.filler2));  // Pad with "0"s which as an MFM encoded byte is 0xAA
	memset(disktrackHD.filler1, 0xAA, sizeof(disktrackHD.filler1));  // Pad with "0"s which as an MFM encoded byte is 0xAA
	memset(disktrackHD.filler2, 0xAA, sizeof(disktrackHD.filler2));  // Pad with "0"s which as an MFM encoded byte is 0xAA

	// Get length
	int sequenceLength = strlen(TEST_BYTE_SEQUENCE);

	// Attempt to write, with verify
	bool eraseOK = false;
	bool writtenOK = false;
	for (int side = 0; side < 4; side++) {
		switch (side) {
		case 0:messageOutput(false, "Writing and Verifying Track 41 (Lower Side)..."); break;
		case 1:messageOutput(false, "Writing and Verifying Track 41 (Upper Side)..."); break;
		case 2:messageOutput(false, "Writing and Verifying Track 41 Precomp Mode (Lower Side)..."); break;
		case 3:messageOutput(false, "Writing and Verifying Track 41 Precomp Mode (Upper Side)..."); break;
		}

		ArduinoFloppyReader::DiskSurface currentSurface = ((side & 1) != 0) ? ArduinoFloppyReader::DiskSurface::dsUpper : ArduinoFloppyReader::DiskSurface::dsLower;
		r = m_device.selectSurface(currentSurface);
		if (r != DiagnosticResponse::drOK) {
			messageOutput(true, m_device.getLastErrorStr());
			free(data);
			return false;
		}

		// Prepare encoding
		unsigned char lastByte = 0;
		for (unsigned int sector = 0; sector < maxSectorsPerTrack; sector++) {

			// Invent some track data
			for (int bytePos = 0; bytePos < SECTOR_BYTES; bytePos++)
				track[sector][bytePos] = TEST_BYTE_SEQUENCE[(sector + bytePos) % sequenceLength];

			if (isHDDrive)
				 encodeSector(41, side ? ArduinoFloppyReader::DiskSurface::dsUpper : ArduinoFloppyReader::DiskSurface::dsLower, isHDDrive, sector, track[sector], disktrackHD.sectors[sector], lastByte); 
			else encodeSector(41, side ? ArduinoFloppyReader::DiskSurface::dsUpper : ArduinoFloppyReader::DiskSurface::dsLower, isHDDrive, sector, track[sector], disktrackDD.sectors[sector], lastByte);

		}

		// Many loops
		for (int a = 1; a <= 10; a++) {

			// Erase the track
			r = m_device.eraseCurrentTrack();
			if (r != DiagnosticResponse::drOK) {
				messageOutput(true, m_device.getLastErrorStr());
				messageOutput(true, "This can also be caused by insufficient power to the drive");
				free(data);
				return false;
			}

			// read it back
			r = m_device.readCurrentTrack(data, readSize, false);
			if (r != DiagnosticResponse::drOK) {
				messageOutput(true, m_device.getLastErrorStr());
				messageOutput(true, "This can also be caused by insufficient power to the drive");
				free(data);
				return false;
			}

			{
				// See if theres any data
				eraseOK = true;

				// Data read.  See if any tracks were detected
				DecodedTrack trk;
				findSectors(*data, isHDDrive, 41, ArduinoFloppyReader::DiskSurface::dsUpper, AMIGA_WORD_SYNC, trk, true);
				// Have a look at any of the found sectors and see if any are valid and matched the phrase we wrote onto the disk
				for (const DecodedSector& sec : trk.validSectors) {
					// See if we can find the sequence in here somewhere 
					std::string s;
					s.resize(SECTOR_BYTES);
					memcpy(&s[0], sec.data, SECTOR_BYTES);

					if (s.find(TEST_BYTE_SEQUENCE) != std::string::npos) {
						// Excellent
						eraseOK = false;
						break;
					}
				}
			}

			// shouldnt have found anything
			if (eraseOK) {
				if (isHDDrive) {
					if (side > 1)
						r = m_device.writeCurrentTrackPrecomp((const unsigned char*)(&disktrackHD), sizeof(disktrackHD), false, true);
					else r = m_device.writeCurrentTrack((const unsigned char*)(&disktrackHD), sizeof(disktrackHD), false);
				}
				else {
					if (side > 1)
						r = m_device.writeCurrentTrackPrecomp((const unsigned char*)(&disktrackDD), sizeof(disktrackDD), false, true);
					else r = m_device.writeCurrentTrack((const unsigned char*)(&disktrackDD), sizeof(disktrackDD), false);
				}

				if (r != DiagnosticResponse::drOK) {
					messageOutput(true, m_device.getLastErrorStr());
					messageOutput(true, "This can also be caused by insufficient power to the drive");
					free(data);
					return false;
				}

				r = m_device.readCurrentTrack(*data, readSize, false);
				if (r != DiagnosticResponse::drOK) {
					messageOutput(true, m_device.getLastErrorStr());
					messageOutput(true, "This can also be caused by insufficient power to the drive");
					free(data);
					return false;
				}

				// Data read.  See if any tracks were detected
				DecodedTrack trk;
				findSectors(*data, readSize, 41, currentSurface, AMIGA_WORD_SYNC, trk, true);
				// Have a look at any of the found sectors and see if any are valid and matched the phrase we wrote onto the disk
				for (const DecodedSector& sec : trk.validSectors) {
					// See if we can find the sequence in here somewhere 
					std::string s;
					s.resize(SECTOR_BYTES);
					memcpy(&s[0], sec.data, SECTOR_BYTES);

					if (s.find(TEST_BYTE_SEQUENCE) != std::string::npos) {
						// Excellent
						writtenOK = true;
						break;
					}
				}
				if (!writtenOK) {
					// See if we can find the sequence in one of the partial sectors
					for (unsigned int sector = 0; sector < maxSectorsPerTrack; sector++) {
						for (const DecodedSector& sec : trk.invalidSectors[sector]) {
							// See if we can find the sequence in here somewhere 
							std::string s;
							s.resize(SECTOR_BYTES);
							memcpy(&s[0], sec.data, SECTOR_BYTES);

							if (s.find(TEST_BYTE_SEQUENCE) != std::string::npos) {
								// Excellent
								writtenOK = true;
								break;
							}
						}
						if (writtenOK) break;
					}
					if (writtenOK) break;
				}
				if (writtenOK) break;
			}			
		}
	}

	free(data);

	// Final results
	if ((!writtenOK) || (!eraseOK)) {
		if (eraseOK) messageOutput(true, "Unable to detect written track.  This could be for one of the following reasons:"); else messageOutput(true, "Unable to erase a track.  This could be for one of the following reasons:"); 
		messageOutput(true, "1.  Please check the following PINS on the Arduino: 3, A0, A1");
		messageOutput(true, "2.  Please check the Arduino IDE config has not been modified from stock.  This was tested using 1.8.4, compiler settings may affect performance");
		messageOutput(true, "3.  Check for poor connections, typically on a breadboard they may be intermittent which may pass the above results but still not work.");
		messageOutput(true, "4.  Check for an electrically noisy environment.  It is possible that electronic noise (eg: from a cell phone) may cause errors reading and writing to the disk");
		messageOutput(true, "5.  Shorten the floppy disk cable to help reduce noise.");
		messageOutput(true, "6.  Ensure your power supply is strong enough to power the floppy drive.  This drive may need too much power for the USB port");
		messageOutput(true, "7.  You can contact me for help, but some basic electronics diagnostics will help, checkout YouTube for guides.");
		if (badRead) messageOutput(true, "8.  Reading also failed, so this may be the first area to look. Check Arduino pin 4 (and the 1K resistor).");
		messageOutput(false, "Please join my Discord server at https://discord.gg/HctVgSFEXu");

		m_device.enableWriting(false);
		return false;
	}

	if (badRead) {
		messageOutput(false, "The good news is your Arduino/DrawBridge board is working correctly.");
		messageOutput(false, "The bad news is that your drive looks like it has a head-alignment issue.");
		messageOutput(false, "It is unlikely this drive will be able to read any disks inserted and");
		messageOutput(false, "any disks written will probably not read on another drive or an Amiga.");
		messageOutput(false, "I recommend you try another drive or investigate how to re-align the drive head.");
		messageOutput(false, "Please join my Discord server at https://discord.gg/HctVgSFEXu");
		return false;
	}
	else {
		messageOutput(false, "Hurray! Writing was successful.  Your Arduino is ready for use! - Send us a photo");
		messageOutput(false, "or join my Discord server at https://discord.gg/HctVgSFEXu");
	}

	m_device.enableWriting(false);
	return true;
}

// Get the current firmware version.  Only valid if openPort is successful
const FirmwareVersion ADFWriter::getFirwareVersion() const {
	return m_device.getFirwareVersion(); 
};

class TrackMemoryUsed {
public:
	RawDecodedTrackHD* trackHD;
	FullDiskTrackHD* disktrackHD;
	FullDiskTrackDD* disktrackDD;

	TrackMemoryUsed() {
		trackHD = (RawDecodedTrackHD*)malloc(sizeof(RawDecodedTrackHD));
		disktrackHD = (FullDiskTrackHD*)malloc(sizeof(FullDiskTrackHD));
		disktrackDD = (FullDiskTrackDD*)malloc(sizeof(FullDiskTrackDD));
		memset(disktrackDD, 0xAA, sizeof(FullDiskTrackDD));  // Pad with "0"s which as an MFM encoded byte is 0xAA
		memset(disktrackHD, 0xAA, sizeof(FullDiskTrackHD));  // Pad with "0"s which as an MFM encoded byte is 0xAA
	}
	~TrackMemoryUsed() {
		free(trackHD);
		free(disktrackHD);
		free(disktrackDD);
	}
};

// Writes an ADF file back to a floppy disk.  Return FALSE in the callback to abort this operation 
// IF using precomp mode then DO NOT connect the Arduino via a USB hub, and try to plug it into a USB2 port
ADFResult ADFWriter::ADFToDisk(const std::wstring& inputFile, bool mediaIsHD, bool verify, bool usePrecompMode, bool eraseFirst, bool writeFromIndex, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const bool isVerifyError, const CallbackOperation operation) > callback) {
	if (!m_device.isOpen()) return ADFResult::adfrDriveError;

	if (callback)
		if (callback(0,DiskSurface::dsLower, false, CallbackOperation::coStarting) == WriteResponse::wrAbort) return ADFResult::adfrAborted;

	// Upgrade to writing mode
	if (m_device.enableWriting(true, true)!=DiagnosticResponse::drOK) return ADFResult::adfrDriveError;

	ArduinoFloppyReader::FirmwareVersion version = m_device.getFirwareVersion();
	bool supportsFluxErase = ((version.major > 1) || ((version.major == 1) && (version.minor == 9) && (version.buildNumber >= 18)));

	// Attempt to open the file
#ifdef _WIN32	
	std::ifstream hADFFile(inputFile, std::ifstream::in | std::ifstream::binary);
#else
	std::string inputFileA;
	quickw2a(inputFile,inputFileA);
	std::ifstream hADFFile(inputFileA, std::ifstream::in | std::ifstream::binary);
#endif
	if (!hADFFile.is_open()) return ADFResult::adfrFileError;

	// find file size
	hADFFile.ignore(StreamMax);
	std::streamsize fileLength = hADFFile.gcount();
	hADFFile.clear();   //  Since ignore will have set eof.
	hADFFile.seekg(0, std::ios_base::beg);

	// See if theres a header
	char buffer[9];
	hADFFile.read(buffer, 8);
	buffer[8] = '\0';
	if (strcmp(buffer, "UAE--ADF") == 0) {
		return ADFResult::adfrExtendedADFNotSupported;
	}
	hADFFile.seekg(0, std::ios_base::beg);

	const int DD_Max_Disk_Size = sizeof(RawDecodedTrackDD) * 84 * 2;  //shouldn't go above 84

	if (mediaIsHD) {
		if (fileLength <= DD_Max_Disk_Size) {
			return ADFResult::adfrMediaSizeMismatch;
		}
	}
	else {
		if (fileLength > DD_Max_Disk_Size) {
			FirmwareVersion version = m_device.getFirwareVersion();
			if ((version.major > 1) || ((version.major == 1) && (version.minor >= 9))) {
				return ADFResult::adfrMediaSizeMismatch;
			} else {
				return ADFResult::adfrFirmwareTooOld;
			}			
		}
	}

	if (m_device.setDiskCapacity(mediaIsHD) != DiagnosticResponse::drOK) {
		return ADFResult::adfrAborted;
	}

	if (m_device.checkForDisk(true) == DiagnosticResponse::drNoDiskInDrive) {
		return ADFResult::adfrDriveError;
	}

	unsigned int currentTrack = 0;
	unsigned int surfaceIndex = 0;

	// Buffer to hold the track
	TrackMemoryUsed tracks;

	// Just make sure nothing weird is going on
	assert(sizeof(RawDecodedTrackDD) == ADF_TRACK_SIZE_DD);
	assert(sizeof(RawDecodedTrackHD) == ADF_TRACK_SIZE_HD);
	bool errors = false;

	const unsigned int AdfTrackSize = mediaIsHD ? ADF_TRACK_SIZE_HD : ADF_TRACK_SIZE_DD;
	const unsigned int maxSectorsPerTrack = mediaIsHD ? NUM_SECTORS_PER_TRACK_HD : NUM_SECTORS_PER_TRACK_DD;


	while (hADFFile.good()) {
		hADFFile.read((char*)tracks.trackHD, AdfTrackSize);
		std::streamsize bytesRead = hADFFile.gcount();

		// Stop if we didnt read a full track
		if (bytesRead != (std::streamsize)AdfTrackSize) break;

		// Select the track we're working on
		if (m_device.selectTrack(currentTrack)!= DiagnosticResponse::drOK) {
			hADFFile.close();
			return ADFResult::adfrDriveError;
		}

		DiskSurface surface = (surfaceIndex == 1) ? DiskSurface::dsUpper : DiskSurface::dsLower;
		// Change the surface we're targeting
		if (m_device.selectSurface(surface) != DiagnosticResponse::drOK) {
			hADFFile.close();
			return ADFResult::adfrDriveError;
		}

		// Handle callback
		if (callback)
			if (callback(currentTrack, surface, false, CallbackOperation::coReadingFile) == WriteResponse::wrAbort) {
				hADFFile.close();
				return ADFResult::adfrAborted;
			}

		 
		unsigned char lastByte;
		unsigned int dataToWrite;
		unsigned char* dataToWritePtr;
		// Now encode the sector into the output buffer
		if (mediaIsHD) {
			lastByte = tracks.disktrackHD->filler1[sizeof(tracks.disktrackHD->filler1) - 1];

			for (unsigned int sector = 0; sector < maxSectorsPerTrack; sector++)
				encodeSector(currentTrack, surface, mediaIsHD, sector, (*tracks.trackHD)[sector], tracks.disktrackHD->sectors[sector], lastByte);

			if (lastByte & 1) tracks.disktrackHD->filler2[7] = 0x2F; else tracks.disktrackHD->filler2[7] = 0xFF;
			dataToWrite = sizeof(FullDiskTrackHD) - (writeFromIndex ? (sizeof(tracks.disktrackHD->filler1)-2) : 0);
			dataToWritePtr = writeFromIndex ? &tracks.disktrackHD->filler1[sizeof(tracks.disktrackHD->filler1) - 2] : (unsigned char*)tracks.disktrackHD;
		}
		else {
			lastByte = tracks.disktrackDD->filler1[sizeof(tracks.disktrackDD->filler1) - 1];

			for (unsigned int sector = 0; sector < maxSectorsPerTrack; sector++)
				encodeSector(currentTrack, surface, mediaIsHD, sector, (*tracks.trackHD)[sector], tracks.disktrackDD->sectors[sector], lastByte);

			if (lastByte & 1) tracks.disktrackDD->filler2[7] = 0x2F; else tracks.disktrackDD->filler2[7] = 0xFF;

			dataToWrite = sizeof(FullDiskTrackDD) - (writeFromIndex ? (sizeof(tracks.disktrackDD->filler1) - 2) : 0);
			dataToWritePtr = writeFromIndex ? &tracks.disktrackDD->filler1[sizeof(tracks.disktrackDD->filler1) - 2] : (unsigned char*)tracks.disktrackDD;
		}

		// Keep looping until it wrote correctly
		DecodedTrack trackRead;
		trackRead.validSectors.clear();
		for (unsigned int a = 0; a < maxSectorsPerTrack; a++) trackRead.invalidSectors[a].clear();


		int failCount = 0;
		while (trackRead.validSectors.size()< maxSectorsPerTrack) {

			if (eraseFirst) {
				// Run the erase cycle twice
				m_device.eraseCurrentTrack();
				if (supportsFluxErase) m_device.eraseFluxOnTrack(); else m_device.eraseCurrentTrack();
				m_device.eraseCurrentTrack();
			} else
				if (writeFromIndex) 
					m_device.eraseCurrentTrack();

			if (callback)
				if (callback(currentTrack, surface, false, failCount > 0 ? CallbackOperation::coRetryWriting : CallbackOperation::coWriting) == WriteResponse::wrAbort) {
					hADFFile.close();
					return ADFResult::adfrAborted;
				}

			DiagnosticResponse resp;
			resp = m_device.writeCurrentTrackPrecomp(dataToWritePtr, dataToWrite, writeFromIndex, (currentTrack >= 40) && usePrecompMode);
			if (resp == DiagnosticResponse::drOldFirmware) resp = m_device.writeCurrentTrack(dataToWritePtr, dataToWrite, writeFromIndex);

			switch (resp) {
			case DiagnosticResponse::drWriteProtected:	hADFFile.close();
														return ADFResult::adfrDiskWriteProtected;
			case DiagnosticResponse::drOK: break;
			default: hADFFile.close();
					 return ADFResult::adfrDriveError;
			}

			if (verify) {	
				if (callback)
					if (callback(currentTrack, surface, false, CallbackOperation::coVerifying) == WriteResponse::wrAbort) {
						hADFFile.close();
						return ADFResult::adfrAborted;
					}

				for (int retries=0; retries<10; retries++) {
					RawTrackDataHD data;
					// Read the track back
					if (m_device.readCurrentTrack(data, mediaIsHD ? sizeof(RawTrackDataHD) : sizeof(RawTrackDataDD), false) == DiagnosticResponse::drOK) {
						// Find hopefully all sectors
						findSectors(data, mediaIsHD, currentTrack, surface, AMIGA_WORD_SYNC, trackRead, false);
					}
					if (trackRead.validSectors.size() == maxSectorsPerTrack) break;

					if (callback) 
						if (callback(currentTrack, surface, false, CallbackOperation::coReVerifying) == WriteResponse::wrAbort) {
							hADFFile.close();
							return ADFResult::adfrAborted;
						}
				} 

				// So we found all sectors, but were they the ones we actually wrote!?
				if (trackRead.validSectors.size() == maxSectorsPerTrack) {
					int sectorsGood = 0;
					for (unsigned int sector = 0; sector < maxSectorsPerTrack; sector++) {
						auto index = std::find_if(trackRead.validSectors.begin(), trackRead.validSectors.end(), [sector](const DecodedSector& sectorfound) -> bool {
							return (sectorfound.sectorNumber == sector);
						});

						// We found this sector.
						if (index != trackRead.validSectors.end()) {
							DecodedSector& rec = trackRead.validSectors[index - trackRead.validSectors.begin()];
							if (memcmp(rec.data, (*tracks.trackHD)[sector], SECTOR_BYTES) == 0) {
								sectorsGood++;  // this one matches on read!
							}
						}
					}
					if ((unsigned int)sectorsGood != maxSectorsPerTrack) {
						// Something went wrong, so we clear them all so it gets reported as an error
						trackRead.validSectors.clear();
					}
				}


				// We failed to verify this track.
				if (trackRead.validSectors.size() < maxSectorsPerTrack) {
					failCount++;
					if (failCount >= 5) {
						if (!callback) break;
						bool breakOut = false;

						switch (callback(currentTrack, surface, true, CallbackOperation::coReVerifying)) {
						case WriteResponse::wrAbort: hADFFile.close();
							return ADFResult::adfrAborted;
						case WriteResponse::wrSkipBadChecksums: breakOut = true; errors = true; break;
						case WriteResponse::wrContinue: break;
						default: break;
						}
						if (breakOut) break;
						failCount = 0;
					}
				}
			}
			else break;
		}

		// Goto the next surface/track
		surfaceIndex++;
		if (surfaceIndex > 1) {
			surfaceIndex = 0;
			currentTrack++;
		}
		// But there is a physical limit
		if (currentTrack > 83) break; 
	}

	return errors? ADFResult::adfrCompletedWithErrors: ADFResult::adfrComplete;
}

#pragma pack(1) 

/* Taken from https://www.cbmstuff.com/downloads/scp/scp_image_specs.txt
This information is copyright(C) 2012 - 2020 By Jim Drew. Permission is granted
for inclusion with any source code when keeping this copyright notice.
*/
struct SCPFileHeader {
	char			headerSCP[3];
	unsigned char	version;
	unsigned char	diskType;
	unsigned char	numRevolutions;
	unsigned char	startTrack;
	unsigned char   endTrack;
	unsigned char	flags;
	unsigned char	bitcellEncoding;   // 0=16 bits per sample, 
	unsigned char	numHeads;
	unsigned char   timeBase;          // Resolution. Time in ns = (timeBase+1)*25
	uint32_t	checksum;
};

struct SCPTrackHeader {
	char			headerTRK[3];
	unsigned char	trackNumber;
};

struct SCPTrackRevolution {
	uint32_t	indexTime;		// Time in NS/25 for this revolution
	uint32_t	trackLength;	// Number of bit-cells in this revolution
	uint32_t	dataOffset;		// From the start of SCPTrackHeader 
};

// Track data is 16-bit value in NS/25.  If =0 means no flux transition for max time 
#pragma pack()

#define BITFLAG_INDEX		0
#define BITFLAG_96TPI		1
#define BITFLAG_NORMALISED  3
#define BITFLAG_EXTENDED    6
#define BITFLAG_FLUXCREATOR 7

typedef std::vector<uint16_t> SCPTrackData;

struct SCPTrackInMemory {
	SCPTrackHeader header;
	std::vector<SCPTrackRevolution> revolution;
	std::vector<SCPTrackData> revolutionData;
}; 

// Reads the disk and write the data to the SCP file supplied.  The callback is for progress, and you can returns FALSE to abort the process
// numTracks is the number of tracks to read.  Usually 80 (0..79), sometimes track 80 and 81 are needed. revolutions is hwo many revolutions of the disk to save (1-5)
// SCP files are a low level flux record of the disk and usually can backup copy protected disks to.  Without special hardware they can't usually be written back to disks.
ADFResult ADFWriter::DiskToSCP(const std::wstring& outputFile, bool isHDMode, const unsigned int numTracks, const unsigned char revolutions, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const int retryCounter, const int sectorsFound, const int badSectorsFound, const int maxSectors, const CallbackOperation operation)> callback, bool useNewFluxReader) {
	if (!m_device.isOpen()) return ADFResult::adfrDriveError;

	if (callback)
		if (callback(0, DiskSurface::dsLower, 0,0,0,0, CallbackOperation::coStarting) == WriteResponse::wrAbort) return ADFResult::adfrAborted;

	FirmwareVersion v = m_device.getFirwareVersion();
	if ((v.major == 1) && (v.minor < 8)) return  ADFResult::adfrFirmwareTooOld;

	if (m_device.setDiskCapacity(isHDMode) != DiagnosticResponse::drOK) {
		return ADFResult::adfrAborted;
	}

	// Higher than this is not supported
	if (numTracks > 84) return ADFResult::adfrDriveError;
	if (revolutions < 1) return ADFResult::adfrDriveError;
	if (revolutions > 5) return ADFResult::adfrDriveError;

	// Attempt ot open the file
#ifdef _WIN32		
	std::fstream hADFFile = std::fstream(outputFile, std::ofstream::out | std::ofstream::in | std::ofstream::binary | std::ofstream::trunc);
#else
	std::string outputFileA;
	quickw2a(outputFile,outputFileA);
	std::fstream hADFFile = std::fstream(outputFileA, std::ofstream::out | std::ofstream::in | std::ofstream::binary | std::ofstream::trunc);
#endif	
	if (!hADFFile.is_open()) return ADFResult::adfrFileError;
	 
	SCPFileHeader header;
	header.headerSCP[0] = 'S';
	header.headerSCP[1] = 'C';
	header.headerSCP[2] = 'P';
	header.version = 0;// 2 | (2 << 4);    // CHECK
	header.diskType = 0x04; // amiga 0x80;
	header.numRevolutions = revolutions;
	header.startTrack = 0;
	header.numHeads = 0;  // both heads
	header.timeBase = 0;  // 25ns
	header.endTrack = (numTracks*2) - 1;
	header.flags = (1 << BITFLAG_INDEX) | (isHDMode?(1 << BITFLAG_NORMALISED):0) | (1 << BITFLAG_96TPI) | (1 << BITFLAG_FLUXCREATOR);
	header.bitcellEncoding = 0; // 16-bit
	// to be calculated
	header.checksum = 0;

	assert(sizeof(SCPFileHeader) == 16);

	SCPTrackInMemory track;
	track.header.headerTRK[0] = 'T';
	track.header.headerTRK[1] = 'R';
	track.header.headerTRK[2] = 'K';

	try {
		hADFFile.write((const char*)&header, sizeof(header));
	} catch (...) {
			hADFFile.close();
		return ADFResult::adfrFileIOError;
	}

	// Pad out the records.  Theres 4 bytes for each track
	uint32_t notPresent = 0;
	for (unsigned int a = 0; a <168; a++) {
		try {
			hADFFile.write((const char*)&notPresent, sizeof(notPresent));
		} catch (...) {
			hADFFile.close();
			return ADFResult::adfrFileIOError;
		}
	}

	RotationExtractor::MFMSample samples[RAW_TRACKDATA_LENGTH_HD];
	RotationExtractor extractor;
	extractor.setAlwaysUseIndex(true);

	// Do all tracks
	for (unsigned int currentTrack = 0; currentTrack < numTracks; currentTrack++) {

		// Select the track we're working on
		if (m_device.selectTrack(currentTrack) != DiagnosticResponse::drOK) {
			hADFFile.close();
			return ADFResult::adfrCompletedWithErrors;
		}

		// Now select the side
		for (unsigned int surfaceIndex = 0; surfaceIndex < 2; surfaceIndex++) {

			// Surface 
			const DiskSurface surface = (surfaceIndex == 1) ? DiskSurface::dsUpper : DiskSurface::dsLower;
			track.revolution.clear();
			track.revolutionData.clear();
			track.header.trackNumber = (currentTrack * 2) + surfaceIndex;

			PLL::BridgePLL pll(false, false);
			pll.setRotationExtractor(&extractor);

			// Change the surface we're looking at
			if (m_device.selectSurface(surface) != DiagnosticResponse::drOK) {
				hADFFile.close();
				return ADFResult::adfrCompletedWithErrors;
			}

			if (callback) {
				switch (callback(currentTrack, surface, 0, 0, 0, 0, CallbackOperation::coReading)) {
				case WriteResponse::wrContinue: break;  // do nothing
				case WriteResponse::wrAbort:    hADFFile.close();
												return ADFResult::adfrAborted;
				default: break;
				}
			}

			// Read in the data in 'raw' mode
			RotationExtractor::IndexSequenceMarker startPatterns;
			SCPTrackRevolution currentRev;
			currentRev.indexTime = 0;
			currentRev.trackLength = 0;

			pll.reset();
			extractor.reset(isHDMode);

			SCPTrackData currentRevData;

			std::function<bool(RotationExtractor::MFMSample** mfmData, const unsigned int dataLengthInBits)> callbackFunction = 
				[this, &track, &currentRev, &currentRevData, revolutions, isHDMode](RotationExtractor::MFMSample** _mfmData, unsigned int dataLengthInBits)->bool {
					if (track.revolution.size() >= revolutions) return false;

					RotationExtractor::MFMSample* mfmData = *_mfmData;
					unsigned int currentTime = 0;
					
					for (unsigned int a = 0; a < dataLengthInBits; a++) {
						const unsigned int bit = 7 - (a & 7);

						currentTime += isHDMode ? ((unsigned int)mfmData->bittime[bit] / 2) : ((unsigned int)mfmData->bittime[bit]);

						// Bit found?
						if (mfmData->mfmData & (1 << bit)) {

							// Convert to 25ns times
							currentTime /= 25;

							// Keep track of time
							currentRev.indexTime += currentTime; 

							// Handle data too big
							while (currentTime > 65535) {
								currentRevData.push_back(0);
								currentTime -= 65536;
							}

							// Save
							currentRevData.push_back((unsigned short)(((currentTime & 0xFF) << 8) | ((currentTime >> 8) & 0xFF)));
							currentRev.trackLength++;

							// Reset
							currentTime = 0;
						}
							
						// Skip to next bit of data
						if (bit == 0) mfmData++;
					}

					track.revolution.push_back(currentRev);
					track.revolutionData.push_back(currentRevData);

					currentRev.indexTime = 0;
					currentRev.trackLength = 0;
					currentRevData.clear();

					// Stop when we have enough data
					return track.revolution.size() < revolutions;
				};

			for (unsigned int retries = 0; retries <= revolutions; retries ++) {
				if (useNewFluxReader) {
					if (m_device.readFlux(pll, RAW_TRACKDATA_LENGTH_HD, samples, startPatterns, callbackFunction) == DiagnosticResponse::drOK)
						break;
				}
				else {
					if (m_device.readRotation(*pll.rotationExtractor(), RAW_TRACKDATA_LENGTH_HD, samples, startPatterns, callbackFunction, true) == DiagnosticResponse::drOK)
						break;
				}
			}
			if (track.revolution.size() < revolutions) 
				return ADFResult::adfrDriveError;

			// Get the current position
			uint32_t currentPosition = (uint32_t)hADFFile.tellp();

			// Move to the beginning of the file, and write the offset for where this starts
			hADFFile.seekp(sizeof(header) + (track.header.trackNumber * 4), std::fstream::beg);
			try {
				hADFFile.write((const char*)&currentPosition, 4);
			} catch (...) {
				hADFFile.close();
				return ADFResult::adfrFileIOError;
			}

			// Restore position and save data
			hADFFile.seekp(currentPosition, std::fstream::beg);

			// Write the header
			try {
				hADFFile.write((const char*)&track.header, sizeof(track.header));
			} catch (...) {
				hADFFile.close();
				return ADFResult::adfrFileIOError;
			}

			// Write out the revolution headers
			unsigned int dataPos = sizeof(track.header) + (track.revolution.size() * sizeof(SCPTrackRevolution));
			for (unsigned int a = 0; a < track.revolution.size(); a++) {
				track.revolution[a].dataOffset = dataPos;
				try {
					hADFFile.write((const char*)&track.revolution[a], sizeof(track.revolution[a]));
				} catch (...) {
					hADFFile.close();
					return ADFResult::adfrFileIOError;
				}
				dataPos += track.revolutionData[a].size() * 2;
			}

			// Now write out the data
			for (unsigned int a = 0; a < track.revolutionData.size(); a++) {
				try {
					hADFFile.write((const char*)track.revolutionData[a].data(), track.revolutionData[a].size() * 2);
				} catch (...) {
					hADFFile.close();
					return ADFResult::adfrFileIOError;
				}
			}
		}
	}
		
	// Compute the checksum
	hADFFile.seekg(sizeof(SCPFileHeader), std::fstream::beg);
	unsigned char buffer[256];
	header.checksum = 0;

	while (hADFFile.good()) {
		try {
			hADFFile.read((char*)buffer, sizeof(buffer));
			const std::streamsize read = hADFFile.gcount();
			for (size_t pos = 0; pos < (size_t)read; pos++)
				header.checksum += buffer[pos];
		}
		catch (...) {			
		}		
	}
	hADFFile.clear();
	hADFFile.seekp(0, std::fstream::beg);

	// Write the header again with the checksum in it
	try {
		hADFFile.write((const char*)&header, sizeof(header));
	} catch (...) {
		hADFFile.close();
		return ADFResult::adfrFileIOError;
	}

	hADFFile.close();

	return ADFResult::adfrComplete;
}

// Writes an SCP file back to a floppy disk.  Return FALSE in the callback to abort this operation.  
ADFResult ADFWriter::SCPToDisk(const std::wstring& inputFile, bool extraErases, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const bool isVerifyError, const CallbackOperation operation) > callback) {
	if (!m_device.isOpen()) return ADFResult::adfrDriveError;
	ArduinoFloppyReader::FirmwareVersion version = m_device.getFirwareVersion();
	if ((version.major == 1) && ((version.minor < 9) || ((version.minor == 9) && (version.buildNumber < 22)))) return ADFResult::adfrFirmwareTooOld;

	m_device.checkForDisk(true);
	if (!m_device.isDiskInDrive()) return ADFResult::adfrDriveError;

	// Attempt ot open the file
#ifdef _WIN32		
	std::fstream hADFFile = std::fstream(inputFile, std::ofstream::in | std::ofstream::binary);
#else
	std::string inputFileA;
	quickw2a(inputFile, inputFileA);
	std::fstream hADFFile = std::fstream(inputFileA, std::ofstream::in | std::ofstream::binary);
#endif	
	if (!hADFFile.is_open()) return ADFResult::adfrFileError;
	assert(sizeof(SCPFileHeader) == 16);
	// Try to read the header

	SCPFileHeader header;
	try {
		hADFFile.read((char*)&header, sizeof(header));
	}
	catch (...) {
		hADFFile.close();
		return ADFResult::adfrFileIOError;
	}

	// Get the drive RPM spin speed
	if (callback)
		if (callback(0, DiskSurface::dsLower, false, CallbackOperation::coStarting) == WriteResponse::wrAbort) return ADFResult::adfrAborted;
	float driveRPM = 300.0f;
#ifndef _DEBUG
	if (m_device.measureDriveRPM(driveRPM) != DiagnosticResponse::drOK) return ADFResult::adfrDriveError;
#else
	driveRPM = 301;
#endif

	// Validate the format that we support
	if ((header.headerSCP[0] != 'S') || (header.headerSCP[1] != 'C') || (header.headerSCP[2] != 'P'))
		return ADFResult::adfrBadSCPFile;
	if (header.numHeads != 0)
		return ADFResult::adfrBadSCPFile;
	if (header.numHeads != 0)
		return ADFResult::adfrBadSCPFile;
	if (header.flags & (1<<BITFLAG_EXTENDED))
		return ADFResult::adfrBadSCPFile;

	const DWORD fluxMultiplier = (header.timeBase + 1) * 25;

	// Read the offsets table
	std::vector<uint32_t> trackOffsets;
	trackOffsets.resize(168);
	try {
		hADFFile.read((char*)trackOffsets.data(), sizeof(uint32_t) * trackOffsets.size());
	}
	catch (...) {
		hADFFile.close();
		return ADFResult::adfrFileIOError;
	}

	// Now write the tracks.
	for (unsigned int track = header.startTrack; track <= header.endTrack; track++) {
		// Lets get into the cotrrect position
		if (m_device.selectTrack(track / 2) != DiagnosticResponse::drOK) return ADFResult::adfrDriveError;
		if (m_device.selectSurface((track & 1) ? DiskSurface::dsUpper : DiskSurface::dsLower) != DiagnosticResponse::drOK) return ADFResult::adfrDriveError;
		if (callback)
			if (callback(track / 2, (track & 1) ? DiskSurface::dsUpper : DiskSurface::dsLower, false, CallbackOperation::coWriting)== WriteResponse::wrAbort) return ADFResult::adfrAborted;


		// Find the track data
		hADFFile.seekp(trackOffsets[track], std::fstream::beg);

		SCPTrackInMemory trk;

		// Read the header
		try {
			hADFFile.read((char*)&trk.header, sizeof(trk.header));
		}
		catch (...) {
			hADFFile.close();
			return ADFResult::adfrFileIOError;
		}
		if (trk.header.trackNumber != track) return ADFResult::adfrBadSCPFile;
		if ((trk.header.headerTRK[0] != 'T') || (trk.header.headerTRK[1] != 'R') || (trk.header.headerTRK[2] != 'K')) return ADFResult::adfrBadSCPFile;

		// Now read in the track info
		for (int r = 0; r < header.numRevolutions; r++) {
			SCPTrackRevolution rev;
			try {
				hADFFile.read((char*)&rev, sizeof(SCPTrackRevolution));
				trk.revolution.push_back(rev);
			}
			catch (...) {
				hADFFile.close();
				return ADFResult::adfrFileIOError;
			}
		}

		std::vector<DWORD> actualFluxTimes;
		{

			SCPTrackData allData;
			// And now read in their data
			for (int r = 0; r < header.numRevolutions; r++) {
				// Goto the data
				hADFFile.seekp(trk.revolution[r].dataOffset + trackOffsets[track], std::fstream::beg);

				allData.resize(trk.revolution[r].trackLength);
				trk.revolution[r].dataOffset = actualFluxTimes.size();  // for use later on
				try {
					hADFFile.read((char*)allData.data(), trk.revolution[r].trackLength * 2);
					// Convert allData into proper flux times in nanoseconds
					// Move the first sample to the end as its sometimes incorrect
					DWORD lastTime = 0;
					for (const uint16_t t : allData) {
						const uint16_t t2 = htons(t);  // paws naidne
						if (t2 == 0) lastTime += 65536; else {
							DWORD totalFlux = (lastTime + t2) * fluxMultiplier;							
							actualFluxTimes.push_back(totalFlux);
							lastTime = 0;
						}
					}

					trk.revolution[r].trackLength = actualFluxTimes.size() - trk.revolution[r].dataOffset;
				}
				catch (...) {
					hADFFile.close();
					return ADFResult::adfrFileIOError;
				}
			}
		}

		// Now compute a master flux times structure from the data for all three.  They *should* all be the same length
		int revolutionToWrite = (header.numRevolutions>1) ? 1 : 0;
		std::vector<DWORD> masterTimes;
		for (DWORD i = 0; i < trk.revolution[revolutionToWrite].trackLength; i++) {
			masterTimes.push_back(actualFluxTimes[i+ trk.revolution[revolutionToWrite].dataOffset]);
		}

		if (extraErases) {
			m_device.eraseFluxOnTrack();
			m_device.eraseFluxOnTrack();
		}
		m_device.eraseCurrentTrack();
		DiagnosticResponse r = m_device.writeFlux(masterTimes,0, driveRPM, false, true);
		if ((r == DiagnosticResponse::drFramingError) || (r == DiagnosticResponse::drSerialOverrun)) {
			// Retry
			m_device.eraseCurrentTrack();
			r = m_device.writeFlux(masterTimes,0, driveRPM, true, true);
		}
		if (r == DiagnosticResponse::drMediaTypeMismatch) return ADFResult::adfrMediaSizeMismatch;
		if (r != DiagnosticResponse::drOK) return ADFResult::adfrDriveError;	
	}

	return ADFResult::adfrComplete;
}


// Reads the disk and write the data to the ADF file supplied.  The callback is for progress, and you can returns FALSE to abort the process
ADFResult ADFWriter::DiskToADF(const std::wstring& outputFile, const bool inHDMode, const unsigned int numTracks, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const int retryCounter, const int sectorsFound, const int badSectorsFound, const int maxSectors, const CallbackOperation operation)> callback) {
	if (!m_device.isOpen()) return ADFResult::adfrDriveError;

	if (callback)
		if (callback(0, DiskSurface::dsLower, 0, 0, 0, 0, CallbackOperation::coStarting) == WriteResponse::wrAbort) return ADFResult::adfrAborted;

	// Higher than this is not supported
	if (numTracks>84) return ADFResult::adfrDriveError; 

	// Attempt ot open the file
#ifdef _WIN32	
	std::ofstream hADFFile = std::ofstream(outputFile, std::ofstream::out | std::ofstream::binary | std::ofstream::trunc);
#else
	std::string outputFileA;
	quickw2a(outputFile,outputFileA);
	std::ofstream hADFFile = std::ofstream(outputFileA, std::ofstream::out | std::ofstream::binary | std::ofstream::trunc);
#endif	
	if (m_device.setDiskCapacity(inHDMode) != DiagnosticResponse::drOK) {
		return ADFResult::adfrAborted;
	}

	if (!hADFFile.is_open()) return ADFResult::adfrFileError;

	// To hold a raw track
	RawTrackDataHD data;
	DecodedTrack track;
	bool includesBadSectors = false;

	const unsigned int readSize = inHDMode ? sizeof(ArduinoFloppyReader::RawTrackDataHD) : sizeof(ArduinoFloppyReader::RawTrackDataDD);
	const unsigned int maxSectorsPerTrack = inHDMode ? NUM_SECTORS_PER_TRACK_HD : NUM_SECTORS_PER_TRACK_DD;

	// Do all tracks
	for (unsigned int currentTrack = 0; currentTrack < numTracks; currentTrack++) {

		// Select the track we're working on
		if (m_device.selectTrack(currentTrack) != DiagnosticResponse::drOK) {
			hADFFile.close();
			return ADFResult::adfrCompletedWithErrors;
		}
		
		// Now select the side
		for (unsigned int surfaceIndex = 0; surfaceIndex < 2; surfaceIndex++) {
			// Surface 
			const DiskSurface surface = (surfaceIndex == 1) ? DiskSurface::dsUpper : DiskSurface::dsLower;

			// Change the surface we're looking at
			if (m_device.selectSurface(surface) != DiagnosticResponse::drOK) {
				hADFFile.close();
				return ADFResult::adfrCompletedWithErrors;
			}

			// Reset the sectors list
			for (unsigned int sector=0; sector< maxSectorsPerTrack; sector++)
				track.invalidSectors[sector].clear();

			track.validSectors.clear();

			// Extract phase code
			int failureTotal = 0;
			bool ignoreChecksums = false;

			// Repeat until we have all 11 sectors
			while (track.validSectors.size() < maxSectorsPerTrack) {

				if (callback) {
					int total = 0;
					for (unsigned int sector = 0; sector < maxSectorsPerTrack; sector++)
						if (track.invalidSectors[sector].size()) total++;

					if ((failureTotal%6)==5) {
						// simulate what the Amiga kinda sounded like it was doing by re-seeking to the track.  This sometimes fixes it, weird eh, and sounds cool
						if (currentTrack < 40) {
							m_device.selectTrack(currentTrack+30, ArduinoFloppyReader::TrackSearchSpeed::tssSlow);
						}
						else {
							m_device.selectTrack(currentTrack-30, ArduinoFloppyReader::TrackSearchSpeed::tssSlow);
						}
						m_device.selectTrack(currentTrack);
					}

					switch (callback(currentTrack, surface, failureTotal, track.validSectors.size(), total, maxSectorsPerTrack, failureTotal > 0 ? CallbackOperation::coRetryReading : CallbackOperation::coReading)) {
						case WriteResponse::wrContinue: break;  // do nothing
						case WriteResponse::wrRetry:    failureTotal = 0; break;
						case WriteResponse::wrAbort:    hADFFile.close();
														return ADFResult::adfrAborted;

						case WriteResponse::wrSkipBadChecksums: 
							if (ignoreChecksums) {
								// Already been here, so we'll create blank sectors just to get this going
								for (unsigned char sectornumber = 0; sectornumber <= maxSectorsPerTrack; sectornumber++) {
									auto index = std::find_if(track.validSectors.begin(), track.validSectors.end(), [sectornumber](const DecodedSector& sector) -> bool {
										return (sector.sectorNumber == sectornumber);
									});
									// Not found. Lets add it
									if (index == track.validSectors.end()) {
										DecodedSector sector;
										memset(&sector, 0, sizeof(sector));
										sector.sectorNumber = sectornumber;
										track.validSectors.push_back(sector);
									}
								}
							}
							ignoreChecksums = true;
							failureTotal = 0;
							break;
					}
				}

				if (m_device.readCurrentTrack(data, readSize, false) == DiagnosticResponse::drOK) {
					findSectors(data, inHDMode, currentTrack, surface, AMIGA_WORD_SYNC, track, ignoreChecksums);
					failureTotal++;
				}
				else {
					hADFFile.close();
					return ADFResult::adfrDriveError;
				}

				// If the user wants to skip invalid sectors and save them
				if (ignoreChecksums) {
					for (unsigned int sector = 0; sector < maxSectorsPerTrack; sector++)
						if (track.invalidSectors[sector].size()) {
							includesBadSectors = true;
							break;
						}
					mergeInvalidSectors(track, inHDMode);
				}
			}

			// Sort the sectors in order
			std::sort(track.validSectors.begin(), track.validSectors.end(), [](const DecodedSector & a, const DecodedSector & b) -> bool {
				return a.sectorNumber < b.sectorNumber;
			});

			// Now write all of them to disk
			for (unsigned int sector = 0; sector < maxSectorsPerTrack; sector++) {
				try {
					hADFFile.write((const char*)track.validSectors[sector].data, 512);
				}
				catch (...) {
					hADFFile.close();
					return ADFResult::adfrFileIOError;
				}
			}
		}
	}

	hADFFile.close();

	return includesBadSectors ? ADFResult::adfrCompletedWithErrors : ADFResult::adfrComplete;
}

struct WeakData {
	UDWORD start, size;
};

enum class BitType : uint8_t {
	btOff = 0,
	btOn = 1,
	btWeak = 2
};

// Converts a density value for a single bit-cell (1000=2us) in ns
#define DensityToNS(densityValue) ((2000UL * densityValue) / 1000UL)
#define IDEAL_RPM_NS 

struct IPFData {
	uint16_t density = 1000;
	BitType bit;
};

// So, DrawBridge was around before things like Greaseweazle... seems only fair I should get some help from its source code.
// Whilst this is not a copy of the functions used by it, I have taken some inspiration from it. credit where credit is due.
// 
// Writes an IPF file back to a floppy disk.  Return FALSE in the callback to abort this operation.  
ADFResult ADFWriter::IPFToDisk(const std::wstring& inputFile, bool extraErases, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const bool isVerifyError, const CallbackOperation operation) > callback) {
	if (!m_device.isOpen()) return ADFResult::adfrDriveError;
	ArduinoFloppyReader::FirmwareVersion version = m_device.getFirwareVersion();
	if ((version.major == 1) && ((version.minor < 9) || ((version.minor == 9) && (version.buildNumber < 22)))) return ADFResult::adfrFirmwareTooOld;

	m_device.checkForDisk(true);
	if (!m_device.isDiskInDrive()) return ADFResult::adfrDriveError;

	// Get drive RPM
	float driveRPM = 300.0f;
#ifndef _DEBUG
	float rpm2;
	if (m_device.measureDriveRPM(rpm2) != DiagnosticResponse::drOK) return ADFResult::adfrDriveError;
	if (m_device.measureDriveRPM(driveRPM) != DiagnosticResponse::drOK) return ADFResult::adfrDriveError;
	driveRPM = ceil((driveRPM + rpm2) / 2.0f);
#else
	driveRPM = 300;
#endif

	// Initialize caps
	if (CapsInit()!= imgeOk) return ADFResult::adfrIPFLibraryNotAvailable;

	SDWORD image = CapsAddImage();
	if (image<0) return ADFResult::adfrIPFLibraryNotAvailable;

	std::string inputFileA;
	quickw2a(inputFile, inputFileA);

	// Load the image
	if (CapsLockImage(image, (PCHAR)inputFileA.c_str()) != imgeOk) {
		CapsRemImage(image);
		return  ADFResult::adfrIPFLibraryNotAvailable;
	}

	// Load the image
	if (CapsLoadImage(image, DI_LOCK_DENVAR | DI_LOCK_UPDATEFD | DI_LOCK_TYPE | DI_LOCK_OVLBIT | DI_LOCK_TRKBIT) != imgeOk) {
		CapsRemImage(image);
		return  ADFResult::adfrIPFLibraryNotAvailable;
	}

	CapsImageInfo fileInfo;
	// get image information
	if (CapsGetImageInfo(&fileInfo, image) != imgeOk) {
		CapsUnlockImage(image);
		CapsRemImage(image);
		return  ADFResult::adfrFileError;
	}

	std::vector<IPFData> data;
	std::vector<DWORD> flux;
	
	size_t topRange = fileInfo.maxcylinder;
	if (topRange > 83) topRange = 83;

	for (size_t cyl = fileInfo.mincylinder; cyl <= topRange; cyl++) {

		// Lets get into the cotrrect position
		if (m_device.selectTrack((unsigned char)cyl) != DiagnosticResponse::drOK) {
			CapsUnlockImage(image);
			CapsRemImage(image);
			return ADFResult::adfrDriveError;
		}
		
		for (DWORD head = fileInfo.minhead; head <= fileInfo.maxhead; head++) {
			if (m_device.selectSurface(head ? DiskSurface::dsUpper : DiskSurface::dsLower) != DiagnosticResponse::drOK) {
				CapsUnlockImage(image);
				CapsRemImage(image);
				return ADFResult::adfrDriveError;
			}

			if (callback)
				if (callback(cyl, head ? DiskSurface::dsUpper : DiskSurface::dsLower, false, CallbackOperation::coWriting) == WriteResponse::wrAbort) {
					CapsUnlockImage(image);
					CapsRemImage(image);
					return ADFResult::adfrAborted;
				}

			// Read information about this from the library
			CapsTrackInfoT2 trackInfo = { 0 };
			trackInfo.type = 2;
			if (CapsLockTrack((PCAPSTRACKINFO)&trackInfo, image, cyl, head, DI_LOCK_DENVAR | DI_LOCK_UPDATEFD | DI_LOCK_TYPE | DI_LOCK_OVLBIT | DI_LOCK_TRKBIT) != imgeOk) {
				CapsUnlockImage(image);
				CapsRemImage(image);
				return  ADFResult::adfrFileError;
			}

			// Check for unformatted/empty track
			if ((trackInfo.trackbuf == nullptr) || (trackInfo.tracklen<1)) {
				// Unformatted track
				if (m_device.eraseFluxOnTrack() != DiagnosticResponse::drOK) {
					CapsUnlockTrack(image, cyl, head);
					CapsUnlockImage(image);
					CapsRemImage(image);
					return ADFResult::adfrDriveError;
				}

				// Done here!
				continue;
			}

			// Convert the trackbuf to a structure we can handle, whilst offsetting everything for the "splice" (where the track starts and stops)	
			data.clear();
			for (size_t i = 0; i < trackInfo.tracklen; i++) {
				int outPos = (i + trackInfo.overlap) % trackInfo.tracklen;
				if (outPos < 0) outPos += trackInfo.tracklen;
				size_t bytePos = outPos / 8;
				uint16_t density = 1000;

				if ((trackInfo.timebuf) && (trackInfo.timelen) && (bytePos < trackInfo.timelen)) {
					density = (uint16_t)trackInfo.timebuf[bytePos];
				}
				data.push_back({ density, (trackInfo.trackbuf[bytePos] & (1 << (7-(outPos & 7)))) ? BitType::btOn : BitType::btOff });
			}
			 
			// Handle 'weak bits'
			std::vector<WeakData> weakList;
			for (size_t weak = 0; weak < trackInfo.weakcnt; weak++) {
				CapsDataInfo wInfo;
				if (CapsGetInfo(&wInfo, image, cyl, head, cgiitWeak, weak) != imgeOk) {
					CapsUnlockTrack(image, cyl, head);
					CapsUnlockImage(image);
					CapsRemImage(image);
					return ADFResult::adfrIPFLibraryNotAvailable;
				}

				// Add weak-list compensated for overlap position
				int previousBit = ((wInfo.start -1) - trackInfo.overlap) % trackInfo.tracklen; if (previousBit < 0) previousBit += trackInfo.tracklen;
				size_t startPos = 0;

				// Previous bit is 'on', force the first 'weak' bit to be off regardless
				if (data[previousBit].bit == BitType::btOn) {
					startPos++;
					previousBit++;
					previousBit %= trackInfo.tracklen;
					data[previousBit].bit = BitType::btOff;
				}
				for (size_t bitPos = startPos; bitPos < wInfo.size; bitPos++) {
					int outPos = (wInfo.start - trackInfo.overlap) % trackInfo.tracklen;
					if (outPos < 0) outPos += trackInfo.tracklen;
					data[outPos].bit = BitType::btWeak;
				}
				int endingWeakBit = (wInfo.start + (wInfo.size-1) - trackInfo.overlap) % trackInfo.tracklen; if (endingWeakBit<0) endingWeakBit+= trackInfo.tracklen;
				int nextBit = (wInfo.start + wInfo.size - trackInfo.overlap) % trackInfo.tracklen; if (nextBit < 0) nextBit += trackInfo.tracklen;
				
				// Ensure we dont leave weak bits next to the actual data
				if (data[nextBit].bit == BitType::btOn) data[endingWeakBit].bit = BitType::btOff;
			}

			// Now convert this data into flux timings, based on the data
			flux.clear();
			DWORD fluxSoFar = 0;
			DWORD fluxSoFarOut = 0;
			DWORD numbits = 0;
			int64_t fluxTimeAtOverlap = 0;
			int overlapPos = trackInfo.overlap % trackInfo.tracklen;
			if (overlapPos < 0) overlapPos += trackInfo.tracklen;
			uint64_t totalTime = 0;

			for (size_t i = 0; i < data.size(); i++) {
				size_t time = (DWORD)DensityToNS(data[i].density);
				fluxSoFar += time;
				fluxSoFarOut = 0;
				numbits++;

				// Flux of some kind
				switch (data[i].bit) {
				case BitType::btOff: 
					break; // we dont care

				case BitType::btOn: 
					flux.push_back(fluxSoFar);
					fluxSoFarOut = fluxSoFar;
					fluxSoFar = 0; 
					numbits = 0;  
					break;

				case BitType::btWeak: 
					if (numbits < 400) {						
						// Just push an extra long 'no flux' region
						if (fluxSoFar >= 3750) {
							flux.push_back(fluxSoFar);
							fluxSoFarOut = fluxSoFar;
						}
					}
					else {
						// Create fuzzy bits where bits are on the boundary of where they should be
						fluxSoFarOut = 0;
						while (fluxSoFar > 183000) {
							// Get the PLL in sync
							flux.push_back(8000);    // 0001
							flux.push_back(6000);    // 001
							flux.push_back(4000);    // 01
							// Then screw with it
							for (int counter = 1; counter <= 7; counter++) {
								flux.push_back(6000 - (counter*125)); 
								flux.push_back(4000 + (counter*125));    // 01
							}
							flux.push_back(5000);    // ?!
							for (int counter = 7; counter >=1; counter--) {
								flux.push_back(4000 + (counter * 125));    // 01
								flux.push_back(6000 - (counter * 125));
							}
							// And go back to normal
							for (int counter=1; counter<=5; counter++)
								flux.push_back(4000); 

							fluxSoFar -= 183000;
							fluxSoFarOut += 183000;
						}

						// See how much flux time is left
						while (fluxSoFar > 32000) {
							flux.push_back(32000);
							fluxSoFar -= 32000;
							fluxSoFarOut += 32000;
						}
						if (fluxSoFar >= 3750) {
							flux.push_back(fluxSoFar);
							fluxSoFarOut += fluxSoFar;
						}
					}
					fluxSoFar = 0;
					numbits = 0;
					break;
				} 

				// Count flux up until the index
				if (i <= (DWORD)overlapPos+1)
					fluxTimeAtOverlap += (int64_t)time;

				totalTime += fluxSoFarOut;
			}
			// This belongs at the start
			if (fluxSoFar) {
				flux[0] += fluxSoFar;
				fluxTimeAtOverlap -= fluxSoFar;
			}

			// Data is gap aligned. So add extra to the gap to ensure it fills the disk.

			// Is splice at the index point?
			bool terminateAtIndex = false;
			if ((overlapPos <= 4) || (overlapPos >= (int)(data.size() - 4))) {
				// Ensure theres more data than needed by repeating the last few flux transitions a little slower
				size_t startPoint = flux.size() - 15;
				unsigned int count = 0;
				while (totalTime < 220000000) {
					count = (count + 1) & 15;
					DWORD t = (DWORD)(flux[startPoint + count] * 1.1f);
					flux.push_back(t);
					totalTime += t;
				}

				// Yes.  This is INDEX to INDEX mode.
				terminateAtIndex = true;
				fluxTimeAtOverlap = 0;
			}
			else {
				// No.  This is INDEX+Delay
				terminateAtIndex = false;
			}

			// Now write the track
			if (extraErases) {
				m_device.eraseFluxOnTrack();
				m_device.eraseFluxOnTrack();
			}
			// Reset to 01010101 etc
			m_device.eraseCurrentTrack();
			DiagnosticResponse r = m_device.writeFlux(flux, (DWORD)fluxTimeAtOverlap, driveRPM, false, terminateAtIndex);
			if ((r == DiagnosticResponse::drFramingError) || (r == DiagnosticResponse::drSerialOverrun)) {
				// Retry
				m_device.eraseFluxOnTrack();
				r = m_device.writeFlux(flux, (DWORD)fluxTimeAtOverlap, driveRPM, false, terminateAtIndex);
			}			
			if (r != DiagnosticResponse::drOK) return ADFResult::adfrDriveError;
						
			CapsUnlockTrack(image, cyl, head);
		}
	}

	CapsUnlockImage(image);
	CapsRemImage(image);

	return ADFResult::adfrComplete;
	

}



// Attempt to work out what the density of the currently inserted disk is
ADFResult ADFWriter::GuessDiskDensity(bool& isHD) {
	if (!m_device.isOpen()) return ADFResult::adfrDriveError;

	if (m_device.selectSurface(ArduinoFloppyReader::DiskSurface::dsLower) != DiagnosticResponse::drOK) return ADFResult::adfrAborted;
	if (m_device.selectTrack(0) != DiagnosticResponse::drOK) return ADFResult::adfrAborted;

	if (m_device.checkDiskCapacity(isHD) == DiagnosticResponse::drOK) return ADFResult::adfrComplete; else return ADFResult::adfrAborted;
}

