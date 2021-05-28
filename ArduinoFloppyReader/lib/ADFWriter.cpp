/* ArduinoFloppyReader (and writer)
*
* Copyright (C) 2017-2021 Robert Smith (@RobSmithDev)
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
// V2.5

#include <vector>
#include <algorithm>
#include "ADFWriter.h"
#include "ArduinoInterface.h"
#include "RotationExtractor.h"
#include <assert.h>
#include <sstream>
#include <string>
#include <codecvt>
#include <thread>
#include <fstream>
#include <iostream>
#ifndef _WIN32
#include <string.h>
#include <cstring>
#endif

using namespace ArduinoFloppyReader;

#ifndef OUTPUT_TIME_IN_NS
WARNING: OUTPUT_TIME_IN_NS MUST BE DEFINED IN REVOLUTIONEXTRACTOR.H FOR THIS CLASS TO WORK CORRECTLY
#endif


#define MFM_MASK    0x55555555L		
#define AMIGA_WORD_SYNC  0x4489							 // Disk SYNC code for the Amiga start of sector
#define SECTOR_BYTES	512								 // Number of bytes in a decoded sector
#define NUM_SECTORS_PER_TRACK 11						 // Number of sectors per track
#define RAW_SECTOR_SIZE (8+56+SECTOR_BYTES+SECTOR_BYTES)      // Size of a sector, *Including* the sector sync word longs
#define ADF_TRACK_SIZE (SECTOR_BYTES*NUM_SECTORS_PER_TRACK)   // Bytes required for a single track


const char* TEST_BYTE_SEQUENCE = "amiga.robsmithdev.co.uk";

// Buffer to hold raw data for just a single sector
typedef unsigned char RawEncodedSector[RAW_SECTOR_SIZE];
typedef unsigned char RawDecodedSector[SECTOR_BYTES];
typedef RawDecodedSector RawDecodedTrack[NUM_SECTORS_PER_TRACK];
typedef unsigned char RawMFMData[SECTOR_BYTES + SECTOR_BYTES];

// When workbench formats a disk, it write 13630 bytes of mfm data to the disk.  So we're going to write this amount, and then we dont need an erase first
typedef struct alignas(1) {
	unsigned char filler1[1654];  // Padding at the start of the track.  This will be set to 0xaa
	// Raw sector data
	RawEncodedSector sectors[NUM_SECTORS_PER_TRACK];   // 11968 bytes
	// Blank "Filler" gap content. (this may get overwritten by the sectors a little)
	unsigned char filler2[8];
} FullDiskTrack;

// Structure to hold data while we decode it
typedef struct alignas(8)  {
	unsigned char trackFormat;        // This will be 0xFF for Amiga
	unsigned char trackNumber;        // Current track number (this is actually (tracknumber*2) + side
	unsigned char sectorNumber;       // The sector we just read (0 to 11)
	unsigned char sectorsRemaining;   // How many more sectors remain until the gap (0 to 10)

	unsigned long sectorLabel[4];     // OS Recovery Data, we ignore this

	unsigned long headerChecksum;	  // Read from the header, header checksum
	unsigned long dataChecksum;		  // Read from the header, data checksum

	unsigned long headerChecksumCalculated;   // The header checksum we calculate
	unsigned long dataChecksumCalculated;     // The data checksum we calculate

	RawDecodedSector data;          // decoded sector data

	RawMFMData rawSector;   // raw track data, for analysis of invalid sectors
} DecodedSector;

// To hold a list of valid and checksum failed sectors
struct DecodedTrack {
	// A list of valid sectors where the checksums are OK
	std::vector<DecodedSector> validSectors;
	// A list of sectors found with invalid checksums.  These are used if ignore errors is triggered
	// We keep copies of each one so we can perform a statistical analysis to see if we can get a working one based on which bits are mostly set the same
	std::vector<DecodedSector> invalidSectors[NUM_SECTORS_PER_TRACK];
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
	return chksum & MFM_MASK;
}

// MFM encoding algorithm part 1 - this just writes the actual data bits in the right places
// *input;	RAW data buffer (size == data_size) 
// *output;	MFM encoded buffer (size == data_size*2) 
// Returns the checksum calculated over the data
unsigned long encodeMFMdataPart1(const unsigned long* input, unsigned long* output, const unsigned int data_size) {
	unsigned long chksum = 0L;
	unsigned int count;

	unsigned long* outputOdd = output;
	unsigned long* outputEven = (unsigned long*)(((unsigned char*)output) + data_size);

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
void alignSectorToByte(const RawTrackData& inTrack, int byteStart, int bitStart, RawEncodedSector& outSector) {
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
			if (inTrack[byteStart%RAW_TRACKDATA_LENGTH] & (1 << bitCounter)) byteOut |= 1;

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
bool decodeSector(const RawEncodedSector& rawSector, const unsigned int trackNumber, const DiskSurface surface, DecodedTrack& decodedTrack, bool ignoreHeaderChecksum, int& lastSectorNumber) {
	DecodedSector sector;

	lastSectorNumber = -1;
	memcpy(sector.rawSector, rawSector, sizeof(RawMFMData));

	// Easier to operate on
	unsigned char* sectorData = (unsigned char*)rawSector;
 
	// Read the first 4 bytes (8).  This  is the track header data	
	sector.headerChecksumCalculated = decodeMFMdata((unsigned long*)(sectorData + 8), (unsigned long*)&sector, 4);
	// Decode the label data and update the checksum
	sector.headerChecksumCalculated ^= decodeMFMdata((unsigned long*)(sectorData + 16), (unsigned long*)&sector.sectorLabel[0], 16);
	// Get the checksum for the header
	decodeMFMdata((unsigned long*)(sectorData + 48), (unsigned long*)&sector.headerChecksum, 4);  // (computed on mfm longs, longs between offsets 8 and 44 == 2 * (1 + 4) longs)
	// If the header checksum fails we just cant trust anything we received, so we just drop it
	if ((sector.headerChecksum != sector.headerChecksumCalculated) && (!ignoreHeaderChecksum)) {
		return false;
	}

	// Check if the header contains valid fields
	if (sector.trackFormat != 0xFF) 
		return false;  // not valid
	if (sector.sectorNumber > 10) 
		return false;
	if (sector.trackNumber > 162) 
		return false;   // 81 tracks * 2 for both sides
	if (sector.sectorsRemaining > 11)
		return false;  // this isnt possible either
	if (sector.sectorsRemaining < 1)
		return false;  // or this

	// And is it from the track we expected?
	const unsigned char targetTrackNumber = (trackNumber << 1) | ((surface == DiskSurface::dsUpper) ? 1 : 0);

	if (sector.trackNumber != targetTrackNumber) return false; // this'd be weird

	// Get the checksum for the data
	decodeMFMdata((unsigned long*)(sectorData + 56), (unsigned long*)&sector.dataChecksum, 4);
	

	// Lets see if we already have this one
	const int searchSector = sector.sectorNumber;
	auto index = std::find_if(decodedTrack.validSectors.begin(), decodedTrack.validSectors.end(), [searchSector](const DecodedSector& sector) -> bool {
		return (sector.sectorNumber == searchSector);
	});

	// We already have it as a GOOD VALID sector, so skip, we don't need it.
	if (index != decodedTrack.validSectors.end()) return true;

	// Decode the data and receive it's checksum
	sector.dataChecksumCalculated = decodeMFMdata((unsigned long*)(sectorData + 64), (unsigned long*)&sector.data[0], SECTOR_BYTES); // (from 64 to 1088 == 2*512 bytes)

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
void encodeSector(const unsigned int trackNumber, const DiskSurface surface, const unsigned int sectorNumber, const RawDecodedSector& input, RawEncodedSector& encodedSector, unsigned char& lastByte) {
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
	header.sectorsRemaining = NUM_SECTORS_PER_TRACK - sectorNumber;  //1..11
	
	
	header.headerChecksumCalculated = encodeMFMdataPart1((const unsigned long*)&header, (unsigned long*)&encodedSector[8], 4);
	// Then theres the 16 bytes of the volume label that isnt used anyway
	header.headerChecksumCalculated ^= encodeMFMdataPart1((const unsigned long*)&header.sectorLabel, (unsigned long*)&encodedSector[16], 16);
	// Thats 40 bytes written as everything doubles (8+4+4+16+16). - Encode the header checksum
	encodeMFMdataPart1((const unsigned long*)&header.headerChecksumCalculated, (unsigned long*)&encodedSector[48], 4);
	// And move on to the data section.  Next should be the checksum, but we cant encode that until we actually know its value!
	header.dataChecksumCalculated = encodeMFMdataPart1((const unsigned long*)&input, (unsigned long*)&encodedSector[64], SECTOR_BYTES);
	// And add the checksum
	encodeMFMdataPart1( (const unsigned long*)&header.dataChecksumCalculated, (unsigned long*)&encodedSector[56], 4);

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
void findSectors(const RawTrackData& track, unsigned int trackNumber, DiskSurface side, unsigned short trackSync, DecodedTrack& decodedTrack, bool ignoreHeaderChecksum) {
	// Work out what we need to search for which is 2AAAAAAAsyncsync
	//const unsigned long long search = (trackSync | (((DWORD)trackSync) << 16)) | (((long long)0x2AAAAAAA) << 32);
	// For speed and to ignore some data errors, we now just search for syncsync and ignore the 2AAAAAAA part

	// Work out what we need to search for which is syncsync
	const unsigned long search = (trackSync | (((unsigned long)trackSync) << 16));

	// Prepare our test buffer
	unsigned long decoded = 0;

	// Keep runnign until we run out of data
	unsigned int byteIndex = 0;

	int nextTrackBitCount = 0;

	// run the entire track length
	while (byteIndex < RAW_TRACKDATA_LENGTH) {

		// Check each bit, the "decoded" variable slowly slides left providing a 64-bit wide "window" into the bitstream
		for (int bitIndex = 7; bitIndex >= 0; bitIndex--) {
			decoded <<= 1;   // shift off one bit to make room for the new bit
			if (track[byteIndex] & (1 << bitIndex)) decoded |= 1;

			// Have we matched the sync words correctly
			++nextTrackBitCount;
			int lastSectorNumber = -1;
			if (decoded == search) {
				RawEncodedSector alignedSector;
				
				// We extract ALL of the track data from this BIT to byte align it properly, then pass it onto the code to read the sector (from the start of the sync code)
				alignSectorToByte(track, byteIndex, bitIndex, alignedSector);

				// Now see if there's a valid sector there.  We now only skip the sector if its valid, incase rogue data gets in there
				if (decodeSector(alignedSector, trackNumber, side, decodedTrack, ignoreHeaderChecksum, lastSectorNumber)) {
					// We know the size of this buffer, so we can skip by exactly this amount
					byteIndex += RAW_SECTOR_SIZE - 8; // skip this many bytes as we know this is part of the track! minus 8 for the SYNC
					if (byteIndex >= RAW_TRACKDATA_LENGTH) break;
					// We know that 8 bytes from here should be another track. - we allow 1 bit either way for slippage, but this allows an extra check incase the SYNC pattern is damaged
					nextTrackBitCount = 0;
				}
				else {

					// Decode failed.  Lets try a "homemade" one
					DecodedSector newTrack;
					if ((lastSectorNumber >= 0) && (lastSectorNumber < NUM_SECTORS_PER_TRACK)) {
						newTrack.sectorNumber = lastSectorNumber;
						if (attemptFixSector(decodedTrack, newTrack)) {
							memcpy(newTrack.rawSector, alignedSector, sizeof(newTrack.rawSector));
							// See if our makeshift data will decode or not
							if (decodeSector(alignedSector, trackNumber, side, decodedTrack, ignoreHeaderChecksum, lastSectorNumber)) {
								// We know the size of this buffer, so we can skip by exactly this amount
								byteIndex += RAW_SECTOR_SIZE - 8; // skip this many bytes as we know this is part of the track! minus 8 for the SYNC
								if (byteIndex >= RAW_TRACKDATA_LENGTH) break;
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
void mergeInvalidSectors(DecodedTrack& track) {
	for (unsigned char sector = 0; sector < NUM_SECTORS_PER_TRACK; sector++) {
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

// Open the device we want to use.  Returns TRUE if it worked
bool ADFWriter::openDevice(const std::wstring& portName) {
	if (m_device.openPort(portName) != DiagnosticResponse::drOK) return false;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	return m_device.enableReading(true, true)== DiagnosticResponse::drOK;
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

	if (!askQuestion(false, "Please insert a disk in the drive.\r\nUse a disk that you don't mind being erased.\nThis disk must contain data/formatted as an AmigaDOS disk")) {
		messageOutput(true, "Diagnostics aborted");
		return false;
	}

	msg << "Attempting to open and use ";
	char convert[20];
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
	sprintf_s(buffer, "Board is running firmware version %i.%i%s\n", version.major, version.minor, version.fullControlMod ? " (modded for DiskChange support)" : "");
#else	
	sprintf(buffer, "Board is running firmware version %i.%i%s\n", version.major, version.minor, version.fullControlMod ? " (modded for DiskChange support)" : "");
#endif
	messageOutput(false, buffer);

	if ((version.major == 1) && (version.minor < 8)) {
		messageOutput(false, "This firmware is out of date.  Please update it!");
	}

	if ((version.major > 1) || ((version.major == 1) && (version.minor >= 8))) {
		messageOutput(false, "Testing USB->Serial transfer speed (read)");
		if (m_device.testTransferSpeed() != DiagnosticResponse::drOK) {
			messageOutput(false, "The USB->Serial adapter in use is not suitable.");
			messageOutput(false, "Arduino UNO: The on-board adapter is not able to sustain large data transfers");
			messageOutput(false, "Arduino Pro Mini: The USB-Serial board is not able to sustain large data transfers");
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
		return false;
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
		return false;
	}

	if (!askQuestion(true,"Could you hear the head moving quite a distance?")) {
        messageOutput(true, "As we successfully found track 0, please check the following PINS on the Arduino: 6, 7");
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

	// So we can open the drive and move the head.  We're going to turn the drive off
	r = m_device.enableReading(false, false);
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		return false;
	}

	for (;;) {
		if (!askQuestion(false, "Please insert a write protected AmigaDOS disk in the drive")) {
			messageOutput(true, "Diagnostics aborted");
			return false;
		}

		if ((version.major > 1) || ((version.major == 1) && (version.minor >= 8))) {
			if (m_device.checkIfDiskIsWriteProtected(true) == DiagnosticResponse::drWriteProtected) break;
			messageOutput(true, "Disk is not write protected.");
			messageOutput(true, "If it is, then check Arduin Pin A0");
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

	messageOutput(false, "Checking for INDEX pulse from drive");
	r = m_device.testIndexPulse();
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		messageOutput(true, "Please check that a disk is inserted and the following PINS on the Arduino: 2");
		return false;
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
	ArduinoFloppyReader::RawTrackData data;
	int counter;
	bool tracksFound = false;

	for (int a = 0; a < 10; a++) {
		r = m_device.readCurrentTrack(data, true);
		if (r != DiagnosticResponse::drOK) {
			messageOutput(true, m_device.getLastErrorStr());
			return false;
		}

		// Data read.  See if any tracks were detected
		DecodedTrack trk1;
		findSectors(data, 40, ArduinoFloppyReader::DiskSurface::dsUpper, AMIGA_WORD_SYNC, trk1, true);
		
		counter=0; 
		for (int sec=0; sec<NUM_SECTORS_PER_TRACK; sec++)
			if (trk1.invalidSectors[sec].size()) counter++;

		if (counter + trk1.validSectors.size() > 0) {
			messageOutput(false, "Tracks found!");
			tracksFound = true;
			break;
		}
		// Nothing found?
		DecodedTrack trk2;
		findSectors(data, 40, ArduinoFloppyReader::DiskSurface::dsLower, AMIGA_WORD_SYNC, trk2, true);
		
		counter = 0;
		for (int sec = 0; sec<NUM_SECTORS_PER_TRACK; sec++)
			if (trk2.invalidSectors[sec].size()) counter++;

		if (counter + trk2.validSectors.size() > 0) {
			messageOutput(false, "Tracks found but on the wrong side.  Please check the following PINS on the Arduino: 9");
			return false;
		}
		if (tracksFound) break;
	}


	messageOutput(false, "Attempting to read a track from the LOWER side of the disk");
	r = m_device.selectSurface(ArduinoFloppyReader::DiskSurface::dsLower);
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		return false;
	}
	tracksFound = false;

	for (int a = 0; a < 10; a++) {
		r = m_device.readCurrentTrack(data, true);
		if (r != DiagnosticResponse::drOK) {
			messageOutput(true, m_device.getLastErrorStr());
			return false;
		}

		// Data read.  See if any tracks were detected
		DecodedTrack trk1;
		findSectors(data, 40, ArduinoFloppyReader::DiskSurface::dsLower, AMIGA_WORD_SYNC, trk1, true);

		counter = 0;
		for (int sec = 0; sec<NUM_SECTORS_PER_TRACK; sec++)
			if (trk1.invalidSectors[sec].size()) counter++;

		if (counter + trk1.validSectors.size() > 0) {
			messageOutput(false, "Tracks found!");
			tracksFound = true;
			break;
		}
		// Nothing found?
		DecodedTrack trk2;
		findSectors(data, 40, ArduinoFloppyReader::DiskSurface::dsUpper, AMIGA_WORD_SYNC, trk2, true);

		counter = 0;
		for (int sec = 0; sec<NUM_SECTORS_PER_TRACK; sec++)
			if (trk2.invalidSectors[sec].size()) counter++;

		if (counter + trk2.validSectors.size() > 0) {
			messageOutput(false, "Tracks found but on the wrong side.  Please check the following PINS on the Arduino: 9");
			return false;
		}
		if (tracksFound) break;
	}

	messageOutput(false, "Reading was successful!");

	// Turn the drive off again
	r = m_device.enableReading(false,false);
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		return false;
	}

	// Now ask.
	if (!askQuestion(true,"Would you like to test writing to a disk?  (please ensure the disk inserted is not important as data will be erased)")) {
		messageOutput(true, "Diagnostic aborted.");
		return false;
	}

	// Try to enable the write head
	do {
		r = m_device.enableWriting(true, true);

		if (r == DiagnosticResponse::drWriteProtected) {
			if (!askQuestion(false,"Please write-enable the disk and try again.")) {
				messageOutput(true, "Diagnostic aborted.");
				return false;
			}
		} else
		if (r != DiagnosticResponse::drOK) {
			messageOutput(true, m_device.getLastErrorStr());
			return false;
		}

	} while (r == DiagnosticResponse::drWriteProtected);
	// Writing is enabled.
	
	// Goto 41, we'll write some stuff
	r = m_device.selectTrack(41);
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		return false;
	}

	// Upper side
	r = m_device.selectSurface(ArduinoFloppyReader::DiskSurface::dsUpper);
	if (r != DiagnosticResponse::drOK) {
		messageOutput(true, m_device.getLastErrorStr());
		return false;
	}
		
	// We're gonna write a test track out

	RawDecodedSector track[NUM_SECTORS_PER_TRACK];
	FullDiskTrack  disktrack;
	memset(disktrack.filler1, 0xAA, sizeof(disktrack.filler1));  // Pad with "0"s which as an MFM encoded byte is 0xAA
	memset(disktrack.filler2, 0xAA, sizeof(disktrack.filler2));  // Pad with "0"s which as an MFM encoded byte is 0xAA

	// Get length
	int sequenceLength = strlen(TEST_BYTE_SEQUENCE);
	unsigned char lastByte = 0;

	for (unsigned int sector = 0; sector < NUM_SECTORS_PER_TRACK; sector++) {

		// Invent some track data
		for (int bytePos = 0; bytePos < SECTOR_BYTES; bytePos++)
			track[sector][bytePos] = TEST_BYTE_SEQUENCE[(sector + bytePos) % sequenceLength];

		encodeSector(41, ArduinoFloppyReader::DiskSurface::dsUpper, sector, track[sector], disktrack.sectors[sector], lastByte);
	}

	// Attempt to write 10 times, with verify
	messageOutput(false, "Writing and Verifying Track 41 (Upper Side).");
	bool writtenOK = false;
	for (int a = 1; a <= 10; a++) {
		
		r = m_device.writeCurrentTrack((const unsigned char*)(&disktrack), sizeof(disktrack), false);
		if (r != DiagnosticResponse::drOK) {
			messageOutput(true, m_device.getLastErrorStr());
			return false;
		}
		r = m_device.readCurrentTrack(data, false);
		if (r != DiagnosticResponse::drOK) {
			messageOutput(true, m_device.getLastErrorStr());
			return false;
		}

		// Data read.  See if any tracks were detected
		DecodedTrack trk;
		findSectors(data, 41, ArduinoFloppyReader::DiskSurface::dsUpper, AMIGA_WORD_SYNC, trk, true);
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
			for (int sector = 0; sector < NUM_SECTORS_PER_TRACK; sector++) {
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
		}
	}

	// Final results
	if (!writtenOK) {
		messageOutput(true, "Unable to detect written track.  This could be for one of the following reasons:");
		messageOutput(true, "1.  Please check the following PINS on the Arduino: 3, A0, A1");
		messageOutput(true, "2.  Please check the Ardiono IDE config has not been modified from stock.  This was tested using 1.8.4, compiler settings may affect performance");
		messageOutput(true, "3.  Check for poor connections, typically on a breadboard they may be intermittent which may pass the above results but still not work.");
		messageOutput(true, "4.  Check for an electrically noisy environment.  It is possible that electronic noise (eg: from a cell phone) may cause errors reading and writing to the disk");
		messageOutput(true, "5.  Shorten the floppy disk cable to help reduce noise.");
		messageOutput(true, "6.  Ensure your power supply is strong enough to power the floppy drive.  Don't rely on the USB port for the 5V for the floppy drive!");
		messageOutput(true, "7.  You can contact me for help, but some basic electronics diagnostics will help, checkout YouTube for guides.");

		m_device.enableWriting(false);
		return false;
	}
	else {
		messageOutput(false, "Hurray! Writing was successful.  Your Arduino is ready for use! - Send us a photo!");
		m_device.enableWriting(false);
		return true;
	}
	
}

// Get the current firmware version.  Only valid if openPort is successful
const FirmwareVersion ADFWriter::getFirwareVersion() const {
	return m_device.getFirwareVersion(); 
};

// Writes an ADF file back to a floppy disk.  Return FALSE in the callback to abort this operation 
// IF using precomp mode then DO NOT connect the Arduino via a USB hub, and try to plug it into a USB2 port
ADFResult ADFWriter::ADFToDisk(const std::wstring& inputFile, bool verify, bool usePrecompMode, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const bool isVerifyError) > callback) {
	if (!m_device.isOpen()) return ADFResult::adfrDriveError;

	// Upgrade to writing mode
	if (m_device.enableWriting(true, true)!=DiagnosticResponse::drOK) return ADFResult::adfrDriveError;

	// Attempt to open the file
#ifdef _WIN32	
	std::ifstream hADFFile(inputFile, std::ifstream::in | std::ifstream::binary);
#else
	std::string inputFileA;
	quickw2a(inputFile,inputFileA);
	std::ifstream hADFFile(inputFileA, std::ifstream::in | std::ifstream::binary);
#endif
	if (!hADFFile.is_open()) return ADFResult::adfrFileError;

	unsigned int currentTrack = 0;
	unsigned int surfaceIndex = 0;

	// Buffer to hold the track
	RawDecodedTrack track;
	FullDiskTrack disktrack;
	memset(disktrack.filler1, 0xAA, sizeof(disktrack.filler1));  // Pad with "0"s which as an MFM encoded byte is 0xAA
	memset(disktrack.filler2, 0xAA, sizeof(disktrack.filler2));  // Pad with "0"s which as an MFM encoded byte is 0xAA

	// Just make sure nothing weird is going on
	assert(sizeof(track) == ADF_TRACK_SIZE);
	bool errors = false;


	while (hADFFile.good()) {
		hADFFile.read((char*)&track, ADF_TRACK_SIZE);
		std::streamsize bytesRead = hADFFile.gcount();

		// Stop if we didnt read a full track
		if (bytesRead != ADF_TRACK_SIZE) break;

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
			if (callback(currentTrack, surface, false) == WriteResponse::wrAbort) {
				hADFFile.close();
				return ADFResult::adfrAborted;
			}

		unsigned char lastByte = disktrack.filler1[sizeof(disktrack.filler1)-1];
		// Now encode the sector into the output buffer
		for (unsigned int sector = 0; sector < NUM_SECTORS_PER_TRACK; sector++)
			encodeSector(currentTrack, surface, sector, track[sector], disktrack.sectors[sector], lastByte);
		if (lastByte & 1) disktrack.filler2[7] = 0x2F; else disktrack.filler2[7] = 0xFF;

		// Keep looping until it wrote correctly
		DecodedTrack trackRead;
		trackRead.validSectors.clear();
		for (unsigned int a = 0; a < NUM_SECTORS_PER_TRACK; a++) trackRead.invalidSectors[a].clear();

		int failCount = 0;
		while (trackRead.validSectors.size()<NUM_SECTORS_PER_TRACK) {

			DiagnosticResponse resp = m_device.writeCurrentTrackPrecomp((const unsigned char*)(&disktrack), sizeof(disktrack), false, (currentTrack >= 40) && usePrecompMode);
			if (resp == DiagnosticResponse::drOldFirmware) resp = m_device.writeCurrentTrack((const unsigned char*)(&disktrack), sizeof(disktrack), false);		

			switch (resp) {
			case DiagnosticResponse::drWriteProtected:	hADFFile.close();
														return ADFResult::adfrDiskWriteProtected;
			case DiagnosticResponse::drOK: break;
			default: hADFFile.close();
					 return ADFResult::adfrDriveError;
			}

			if (verify) {				
				for (int retries=0; retries<10; retries++) {
					RawTrackData data;
					// Read the track back
					if (m_device.readCurrentTrack(data, false) == DiagnosticResponse::drOK) {
						// Find hopefully all sectors
						findSectors(data, currentTrack, surface, AMIGA_WORD_SYNC, trackRead, false);
					}
					if (trackRead.validSectors.size() == NUM_SECTORS_PER_TRACK) break;
				} 

				// So we found all 11 sectors, but were they the ones we actually wrote!?
				if (trackRead.validSectors.size() == NUM_SECTORS_PER_TRACK) {
					int sectorsGood = 0;
					for (int sector = 0; sector < NUM_SECTORS_PER_TRACK; sector++) {
						auto index = std::find_if(trackRead.validSectors.begin(), trackRead.validSectors.end(), [sector](const DecodedSector& sectorfound) -> bool {
							return (sectorfound.sectorNumber == sector);
						});

						// We found this sector.
						if (index != trackRead.validSectors.end()) {
							DecodedSector& rec = trackRead.validSectors[index - trackRead.validSectors.begin()];
							if (memcmp(rec.data, track[sector], SECTOR_BYTES) == 0) {
								sectorsGood++;  // this one matches on read!
							}
						}
					}
					if (sectorsGood != NUM_SECTORS_PER_TRACK) {
						// Something went wrong, so we clear them all so it gets reported as an error
						trackRead.validSectors.clear();
					}
				}


				// We failed to verify this track.
				if (trackRead.validSectors.size() < NUM_SECTORS_PER_TRACK) {
					failCount++;
					if (failCount >= 5) {
						if (!callback) break;
						bool breakOut = false;

						switch (callback(currentTrack, surface, true)) {
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
		if (currentTrack > 81) break; 
	}

	return errors? ADFResult::adfrCompletedWithErrors: ADFResult::adfrComplete;
}

#pragma pack(1) 

/* Taken from https://www.cbmstuff.com/downloads/scp/scp_image_specs.txt
This information is copyright(C) 2012 - 2020 By Jim Drew.Permission is granted
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
	unsigned char	bitcellEncoding;
	unsigned char	numHeads;
	unsigned char   timeBase;
	unsigned long	checksum;
};

struct SCPTrackHeader {
	char			headerTRK[3];
	unsigned char	trackNumber;
};

struct SCPTrackRevolution {
	unsigned long	indexTime;		// Time in NS/25 for this revolution
	unsigned long	trackLength;	// Number of bit-cells in this revolution
	unsigned long	dataOffset;		// From the start of SCPTrackHeader 
};

// Track data is 16-bit value in NS/25.  If =0 means no flux transition for max time 
#pragma pack()

#define BITFLAG_INDEX		0
#define BITFLAG_NORMALISED  3

typedef std::vector<unsigned short> SCPTrackData;

struct SCPTrackInMemory {
	SCPTrackHeader header;
	std::vector<SCPTrackRevolution> revolution;
	std::vector<SCPTrackData> revolutionData;
}; 

// Reads the disk and write the data to the SCP file supplied.  The callback is for progress, and you can returns FALSE to abort the process
// numTracks is the number of tracks to read.  Usually 80 (0..79), sometimes track 80 and 81 are needed. revolutions is hwo many revolutions of the disk to save (1-5)
// SCP files are a low level flux record of the disk and usually can backup copy protected disks to.  Without special hardware they can't usually be written back to disks.
ADFResult ADFWriter::DiskToSCP(const std::wstring& outputFile, const unsigned int numTracks, const unsigned char revolutions, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const int retryCounter, const int sectorsFound, const int badSectorsFound)> callback) {
	if (!m_device.isOpen()) return ADFResult::adfrDriveError;

	FirmwareVersion v = m_device.getFirwareVersion();
	if ((v.major == 1) && (v.minor < 8)) return  ADFResult::adfrFirmwareTooOld;

	// Higher than this is not supported
	if (numTracks > 82) return ADFResult::adfrDriveError;
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
	header.version = 2 | (2 << 4);    // CHECK
	header.diskType = 0x80;
	header.numRevolutions = revolutions;
	header.startTrack = 0;
	header.numHeads = 0;  // both heads
	header.timeBase = 0;
	header.endTrack = (numTracks*2) - 1;
	header.flags = (1 << BITFLAG_INDEX) | (1 << BITFLAG_NORMALISED);
	header.bitcellEncoding = 0; // 25ns
	// to be calculated
	header.checksum = 0;

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
	unsigned long notPresent = 0;
	for (unsigned int a = 0; a <168; a++) {
		try {
			hADFFile.write((const char*)&notPresent, sizeof(notPresent));
		} catch (...) {
			hADFFile.close();
			return ADFResult::adfrFileIOError;
		}
	}

	RotationExtractor extractor;
	extractor.setAlwaysUseIndex(true);
	RotationExtractor::MFMSample samples[RAW_TRACKDATA_LENGTH];

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

			// Change the surface we're looking at
			if (m_device.selectSurface(surface) != DiagnosticResponse::drOK) {
				hADFFile.close();
				return ADFResult::adfrCompletedWithErrors;
			}

			if (callback) {
				switch (callback(currentTrack, surface, 0, 0, 0)) {
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

			SCPTrackData currentRevData;

			if (m_device.readRotation(extractor, RAW_TRACKDATA_LENGTH, samples, startPatterns, [this, &track, &currentRev, &currentRevData, revolutions](RotationExtractor::MFMSample** _mfmData, unsigned int dataLengthInBits)->bool {
				RotationExtractor::MFMSample* mfmData = *_mfmData;
				unsigned int currentTime = 0;
				unsigned int numBits = 0;

				for (unsigned int a = 0; a < dataLengthInBits; a++) {
					const unsigned int bit = 7 - (a & 7);

					currentTime += (unsigned int)mfmData->bittime[bit];
					numBits++;

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
						numBits = 0;
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

			}) != DiagnosticResponse::drOK) {
				hADFFile.close();
				return ADFResult::adfrDriveError;
			}

			// Get the current position
			unsigned long currentPosition = (unsigned long)hADFFile.tellp();

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
	hADFFile.seekg(sizeof(header), std::fstream::beg);
	unsigned int buffer[128];
	
	while (hADFFile.good()) {
		try {
			hADFFile.read((char*)buffer, sizeof(buffer));
			const unsigned long read = sizeof(buffer) / 4;
			for (size_t pos = 0; pos < read; pos++)
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


// Reads the disk and write the data to the ADF file supplied.  The callback is for progress, and you can returns FALSE to abort the process
ADFResult ADFWriter::DiskToADF(const std::wstring& outputFile, const unsigned int numTracks, std::function < WriteResponse(const int currentTrack, const DiskSurface currentSide, const int retryCounter, const int sectorsFound, const int badSectorsFound)> callback) {
	if (!m_device.isOpen()) return ADFResult::adfrDriveError;

	// Higher than this is not supported
	if (numTracks>82) return ADFResult::adfrDriveError; 

	// Attempt ot open the file
#ifdef _WIN32	
	std::ofstream hADFFile = std::ofstream(outputFile, std::ofstream::out | std::ofstream::binary | std::ofstream::trunc);
#else
	std::string outputFileA;
	quickw2a(outputFile,outputFileA);
	std::ofstream hADFFile = std::ofstream(outputFileA, std::ofstream::out | std::ofstream::binary | std::ofstream::trunc);
#endif	
	if (!hADFFile.is_open()) return ADFResult::adfrFileError;

	// To hold a raw track
	RawTrackData data;
	DecodedTrack track;
	bool includesBadSectors = false;

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
			for (unsigned int sector=0; sector<NUM_SECTORS_PER_TRACK; sector++)
				track.invalidSectors[sector].clear();

			track.validSectors.clear();

			// Extract phase code
			int failureTotal = 0;
			bool ignoreChecksums = false;

			// Repeat until we have all 11 sectors
			while (track.validSectors.size() < NUM_SECTORS_PER_TRACK) {

				if (callback) {
					int total = 0;
					for (unsigned int sector = 0; sector < NUM_SECTORS_PER_TRACK; sector++)
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

					switch (callback(currentTrack, surface, failureTotal, track.validSectors.size(), total)) {
						case WriteResponse::wrContinue: break;  // do nothing
						case WriteResponse::wrRetry:    failureTotal = 0; break;
						case WriteResponse::wrAbort:    hADFFile.close();
														return ADFResult::adfrAborted;

						case WriteResponse::wrSkipBadChecksums: 
							if (ignoreChecksums) {
								// Already been here, so we'll create blank sectors just to get this going
								for (unsigned char sectornumber = 0; sectornumber <= 11; sectornumber++) {
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

				if (m_device.readCurrentTrack(data, false) == DiagnosticResponse::drOK) {
					findSectors(data, currentTrack, surface, AMIGA_WORD_SYNC, track, ignoreChecksums);
					failureTotal++;
				}
				else {
					hADFFile.close();
					return ADFResult::adfrDriveError;
				}

				// If the user wants to skip invalid sectors and save them
				if (ignoreChecksums) {
					for (unsigned int sector = 0; sector < NUM_SECTORS_PER_TRACK; sector++)
						if (track.invalidSectors[sector].size()) {
							includesBadSectors = true;
							break;
						}
					mergeInvalidSectors(track);
				}
			}

			// Sort the sectors in order
			std::sort(track.validSectors.begin(), track.validSectors.end(), [](const DecodedSector & a, const DecodedSector & b) -> bool {
				return a.sectorNumber < b.sectorNumber;
			});

			// Now write all of them to disk
			for (unsigned int sector = 0; sector < 11; sector++) {
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