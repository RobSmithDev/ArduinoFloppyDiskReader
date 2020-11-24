/* ArduinoFloppyReader (and writer)
*
* Copyright (C) 2017-2020 Robert Smith (@RobSmithDev)
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

////////////////////////////////////////////////////////////////////////////////////////
// Class to manage the communication between the computer and the Arduino             //
////////////////////////////////////////////////////////////////////////////////////////
//
// Purpose:
// The class handles the command interface to the arduino.  It doesn't do any decoding
// Just open ports, switch motors on and off, seek to tracks etc.
//
//
//

#pragma once
#ifdef USING_MFC
#include <afxwin.h>
#else
#include <Windows.h>
#endif
#include <string>

// Paula on the Amiga used to find the SYNC then read 1900 WORDS. (12868 bytes)
// As the PC is doing the SYNC we need to read more than this to allow a further overlap
// This number must match what the sketch in the Arduino is set to. 
#define RAW_TRACKDATA_LENGTH    (0x1900*2+0x440)
// With the disk spinning at 300rpm, and data rate of 500kbps, for a full revolution we should receive 12500 bytes of data (12.5k)
// The above buffer assumes a full Paula data capture plsu the size of a sector.


namespace ArduinoFloppyReader {

	// Array to hold data from a floppy disk read
	typedef unsigned char RawTrackData[RAW_TRACKDATA_LENGTH];

	// Sketch firmware version
	struct FirmwareVersion {
		unsigned char major, minor;
	};


	// Represent which side of the disk we're looking at
	enum class DiskSurface {
							dsUpper,            // The upper side of the disk
							dsLower             // The lower side of the disk
						};

	// Diagnostic responses from the interface
	enum class DiagnosticResponse {
								drOK,

								// Responses from openPort
								drPortInUse,
								drPortNotFound,
								drPortError,
								drAccessDenied,
								drComportConfigError,
								drBaudRateNotSupported,
								drComportTimeoutsError,
								drErrorReadingVersion,
								drErrorMalformedVersion,
								drOldFirmware,

								// Responses from commands
								drSendFailed,
								drSendParameterFailed,
								drReadResponseFailed,
								drError,

								// Response from selectTrack
								drTrackRangeError,
								drSelectTrackError,
								drWriteProtected,
								drStatusError,
								drSendDataFailed,
								drTrackWriteResponseError,

								drDiagnosticNotAvailable,
								drCTSFailure,
								drRewindFailure,
							};

	enum class LastCommand {
		lcOpenPort,
		lcGetVersion,
		lcEnableWrite,
		lcRewind,
		lcDisableMotor,
		lcEnableMotor,
		lcGotoTrack,
		lcSelectSurface,
		lcReadTrack,
		lcWriteTrack,
		lcEraseTrack,
		lcRunDiagnostics
	};

	class ArduinoInterface {
	private:
		// Windows handle to the serial port device
		HANDLE			m_comPort;
		FirmwareVersion m_version;
		bool			m_inWriteMode;
		LastCommand		m_lastCommand;
		DiagnosticResponse m_lastError;

		// Read a desired number of bytes into the target pointer
		bool deviceRead(void* target, const unsigned int numBytes);
		// Writes a desired number of bytes from the the pointer
		bool deviceWrite(const void* source, const unsigned int numBytes);

		// Version of the above where the command has a parameter on the end (as long as its not char 0)
		DiagnosticResponse runCommand(const char command, const char parameter = '\0', char* actualResponse = nullptr);

	public:
		// Constructor for this class
		ArduinoInterface();

		// Free me
		~ArduinoInterface();

		const LastCommand getLastFailedCommand() const { return m_lastCommand;  };
		const DiagnosticResponse getLastError() const { return m_lastError; };

		// Uses the above fields to constructr a suitable error message, hopefully useful in diagnosing the issue
		const std::string getLastErrorStr() const;

		const bool isOpen() const { return m_comPort != INVALID_HANDLE_VALUE; };
		const bool isInWriteMode() const { return m_inWriteMode; };

		// Get the current firmware version.  Only valid if openPort is successful
		const FirmwareVersion getFirwareVersion() const { return m_version; };

		// Turns on and off the reading interface
		DiagnosticResponse enableReading(const bool enable, const bool reset = true);

		// Turns on and off the reading interface. If irError is returned the disk is write protected
		DiagnosticResponse  enableWriting(const bool enable, const bool reset = true);

		// Attempts to open the reader running on the COM port number provided.  Port MUST support 2M baud
		DiagnosticResponse openPort(const unsigned int portNumber, bool enableCTSflowcontrol = true);

		// Seek to track 0
		DiagnosticResponse findTrack0();

		// Check CTS status by asking the device to set it and then checking what happened
		DiagnosticResponse testIndexPulse();

		// Check CTS status by asking the device to set it and then checking what happened
		DiagnosticResponse testDataPulse();

		// Select the track, this makes the motor seek to this position
		DiagnosticResponse  selectTrack(const unsigned char trackIndex);

		// Choose which surface of the disk to read from
		DiagnosticResponse  selectSurface(const DiskSurface side);

		// Read RAW data from the current track and surface selected 
		DiagnosticResponse  readCurrentTrack(RawTrackData& trackData, const bool readFromIndexPulse);

		// Attempts to write a sector back to the disk.  This must be pre-formatted and MFM encoded correctly
		DiagnosticResponse  writeCurrentTrack(const unsigned char*, const unsigned short numBytes, const bool writeFromIndexPulse);

		// Asks the Arduino to wipe the current track, filling it with the 0xAA pattern
		DiagnosticResponse eraseCurrentTrack();

		// Check CTS status 
		DiagnosticResponse testCTS(const unsigned int portNumber);

		// Returns true if the track actually contains some data, else its considered blank or unformatted
		bool trackContainsData(const RawTrackData& trackData) const;

		// Closes the port down
		void closePort();
	};

};