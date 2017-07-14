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

#include "stdafx.h"
#include "ArduinoInterface.h"

using namespace ArduinoFloppyReader;

// Command that the ARDUINO Sketch understands
#define COMMAND_VERSION    '?'
#define COMMAND_REWIND     '.'
#define COMMAND_GOTOTRACK  '#'
#define COMMAND_HEAD0      '['
#define COMMAND_HEAD1      ']'
#define COMMAND_READTRACK  '<'
#define COMMAND_ENABLE     '+'
#define COMMAND_DISABLE    '-'

// Constructor for this class
ArduinoInterface::ArduinoInterface() {
	m_comPort = INVALID_HANDLE_VALUE;
	m_version = { 0,0 };
}

// Free me
ArduinoInterface::~ArduinoInterface() {
	closePort();
}

// Attempts to open the reader running on the COM port number provided.  Port MUST support 1M baud
InterfaceResult ArduinoInterface::openPort(const unsigned int portNumber) {
	closePort();

	// Communicate with the serial port
	char buffer[20];
	sprintf_s(buffer, "\\\\.\\COM%i", portNumber);
	m_comPort = CreateFileA(buffer, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, 0);

	// No com port? Error!
	if (m_comPort == INVALID_HANDLE_VALUE) return InterfaceResult::irError;

	// Prepare communication settings
	COMMCONFIG config;
	DWORD comConfigSize = sizeof(config);
	memset(&config, 0, sizeof(config));

	GetCommConfig(m_comPort, &config, &comConfigSize);
	config.dwSize = sizeof(config);
	config.dcb.DCBlength = sizeof(config.dcb);
	config.dcb.BaudRate = 1000000;  // 1M baudrate
	config.dcb.ByteSize = 8;        // 8-bit
	config.dcb.fBinary = true;
	config.dcb.Parity = false;
	config.dcb.fOutxCtsFlow = false;
	config.dcb.fOutxDsrFlow = false;
	config.dcb.fDtrControl = DTR_CONTROL_DISABLE;
	config.dcb.fDsrSensitivity = false;
	config.dcb.fNull = false;
	config.dcb.fRtsControl = RTS_CONTROL_DISABLE;
	config.dcb.fAbortOnError = false;
	config.dcb.StopBits = 0;  // 1 stop bit

	// Try to put the serial port in the mode we require
	if (!SetCommConfig(m_comPort, &config, sizeof(config))) {
		closePort();
		return InterfaceResult::irCommError;
	}

	// Setup port timeouts
	COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout = 2000;
	timeouts.ReadTotalTimeoutConstant = 2000;
	timeouts.ReadTotalTimeoutMultiplier =200;
	timeouts.WriteTotalTimeoutConstant = 2000;
	timeouts.WriteTotalTimeoutMultiplier = 200;
	if (!SetCommTimeouts(m_comPort, &timeouts)) {
		closePort();
		return InterfaceResult::irCommError;
	}

	if (runCommand(COMMAND_VERSION) != InterfaceResult::irOK) {
		closePort();
		return InterfaceResult::irCommError;
	}

	// Version is always 4 bytes, we're gonna read them in one by one, and if any are wrong we exit with an error
	char versionBuffer[4];
	if (!deviceRead(versionBuffer, 4)) {
		closePort();
		return InterfaceResult::irCommError;
	}

	// Now check the response
	if ((versionBuffer[0] != 'V') || (versionBuffer[2] != '.')) {
		closePort();
		return InterfaceResult::irCommError;
	}

	// Looks like its formatted correctly.  There's a good chance this is our device
	m_version.major = versionBuffer[1] - '0';
	m_version.minor = versionBuffer[3] - '0';

	// Ok, success
	return InterfaceResult::irOK;
};

// Closes the port down
void ArduinoInterface::closePort() {
	if (m_comPort != INVALID_HANDLE_VALUE) {
		// Force the drive to power down
		enableReading(false);
		// And close the handle
		CloseHandle(m_comPort);
		m_comPort = INVALID_HANDLE_VALUE;
	}
}

// Returns true if the track actually contains some data, else its considered blank or unformatted
bool ArduinoInterface::trackContainsData(const RawTrackData& trackData) {
	int zerocount=0, ffcount = 0;
	unsigned char lastByte = trackData[0];
	for (unsigned int counter = 1; counter < RAW_TRACKDATA_LENGTH; counter++) {
		if (trackData[counter]==lastByte) {
			switch (lastByte) {
			case 0xFF: ffcount++; zerocount = 0; break;
			case 0x00: ffcount=0; zerocount++; break;
			default: zerocount = 0; ffcount = 0;
			}
		}
		else {
			lastByte = trackData[counter];
			zerocount = 0; ffcount = 0;
		}
	}

	// More than this in a row and its bad
	return ((ffcount < 20) && (zerocount < 20));
}

// Turns on and off the reading interface
InterfaceResult ArduinoInterface::enableReading(const bool enable, const bool reset) {
	if (enable) {
		// Enable the device
		if (runCommand(COMMAND_ENABLE) != InterfaceResult::irOK) return InterfaceResult::irCommError;

		// Reset?
		if (reset) {
			// And rewind to the first track
			if (runCommand(COMMAND_REWIND) != InterfaceResult::irOK) return InterfaceResult::irCommError;
			// Lets know where we are
			return selectSurface(DiskSurface::dsUpper);
		}
		return InterfaceResult::irOK;
	}
	else {
		// Disable the device
		if (runCommand(COMMAND_DISABLE) != InterfaceResult::irOK) return InterfaceResult::irCommError;
		return InterfaceResult::irOK;
	}
}

// Select the track, this makes the motor seek to this position
InterfaceResult ArduinoInterface::selectTrack(const unsigned char trackIndex) {
	if (trackIndex > 81) return InterfaceResult::irError; // no chance, it can't be done.

	// And send the command and track.  This is sent as ASCII text as a result of terminal testing.  Easier to see whats going on
	char buf[4];
	sprintf_s(buf, "%c%02i", COMMAND_GOTOTRACK,trackIndex);

	// Send track number
	if (!deviceWrite(buf,3)) return InterfaceResult::irCommError;

	// Get result
	char result;

	if (!deviceRead(&result,1)) return InterfaceResult::irCommError;
	switch (result) {
		case '1': return InterfaceResult::irOK;
		case '0': return InterfaceResult::irError;
		default: return InterfaceResult::irCommError;
	}
}

// Choose which surface of the disk to read from
InterfaceResult ArduinoInterface::selectSurface(const DiskSurface side) {
	return runCommand((side == DiskSurface::dsUpper) ? COMMAND_HEAD0 : COMMAND_HEAD1);
}

// Read RAW data from the current track and surface selected using the supplied phase.  Phase must be 0123456789ABCDEF - see notes
InterfaceResult ArduinoInterface::readCurrentTrack(const char phase, RawTrackData& trackData) {
	InterfaceResult ir = runCommand(COMMAND_READTRACK, phase);
	
	// Allow command retry
	if (ir != InterfaceResult::irOK) {
		// Clear the buffer
		deviceRead(&trackData, RAW_TRACKDATA_LENGTH);
		ir = runCommand(COMMAND_READTRACK, phase);
		if (ir != InterfaceResult::irOK) return ir;
	}

	// Failed to read the track?
	if (!deviceRead(&trackData, RAW_TRACKDATA_LENGTH)) {
		ir = runCommand(COMMAND_READTRACK, phase);
		if (ir != InterfaceResult::irOK) return ir;

		return deviceRead(&trackData, RAW_TRACKDATA_LENGTH) ? InterfaceResult::irOK : InterfaceResult::irCommError;
	}
	else return InterfaceResult::irOK;
}

// Run a command that returns 1 or 0 for its response
InterfaceResult ArduinoInterface::runCommand(const char command, const char parameter) {
	unsigned char response;

	// Send the command
	if (!deviceWrite(&command,1)) return InterfaceResult::irCommError;

	// Only send the parameter if its not NULL
	if (parameter!='\0') 
		if (!deviceWrite(&parameter,1)) return InterfaceResult::irCommError;

	// And read the response
	if (!deviceRead(&response,1)) return InterfaceResult::irCommError;

	// Evaluate the response
	switch (response) {
		case '1': return InterfaceResult::irOK;
		case '0': return InterfaceResult::irError;
		default: return InterfaceResult::irCommError;
	}
}

// Read a desired number of bytes into the target pointer
bool ArduinoInterface::deviceRead(void* target, const unsigned int numBytes) {
	DWORD read;
	if (m_comPort == INVALID_HANDLE_VALUE) return false;

	if (!ReadFile(m_comPort, target, numBytes, &read, NULL)) return false;
	if (read < numBytes) {
		// Clear the unread bytes
		char* target2 = ((char*)target) + read;
		memset(target2, 0, numBytes - read);
		return true;
	}
	else return true;
}

// Writes a desired number of bytes from the the pointer
bool ArduinoInterface::deviceWrite(const void* source, const unsigned int numBytes) {
	DWORD written;
	if (m_comPort == INVALID_HANDLE_VALUE) return false;
	return (WriteFile(m_comPort, source, numBytes, &written, NULL)) && (written == numBytes);
}
