/* ArduinoFloppyReader (and writer)
*
* Copyright (C) 2017-2018 Robert Smith (@RobSmithDev)
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
#include <sstream>

using namespace ArduinoFloppyReader;

// Command that the ARDUINO Sketch understands
#define COMMAND_VERSION            '?'
#define COMMAND_REWIND             '.'
#define COMMAND_GOTOTRACK          '#'
#define COMMAND_HEAD0              '['
#define COMMAND_HEAD1              ']'
#define COMMAND_READTRACK          '<'
#define COMMAND_ENABLE             '+'
#define COMMAND_DISABLE            '-'
#define COMMAND_WRITETRACK         '>'
#define COMMAND_ENABLEWRITE        '~'
#define COMMAND_ERASETRACK         'X'
#define COMMAND_DIAGNOSTICS        '&'


// Convert the last executed command that had an error to a string
std::string lastCommandToName(LastCommand cmd) {
	switch (cmd) {
	case lcOpenPort:		return "OpenPort";
	case lcGetVersion:		return "GetVersion";
	case lcEnableWrite:		return "EnableWrite";
	case lcRewind:			return "Rewind";
	case lcDisableMotor:	return "DisableMotor";
	case lcEnableMotor:		return "EnableMotor";
	case lcGotoTrack:		return "GotoTrack";
	case lcSelectSurface:	return "SelectSurface";
	case lcReadTrack:		return "ReadTrack";
	case lcWriteTrack:		return "WriteTrack";
	case lcRunDiagnostics:	return "RunDiagnostics";
	case lcEraseTrack:		return "EraseTrack";
	default:				return "Unknown";
	}
}

// Uses the above fields to constructr a suitable error message, hopefully useful in diagnosing the issue
const std::string ArduinoInterface::getLastErrorStr() const {
	std::stringstream tmp;
	switch (m_lastError) {
	case drOldFirmware: return "The Arduino is running an older version of the firmware/sketch.  Please re-upload.";
	case drOK: return "Last command completed successfully."; 
	case drPortInUse: return "The specified COM port is currently in use by another application.";
	case drPortNotFound: return "The specified COM port was not found.";
	case drAccessDenied: return "The operating system denied access to the specified COM port.";
	case drComportConfigError: return "We were unable to configure the COM port using the SetCommConfig() command.";
	case drBaudRateNotSupported: return "The COM port does not support the 2M baud rate required by this application.";
	case drErrorReadingVersion: return "An error occured attempting to read the version of the sketch running on the Arduino.";
	case drErrorMalformedVersion: return "The Arduino returned an unexpected string when version was requested.  This could be a baud rate mismatch or incorrect loaded sketch.";
	case drCTSFailure: return "Diagnostics report the CTS line is not connected correctly or is not behaving correctly.";
	case drTrackRangeError: return "An error occured attempting to go to a track number that was out of allowed range.";
	case drWriteProtected: return "Unable to write to the disk.  The disk is write protected.";
	case drPortError: return "An unknown error occured attempting to open access to the specified COM port.";
	case drComportTimeoutsError: return "An error occured attempting to set the timeouts on the specified COM port.";
	case drDiagnosticNotAvailable: return "CTS diagnostic not available, command GetCommModemStatus failed to execute.";
	case drSelectTrackError: return "Arduino reported an error seeking to a specific track.";
	case drTrackWriteResponseError: return "Error receiving status from Arduino after a track write operation.";
	case drSendDataFailed: return "Error sending track data to be written to disk.  This could be a COM timeout.";
	case drRewindFailure: return "Arduino was unable to find track 0.  This could be a wiring fault or power supply failure.";
	case drError:	tmp << "Arduino responded with an error running the " << lastCommandToName(m_lastCommand) << " command.";
					return tmp.str();

	case drReadResponseFailed:
		switch (m_lastCommand) {
		case lcGotoTrack: return "Unable to read response from Arduino after requesting to go to a specific track";
		case lcReadTrack: return "Gave up trying to read a full track from the disk.";
		case lcWriteTrack: return "Unable to read response to requesting to write a track.";
		default: tmp << "Error reading response from the Arduino while running command " << lastCommandToName(m_lastCommand) << ".";
				 return tmp.str();
		}

	case drSendFailed:
		if (m_lastCommand == lcGotoTrack) 
			return "Unable to send the complete select track command to the Arduino.";
		else {
			tmp << "Error sending the command " << lastCommandToName(m_lastCommand) << " to the Arduino.";
			return tmp.str();
		}

	case drSendParameterFailed:	tmp << "Unable to send a parameter while executing the " << lastCommandToName(m_lastCommand) << " command.";
		return tmp.str();
	case drStatusError: tmp << "An unknown response was was received from the Arduino while executing the " << lastCommandToName(m_lastCommand) << " command.";
		return tmp.str();

	default: return "Unknown error.";
	}
}



// Constructor for this class
ArduinoInterface::ArduinoInterface() {
	m_comPort = INVALID_HANDLE_VALUE;
	m_version = { 0,0 };
}

// Free me
ArduinoInterface::~ArduinoInterface() {
	closePort();
}


// Check CTS status by asking the device to set it and then checking what happened
DiagnosticResponse ArduinoInterface::testIndexPulse() {
	// Port opned.  We need to check what hapens as the pin is toggled
	m_lastError = runCommand(COMMAND_DIAGNOSTICS, '3');
	if (m_lastError != drOK) {
		m_lastCommand = lcRunDiagnostics;
		return m_lastError;
	}
	return m_lastError;
}

// Check CTS status by asking the device to set it and then checking what happened
DiagnosticResponse ArduinoInterface::testDataPulse() {
	// Port opned.  We need to check what hapens as the pin is toggled
	m_lastError = runCommand(COMMAND_DIAGNOSTICS, '4');
	if (m_lastError != drOK) {
		m_lastCommand = lcRunDiagnostics;
		return m_lastError;
	}
	return m_lastError;
}

// Check CTS status by asking the device to set it and then checking what happened
DiagnosticResponse ArduinoInterface::testCTS(const unsigned int portNumber) {
	m_lastError = openPort(portNumber, false);
	if (m_lastError != DiagnosticResponse::drOK) return m_lastError;

	for (int a = 1; a <= 10; a++) {
		// Port opned.  We need to check what hapens as the pin is toggled
		m_lastError = runCommand(COMMAND_DIAGNOSTICS, (a&1)?'1':'2');
		if (m_lastError != drOK) {
			m_lastCommand = lcRunDiagnostics;
			closePort();
			return m_lastError;
		}
		Sleep(1);
		// Check the state of it
		DWORD mask;

		if (!GetCommModemStatus(m_comPort, &mask)) {
			closePort();
			m_lastError = DiagnosticResponse::drDiagnosticNotAvailable;
			return m_lastError;
		}

		// This doesnt actually run a command, this switches the CTS line back to its default setting
		m_lastError = runCommand(COMMAND_DIAGNOSTICS);

		if (((mask&MS_CTS_ON)!=0)^((a&1)!=0)) {
			// If we get here then the CTS value isn't what it should be
			closePort();
			m_lastError = DiagnosticResponse::drCTSFailure;
			return m_lastError;
		}
		// Pass.  Try the other state
		Sleep(1);
	}

	closePort();

	return DiagnosticResponse::drOK;
}

// Attempts to open the reader running on the COM port number provided.  Port MUST support 2M baud
DiagnosticResponse ArduinoInterface::openPort(const unsigned int portNumber, bool enableCTSflowcontrol) {
	m_lastCommand = lcOpenPort;
	closePort();

	// Communicate with the serial port
	char buffer[20];
	sprintf_s(buffer, "\\\\.\\COM%i", portNumber);
	m_comPort = CreateFileA(buffer, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, 0);

	// No com port? Error!
	if (m_comPort == INVALID_HANDLE_VALUE) {
		int i= GetLastError();
		switch (i) {
		case ERROR_FILE_NOT_FOUND:  m_lastError = DiagnosticResponse::drPortNotFound;
			return m_lastError;		
		case ERROR_ACCESS_DENIED:   m_lastError = DiagnosticResponse::drPortInUse;
									return m_lastError;
		default: m_lastError = DiagnosticResponse::drPortError;
				 return m_lastError;
		}
	}

	// Prepare communication settings
	COMMCONFIG config;
	DWORD comConfigSize = sizeof(config);
	memset(&config, 0, sizeof(config));

	GetCommConfig(m_comPort, &config, &comConfigSize);
	config.dwSize = sizeof(config);
	config.dcb.DCBlength = sizeof(config.dcb);
	config.dcb.BaudRate = 2000000;  // 2M baudrate
	config.dcb.ByteSize = 8;        // 8-bit
	config.dcb.fBinary = true;
	config.dcb.Parity = false;
	config.dcb.fOutxCtsFlow = enableCTSflowcontrol;  // Turn on CTS flow control
	config.dcb.fOutxDsrFlow = false;
	config.dcb.fDtrControl = DTR_CONTROL_DISABLE;
	config.dcb.fDsrSensitivity = false;
	config.dcb.fNull = false;
	config.dcb.fTXContinueOnXoff = false;
	config.dcb.fRtsControl = RTS_CONTROL_DISABLE;
	config.dcb.fAbortOnError = false;
	config.dcb.StopBits = 0;  // 1 stop bit
	config.dcb.fOutX = 0;  
	config.dcb.fInX = 0;
	config.dcb.fErrorChar = 0;
	config.dcb.fAbortOnError = 0;
	config.dcb.fInX = 0;
				  
	// Try to put the serial port in the mode we require
	if (!SetCommConfig(m_comPort, &config, sizeof(config))) {
		// If it failed something went wrong.  We'll change the baud rate to see if its that
		config.dcb.BaudRate = 9600;
		if (!SetCommConfig(m_comPort, &config, sizeof(config))) {
			closePort();
			m_lastError = DiagnosticResponse::drComportConfigError;
			return m_lastError;
		}
		else {
			closePort();
			m_lastError = DiagnosticResponse::drBaudRateNotSupported;
			return m_lastError;
		}
	}

	// Setup port timeouts
	COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout = 2000;
	timeouts.ReadTotalTimeoutConstant = 2000;
	timeouts.ReadTotalTimeoutMultiplier = 200;
	timeouts.WriteTotalTimeoutConstant = 2000;
	timeouts.WriteTotalTimeoutMultiplier = 200;
	if (!SetCommTimeouts(m_comPort, &timeouts)) {
		closePort();
		m_lastError = DiagnosticResponse::drComportTimeoutsError;
		return m_lastError;
	}

	// Request version from the Arduino device running our software
	m_lastError = runCommand(COMMAND_VERSION);
	if (m_lastError != DiagnosticResponse::drOK) {
		m_lastCommand = lcGetVersion;
		closePort();
		return m_lastError;
	}

	// Version is always 4 bytes, we're gonna read them in one by one, and if any are wrong we exit with an error
	char versionBuffer[4];
	if (!deviceRead(versionBuffer, 4)) {
		closePort();
		m_lastError = DiagnosticResponse::drErrorReadingVersion;
		return m_lastError;
	}

	// Now check the response
	if ((versionBuffer[0] != 'V') || (versionBuffer[2] != '.')) {
		closePort();
		m_lastError = DiagnosticResponse::drErrorMalformedVersion;
		return m_lastError;
	}

	// Looks like its formatted correctly.  There's a good chance this is our device
	m_version.major = versionBuffer[1] - '0';
	m_version.minor = versionBuffer[3] - '0';

	if ((m_version.major == 1) && (m_version.minor < 2)) {
		// Ok, success
		m_lastError = DiagnosticResponse::drOldFirmware;
		return m_lastError;
	}

	// Ok, success
	m_lastError = DiagnosticResponse::drOK;
	return m_lastError;
}

// Closes the port down
void ArduinoInterface::closePort() {
	if (m_comPort != INVALID_HANDLE_VALUE) {
		// Force the drive to power down
		enableReading(false);
		// And close the handle
		CloseHandle(m_comPort);
		m_comPort = INVALID_HANDLE_VALUE;
	}
	m_inWriteMode = false;
}

// Returns true if the track actually contains some data, else its considered blank or unformatted
bool ArduinoInterface::trackContainsData(const RawTrackData& trackData) const {
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

// Turns on and off the writing interface.  If irError is returned the disk is write protected
DiagnosticResponse ArduinoInterface::enableWriting(const bool enable, const bool reset) {
	if (enable) {
		// Enable the device
		m_lastError = runCommand(COMMAND_ENABLEWRITE);
		if (m_lastError == DiagnosticResponse::drError) {
			m_lastCommand = lcEnableWrite;
			m_lastError = DiagnosticResponse::drWriteProtected;
			return m_lastError;
		}
		if (m_lastError != DiagnosticResponse::drOK) {
			m_lastCommand = lcEnableWrite;
			return m_lastError;
		}
		m_inWriteMode = true;

		// Reset?
		if (reset) {
			// And rewind to the first track
			m_lastError = findTrack0();
			if (m_lastError != DiagnosticResponse::drOK) return m_lastError;

			// Lets know where we are
			return selectSurface(DiskSurface::dsUpper);
		}
		m_lastError = DiagnosticResponse::drOK;
		return m_lastError;
	}
	else {
		// Disable the device
		m_lastError = runCommand(COMMAND_DISABLE);
		if (m_lastError != DiagnosticResponse::drOK) {
			m_lastCommand = lcDisableMotor;
			return m_lastError;
		}
		m_inWriteMode = false;
		
		return m_lastError;
	}
}

DiagnosticResponse ArduinoInterface::findTrack0() {
	// And rewind to the first track
	char status = '0';
	m_lastError = runCommand(COMMAND_REWIND, '\000', &status);
	if (m_lastError != DiagnosticResponse::drOK) {
		m_lastCommand = lcRewind;
		if (status = '#') return drRewindFailure;
		return m_lastError;
	}
	return m_lastError;
}

// Turns on and off the reading interface
DiagnosticResponse ArduinoInterface::enableReading(const bool enable, const bool reset) {
	m_inWriteMode = false;
	if (enable) {
		// Enable the device
		m_lastError = runCommand(COMMAND_ENABLE);
		if (m_lastError != DiagnosticResponse::drOK) {
			m_lastCommand = lcEnableMotor;
			return m_lastError;
		}

		// Reset?
		if (reset) {
			m_lastError = findTrack0();
			if (m_lastError != DiagnosticResponse::drOK) return m_lastError;

			// Lets know where we are
			return selectSurface(DiskSurface::dsUpper);
		}
		m_lastError = DiagnosticResponse::drOK;
		return m_lastError;

	}
	else {
		// Disable the device
		m_lastError = runCommand(COMMAND_DISABLE);
		if (m_lastError != DiagnosticResponse::drOK) {
			m_lastCommand = lcDisableMotor;
			return m_lastError;
		}

		return m_lastError;
	}
}

// Select the track, this makes the motor seek to this position
DiagnosticResponse ArduinoInterface::selectTrack(const unsigned char trackIndex) {
	if (trackIndex > 81) {
		m_lastError = DiagnosticResponse::drTrackRangeError;
		return m_lastError; // no chance, it can't be done.
	}

	// And send the command and track.  This is sent as ASCII text as a result of terminal testing.  Easier to see whats going on
	char buf[4];
	sprintf_s(buf, "%c%02i", COMMAND_GOTOTRACK,trackIndex);

	// Send track number
	if (!deviceWrite(buf, 3)) {
		m_lastCommand = lcGotoTrack;
		m_lastError = DiagnosticResponse::drSendFailed;
		return m_lastError;
	}

	// Get result
	char result;

	if (!deviceRead(&result, 1)) {
		m_lastCommand = lcGotoTrack;
		m_lastError = DiagnosticResponse::drReadResponseFailed;
		return m_lastError;
	}
	
	switch (result) {
		case '1':   m_lastError = DiagnosticResponse::drOK;
					break;
		case '0':   m_lastCommand = lcGotoTrack; 
					m_lastError = DiagnosticResponse::drSelectTrackError;
					break;
		default:	m_lastCommand = lcGotoTrack;
					m_lastError = DiagnosticResponse::drStatusError;
					break;
	}
	return m_lastError;
}

// Choose which surface of the disk to read from
DiagnosticResponse ArduinoInterface::selectSurface(const DiskSurface side) {
	m_lastError = runCommand((side == DiskSurface::dsUpper) ? COMMAND_HEAD0 : COMMAND_HEAD1);
	if (m_lastError != DiagnosticResponse::drOK) {
		m_lastCommand = lcSelectSurface;
		return m_lastError;
	}
	return m_lastError;
}


void writeBit(RawTrackData& output, int& pos, int& bit, int value) {
	if (pos >= RAW_TRACKDATA_LENGTH) return;

	output[pos] <<= 1;
	output[pos] |= value;
	bit++;
	if (bit >= 8) {
		pos++;
		bit = 0;
	}
}

void unpack(const RawTrackData& data, RawTrackData& output) {
	int pos = 0;
	int index = 0;
	int p2 = 0;
	memset(output, 0, sizeof(output));
	while (pos < RAW_TRACKDATA_LENGTH) {
		// Each byte contains four pairs of bits that identify an MFM sequence to be encoded

		for (int b = 6; b >= 0; b -= 2) {
			unsigned char value = (data[index] >> b) & 3;
			switch (value) {
			case 0: // This is an '1'
				// This can't happen, its an END OF DATA marker
				return;				
				break;
			case 1: // This is an '01'
				writeBit(output, pos, p2, 0);
				writeBit(output, pos, p2, 1);
				break;
			case 2: // This is an '001'
				writeBit(output, pos, p2, 0);
				writeBit(output, pos, p2, 0);
				writeBit(output, pos, p2, 1);
				break;
			case 3: // this is an '0001'
				writeBit(output, pos, p2, 0);
				writeBit(output, pos, p2, 0);
				writeBit(output, pos, p2, 0);
				writeBit(output, pos, p2, 1);
				break;
			}
		}
		index++;
		if (index >= sizeof(data)) return;
	}
	// There will be left-over data
}

// Read RAW data from the current track and surface 
DiagnosticResponse ArduinoInterface::readCurrentTrack(RawTrackData& trackData, const bool readFromIndexPulse) {
	m_lastError = runCommand(COMMAND_READTRACK);
	
	RawTrackData tmp;

	// Allow command retry
	if (m_lastError != DiagnosticResponse::drOK) {
		// Clear the buffer
		deviceRead(&tmp, RAW_TRACKDATA_LENGTH);
		m_lastError = runCommand(COMMAND_READTRACK);
		if (m_lastError != DiagnosticResponse::drOK) {
			m_lastCommand = lcReadTrack;
			return m_lastError;
		}
	}

	unsigned char signalPulse = readFromIndexPulse ? 1 : 0;
	if (!deviceWrite(&signalPulse, 1)) {
		m_lastCommand = lcReadTrack;
		m_lastError = DiagnosticResponse::drSendParameterFailed;
		return m_lastError;
	}

	// Keep reading until he hit RAW_TRACKDATA_LENGTH or a null byte is recieved
	int bytePos = 0;
	int readFail = 0;
	for (;;) {
		unsigned char value;
		if (deviceRead(&value, 1)) {
			if (value == 0) break; else
				if (bytePos < RAW_TRACKDATA_LENGTH) tmp[bytePos++] = value;
		}
		else {
			readFail++;
			if (readFail > 4) {
				m_lastCommand = lcReadTrack;
				m_lastError = DiagnosticResponse::drReadResponseFailed;
				return m_lastError;
			}
		}
	}
	unpack(tmp, trackData);
	m_lastError = DiagnosticResponse::drOK;
	return m_lastError;
}

// Asks the Arduino to wipe the current track
DiagnosticResponse ArduinoInterface::eraseCurrentTrack() {
	m_lastError = runCommand(COMMAND_ERASETRACK);
	if (m_lastError != DiagnosticResponse::drOK) {
		m_lastCommand = lcEraseTrack;
		return m_lastError;
	}

	unsigned char chr;
	if (!deviceRead(&chr, 1)) {
		m_lastCommand = lcEraseTrack;
		m_lastError = DiagnosticResponse::drReadResponseFailed;
		return m_lastError;
	}

	// 'N' means NO Writing, aka write protected
	if (chr == 'N') {
		m_lastCommand = lcEraseTrack;
		m_lastError = DiagnosticResponse::drWriteProtected;
		return m_lastError;
	}
	if (chr != 'Y') {
		m_lastCommand = lcEraseTrack;
		m_lastError = DiagnosticResponse::drStatusError;
		return m_lastError;
	}

	unsigned char response;
	if (!deviceRead(&response, 1)) {
		m_lastCommand = lcEraseTrack;
		m_lastError = DiagnosticResponse::drReadResponseFailed;
		return m_lastError;
	}
	// If this is a '1' then the Arduino erased the track
	if (response != '1') {
		m_lastCommand = lcEraseTrack;
		m_lastError = DiagnosticResponse::drStatusError;
		return m_lastError;
	}

	m_lastError = DiagnosticResponse::drOK;
	return m_lastError;
}

// Writes RAW data onto the current track
DiagnosticResponse ArduinoInterface::writeCurrentTrack(const unsigned char* data, const unsigned short numBytes, const bool writeFromIndexPulse) {
	m_lastError = runCommand(COMMAND_WRITETRACK);
	if (m_lastError != DiagnosticResponse::drOK) {
		m_lastCommand = lcWriteTrack;
		return m_lastError;
	}
	unsigned char chr;
	if (!deviceRead(&chr, 1)) {
		m_lastCommand = lcWriteTrack;
		m_lastError = DiagnosticResponse::drReadResponseFailed;
		return m_lastError;
	}

	// 'N' means NO Writing, aka write protected
	if (chr == 'N') {
		m_lastCommand = lcWriteTrack;
		m_lastError = DiagnosticResponse::drWriteProtected;
		return m_lastError;
	}
	if (chr != 'Y') {
		m_lastCommand = lcWriteTrack;
		m_lastError = DiagnosticResponse::drStatusError;
		return m_lastError;
	}

	// Now we send the number of bytes we're planning on transmitting
	unsigned char b = (numBytes >> 8);
	if (!deviceWrite(&b, 1)) {
		m_lastCommand = lcWriteTrack; 
		m_lastError = DiagnosticResponse::drSendParameterFailed;
		return m_lastError;
	}
	b = numBytes & 0xFF;
	if (!deviceWrite(&b, 1)) {
		m_lastCommand = lcWriteTrack; 
		m_lastError = DiagnosticResponse::drSendParameterFailed;
		return m_lastError;
	}

	// Explain if we want index pulse sync writing (slower and not required by normal AmigaDOS disks)
	b = writeFromIndexPulse ? 1 : 0;
	if (!deviceWrite(&b, 1)) {
		m_lastCommand = lcWriteTrack;
		m_lastError = DiagnosticResponse::drSendParameterFailed;
		return m_lastError;
	}

	unsigned char response;
	if (!deviceRead(&response, 1)) {
		m_lastCommand = lcWriteTrack;
		m_lastError = DiagnosticResponse::drReadResponseFailed;
		return m_lastError;
	}
	
	if (response != '!') {
		m_lastCommand = lcWriteTrack;
		m_lastError = DiagnosticResponse::drStatusError;
		return m_lastError;
	}

	if (!deviceWrite((const void*)data, numBytes)) {
		m_lastCommand = lcWriteTrack;
		m_lastError = DiagnosticResponse::drSendDataFailed;
		return m_lastError;
	}

	if (!deviceRead(&response, 1)) {
		m_lastCommand = lcWriteTrack;
		m_lastError = DiagnosticResponse::drTrackWriteResponseError;
		return m_lastError;
	}

	// If this is a '1' then the Arduino didn't miss a single bit!
	if (response != '1') {
		m_lastCommand = lcWriteTrack;
		m_lastError = DiagnosticResponse::drStatusError;
		return m_lastError;
	}

	m_lastError = DiagnosticResponse::drOK;
	return m_lastError;
}

// Run a command that returns 1 or 0 for its response
DiagnosticResponse ArduinoInterface::runCommand(const char command, const char parameter, char* actualResponse) {
	unsigned char response;

	// Send the command
	if (!deviceWrite(&command, 1)) {
		m_lastError = DiagnosticResponse::drSendFailed;
		return m_lastError;
	}

	// Only send the parameter if its not NULL
	if (parameter!='\0') 
		if (!deviceWrite(&parameter, 1)) {
			m_lastError = DiagnosticResponse::drSendParameterFailed;
			return m_lastError;
		}

	// And read the response
	if (!deviceRead(&response, 1)) {
		m_lastError = DiagnosticResponse::drReadResponseFailed;
		return m_lastError;
	}

	if (actualResponse) *actualResponse = response;

	// Evaluate the response
	switch (response) {
		case '1': m_lastError = DiagnosticResponse::drOK;
			      break;
		case '0': m_lastError = DiagnosticResponse::drError;
				  break;
		default:  m_lastError = DiagnosticResponse::drStatusError;
			      break;
	}
	return m_lastError;
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
	}
	
	return true;
}

// Writes a desired number of bytes from the the pointer
bool ArduinoInterface::deviceWrite(const void* source, const unsigned int numBytes) {
	DWORD written;
	if (m_comPort == INVALID_HANDLE_VALUE) return false;
	return (WriteFile(m_comPort, source, numBytes, &written, NULL)) && (written == numBytes);
}
