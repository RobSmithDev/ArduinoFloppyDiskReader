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
// Simple Phase Statistical Analysis Class                                            //
////////////////////////////////////////////////////////////////////////////////////////
//
// Purpose:
// The Arduino sketch can adjust a phase charasteristic during its read (see notes)
// The best values for the phase for fastest read change from disk to disk.
// The purpose of this class is to track successful reads for each phase and maintain
// A list of the most efficient phase sequence for reading
//
//
//

#pragma once
#include <vector>
#include <algorithm>

#define NUMBER_OF_PHASES 16

namespace ArduinoFloppyReader {

	// History about success from a chosen phase
	struct PhaseData {
		// Index of this phase character in the initial sequence
		unsigned int phaseIndex;
		// Counter of number of successful sectors read using this phase
		int counter;
	};

	// Class to handle this
	class PhaseAnalyser {
	private:
		// This is a rough guide/starting point
		static const char* m_initialSequence;

		std::vector<ArduinoFloppyReader::PhaseData> m_phaseData;
	public:
		// Initialize the class
		PhaseAnalyser(); 

		// Reset back to the basic original sequence
		void reset();

		// Submit some statistics to the class regarding successful sectors read for this phase.  
		void submitStatistics(const char phase, const int successfulSectorsRead);

		// Causes the class to re-order the phases based on the best results
		void reflectOrder();

		// Retreives the current suggested phase to use at index 0..NUMBER_OF_PHASES-1
		const char getPhaseAtIndex(const unsigned int index) const;
	};

};