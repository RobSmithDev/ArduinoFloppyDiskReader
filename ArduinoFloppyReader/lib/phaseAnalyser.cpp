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

#include "stdafx.h"
#include <vector>
#include <algorithm>
#include "phaseAnalyser.h"

using namespace ArduinoFloppyReader;

// Default phase sequence order in distance from when it should work
const char* PhaseAnalyser::m_initialSequence = "897A6B5C4D3E2F10"; 


// Initialize the class
PhaseAnalyser::PhaseAnalyser() {
	reset();
}

// Reset back to the basic original sequence
void PhaseAnalyser::reset() {
	m_phaseData.clear();
	for (unsigned int phaseIndex = 0; phaseIndex < NUMBER_OF_PHASES; phaseIndex++)
		m_phaseData.push_back({ phaseIndex , 0 });
}

// Submit some statistics to the class regarding successful sectors read for this phase.  
void PhaseAnalyser::submitStatistics(const char phase, const int successfulSectorsRead) {
	// First we need to find what the current position for this phase is
	std::vector<PhaseData>::iterator index = std::find_if(m_phaseData.begin(), m_phaseData.end(), [phase](const PhaseData& data) -> bool {
		return m_initialSequence[data.phaseIndex] == phase;
	});

	if (index == m_phaseData.end()) return;

	m_phaseData[index - m_phaseData.begin()].counter += successfulSectorsRead;
}

// Causes the class to re-order the phases based on the best results
void PhaseAnalyser::reflectOrder() {
	std::sort(m_phaseData.begin(), m_phaseData.end(), [](const PhaseData& a, const PhaseData& b) -> bool {
		if (a.counter == b.counter) 
			return a.phaseIndex < b.phaseIndex;
		else 
			return a.counter > b.counter;
	});
}

// Retreives the current suggested phase to use at index 0..NUMBER_OF_PHASES-1
const char PhaseAnalyser::getPhaseAtIndex(const unsigned int index) const {
	if (index >= m_phaseData.size()) 
		return m_initialSequence[m_phaseData[0].phaseIndex];
	else 
		return m_initialSequence[m_phaseData[index].phaseIndex];
}
