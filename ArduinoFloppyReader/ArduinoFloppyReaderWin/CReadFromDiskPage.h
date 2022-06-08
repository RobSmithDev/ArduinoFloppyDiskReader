/* ArduinoFloppyReaderWin
*
* Copyright (C) 2017-2022 Robert Smith (@RobSmithDev)
* https://amiga.robsmithdev.co.uk
*
* This program is free software; you can redistribute it and/or
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
* License along with this program; if not see http://www.gnu.org/licenses/
*/

#pragma once

#include "afxwin.h"
#include "afxcmn.h"
#include <thread>
#include <string>
#include "resource.h"
#include <functional>


// CReadFromDiskPage dialog

class CReadFromDiskPage : public CDialog
{
	DECLARE_DYNAMIC(CReadFromDiskPage)
private:
	std::function<void()> m_onStart;

public:

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_PANEL_READ_DISK };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	virtual BOOL OnInitDialog();

	DECLARE_MESSAGE_MAP()
	CComboBox m_fileformat;
	CButton m_tracks80;
	CButton m_tracks82;
	CButton m_tracks84;
	CEdit m_filename;
	CButton m_scpdd;
	CButton m_scphd;
	CStatic m_scp2;
	CButton m_experimental;

	afx_msg void OnBnClickedBrowse();
	afx_msg void OnBnClickedStartstop();
	afx_msg void OnCbnSelchangeDiskformat();
public:
	enum class ImageType { itADF = 0, itSCP = 1 };
	enum class NumTracks { nt80, nt82, nt84 };
	CReadFromDiskPage(std::function<void()> onStart, CWnd* pParent = nullptr);   // standard constructor
	virtual ~CReadFromDiskPage();

	// Save to registry
	void saveSettings();

	// Properties set on this dialog
	const std::wstring getFilename() const;
	const ImageType getImageType() const { return static_cast<ImageType>(m_fileformat.GetCurSel()); }
	const NumTracks getNumTracks() const { return (m_tracks84.GetCheck() != 0) ? NumTracks::nt84 : ((m_tracks82.GetCheck() != 0) ? NumTracks::nt82 : NumTracks::nt80 ); };
	const bool isExperimentalMode() const { return m_experimental.GetCheck() != 0; };
};
