/* DrawBridge - aka ArduinoFloppyReader (and writer)
*
* Copyright (C) 2017-2024 Robert Smith (@RobSmithDev)
* https://amiga.robsmithdev.co.uk
*
* This file is multi-licensed under the terms of the Mozilla Public
* License Version 2.0 as published by Mozilla Corporation and the
* GNU General Public License, version 2 or later, as published by the
* Free Software Foundation.
*
* MPL2: https://www.mozilla.org/en-US/MPL/2.0/
* GPL2: https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html
*
*/

#pragma once

#include "afxwin.h"
#include "afxcmn.h"
#include <thread>
#include <string>
#include "resource.h"
#include <functional>
enum class ImageType { itADF = 0, itIBM = 1, itST = 2, itSCP = 3, itIPF = 4 };

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
