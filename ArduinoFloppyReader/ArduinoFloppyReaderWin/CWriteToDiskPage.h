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
#include "CReadFromDiskPage.h"

// CWriteToDiskPage dialog

class CWriteToDiskPage : public CDialogEx {
private:
	std::function<void()> m_onStart;
public:
// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_PANEL_WRITE_DISK };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	virtual BOOL OnInitDialog();

	DECLARE_MESSAGE_MAP()

	CEdit m_filename;
	CButton m_verify;
	CButton m_precomp;
	CButton m_erase;
	CButton m_index;

	afx_msg void OnBnClickedBrowse2();
	afx_msg void OnBnClickedStartstop2();
public:
	CWriteToDiskPage(std::function<void()> onStart, CWnd* pParent = nullptr);   // standard constructor
	virtual ~CWriteToDiskPage();

	// Save current config to the regstry
	void saveSettings();

	// Properties set on this dialog
	const std::wstring getFilename() const;
	const bool isVerify() const { return m_verify.GetCheck() != 0; };
	const bool isPrecomp() const { return m_precomp.GetCheck() != 0; };
	const bool isErase() const { return m_erase.GetCheck() != 0; };
	const bool isIndex() const { return m_index.GetCheck() != 0; };
	const ImageType fileType() const;
	afx_msg void OnEnChangeFilename();
};
