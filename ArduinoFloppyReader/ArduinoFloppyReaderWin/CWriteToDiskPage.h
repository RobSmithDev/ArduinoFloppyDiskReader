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
	const bool isSCPFile() const;
	const bool isIPFFile() const;
	afx_msg void OnEnChangeFilename();
};
