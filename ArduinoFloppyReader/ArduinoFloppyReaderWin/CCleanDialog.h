#pragma once

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

#include "afxwin.h"
#include "afxcmn.h"
#include <thread>
#include <string>
#include "resource.h"
#include <functional>
#include "../lib/ArduinoInterface.h"
#include "../lib/ADFWriter.h"

// CCleanDialog dialog

class CCleanDialog : public CDialogEx
{
	DECLARE_DYNAMIC(CCleanDialog)

private:
	std::thread* m_iothread = nullptr;
	bool m_abortPressed = false;
	bool m_stateNeedsToChange = false;
	int m_maxValue = 100;
	const std::wstring m_comPort;

	CComPtr<ITaskbarList3> m_spTaskbarList;
public:
	CCleanDialog(const std::wstring& m_comPort);
	virtual ~CCleanDialog();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_CLEAN };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()

public:
	virtual BOOL OnInitDialog();
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);

public:
	bool wasAbortPressed() const { return m_abortPressed; };
	void resetProgress(int max);
	void setProgress(int position);
protected:
	afx_msg LRESULT OnUser(WPARAM wParam, LPARAM lParam);
public:
	
	afx_msg void OnBnClickedClose();
	afx_msg void OnStnClickedMakeyourown();
	afx_msg BOOL OnSetCursor(CWnd* pWnd, UINT nHitTest, UINT message);
	CStatic m_makeYourOwn;
	afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
	afx_msg void OnBnClickedStart();
	CButton m_startButton;
	CProgressCtrl m_progress;
};
