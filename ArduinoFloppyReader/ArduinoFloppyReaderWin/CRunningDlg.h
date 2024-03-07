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

// CRunningDlg dialog

class CRunningDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CRunningDlg)

private:
	std::thread* m_iothread = nullptr;
	const std::wstring m_title;
	bool m_threadRunning = false;
	bool m_abortPressed = false;
	ULONGLONG m_maxValue;
	bool m_stateNeedsToChange = false;

	CComPtr<ITaskbarList3> m_spTaskbarList;

	std::function<void(CRunningDlg* dlg)> m_onRun;
public:
	CRunningDlg(const std::wstring windowTitle, std::function<void(CRunningDlg* dlg)> onRun, CWnd* pParent = nullptr);   // standard constructor
	virtual ~CRunningDlg();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_RUNNING };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
	virtual BOOL OnInitDialog();
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnBnClickedCancel();

	
	CStatic m_cylinder;
	CStatic m_side;
	CStatic m_good;
	CStatic m_partial;
	CStatic m_message;
	CButton m_cancelbtn;
	CProgressCtrl m_progress;

public:
	void setCylinder(int cylinder) { m_cylinder.SetWindowTextW(std::to_wstring(cylinder).c_str()); };
	void setSide(ArduinoFloppyReader::DiskSurface surface) { m_side.SetWindowTextW(surface == ArduinoFloppyReader::DiskSurface::dsLower ? L"Lower" : L"Upper"); };
	void setGoodSectors(int amount, int total) { m_good.SetWindowTextW( std::wstring(std::to_wstring(amount) + L"/" + std::to_wstring(total)).c_str()); };
	void setPartialSectors(int total) { m_partial.SetWindowTextW(std::to_wstring(total).c_str()); };
	void setStatus(const std::wstring& message) { m_message.SetWindowTextW(message.c_str()); };
	void resetProgress(int max);
	void setOperation(ArduinoFloppyReader::CallbackOperation op);
	void setProgress(int position);
	bool wasAbortPressed() const { return m_abortPressed; };
protected:
	afx_msg LRESULT OnUser(WPARAM wParam, LPARAM lParam);
public:
	CStatic m_operation;
};
