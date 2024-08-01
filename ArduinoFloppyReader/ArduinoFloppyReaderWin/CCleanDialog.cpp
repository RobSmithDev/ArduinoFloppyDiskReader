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

#include "stdafx.h"
#include "ArduinoFloppyReaderWin.h"
#include "CCleanDialog.h"
#include "afxdialogex.h"



// CCleanDialog dialog

IMPLEMENT_DYNAMIC(CCleanDialog, CDialogEx)

CCleanDialog::CCleanDialog(const std::wstring& comPort) : m_comPort(comPort), CDialogEx(IDD_CLEAN) {
}

CCleanDialog::~CCleanDialog()
{
	if (m_iothread) {
		if (m_iothread->joinable()) m_iothread->join();
		delete m_iothread;
		m_iothread = nullptr;
	}
}

void CCleanDialog::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_MAKEYOUROWN, m_makeYourOwn);
	DDX_Control(pDX, ID_START, m_startButton);
	DDX_Control(pDX, 1007, m_progress);
}


BEGIN_MESSAGE_MAP(CCleanDialog, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_MESSAGE(WM_USER, &CCleanDialog::OnUser)
	ON_BN_CLICKED(ID_CLOSE, &CCleanDialog::OnBnClickedClose)
	ON_STN_CLICKED(IDC_MAKEYOUROWN, &CCleanDialog::OnStnClickedMakeyourown)
	ON_WM_SETCURSOR()
	ON_WM_CTLCOLOR()
	ON_BN_CLICKED(ID_START, &CCleanDialog::OnBnClickedStart)
END_MESSAGE_MAP()

void CCleanDialog::resetProgress(int max) { 
	if (m_spTaskbarList) m_spTaskbarList->SetProgressState(GetSafeHwnd(), TBPF_INDETERMINATE);
	m_maxValue = max;
	m_stateNeedsToChange = true;
};

void CCleanDialog::setProgress(int position) { 
	m_progress.SetPos(position);
	if (m_stateNeedsToChange) {
		m_stateNeedsToChange = false;
		if (m_spTaskbarList) m_spTaskbarList->SetProgressState(GetSafeHwnd(), TBPF_NORMAL);
	}
	if (m_spTaskbarList) m_spTaskbarList->SetProgressValue(GetSafeHwnd(), position, m_maxValue);	
};

// CCleanDialog message handlers
BOOL CCleanDialog::OnInitDialog()
{
	BOOL ret = CDialogEx::OnInitDialog();

	HRESULT hr = ::CoCreateInstance(CLSID_TaskbarList, NULL, CLSCTX_INPROC_SERVER, __uuidof(ITaskbarList3), reinterpret_cast<void**>(&m_spTaskbarList));
	if (SUCCEEDED(hr)) m_spTaskbarList->HrInit(); else m_spTaskbarList = nullptr;

	resetProgress(100);

	return ret;  
}


BOOL CCleanDialog::PreTranslateMessage(MSG* pMsg)
{
	if (pMsg->message == WM_KEYDOWN)
	{
		if (pMsg->wParam == VK_RETURN || pMsg->wParam == VK_ESCAPE)
		{
			return !m_iothread;                // Do not process further
		}
	}

	if (pMsg->message == WM_CLOSE) {
		if (m_iothread) return 1;
	}

	return CWnd::PreTranslateMessage(pMsg);
}


void CCleanDialog::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == SC_CLOSE)
	{
		if (m_iothread) return;
	}

	CDialogEx::OnSysCommand(nID, lParam);
}

afx_msg LRESULT CCleanDialog::OnUser(WPARAM wParam, LPARAM lParam)
{
	if (wParam == 100) {
		if (lParam == 2) MessageBox(_T("No disk was detected in the drive. Clean aborted."), _T("Sorry"), MB_OK | MB_ICONEXCLAMATION); else
			if (lParam == 1) MessageBox(_T("Unable to communicate with the COM port.  Please check port or run diagnostics."), _T("Sorry"), MB_OK | MB_ICONEXCLAMATION); else
			if (m_abortPressed) CDialogEx::OnCancel(); else MessageBox(L"Cleaning cycle completed.", L"Drive Head Cleaner", MB_OK | MB_ICONINFORMATION);
		if (m_iothread) {
			if (m_iothread->joinable()) m_iothread->join();
			delete m_iothread;
			m_iothread = nullptr;
		}

		m_startButton.EnableWindow(true);
	}
	return 0;
}


void CCleanDialog::OnBnClickedClose() {
	if (m_iothread) m_abortPressed = true; else CDialogEx::OnCancel();
}


void CCleanDialog::OnStnClickedMakeyourown()
{
	ShellExecute(GetSafeHwnd(), L"OPEN", L"https://youtu.be/7E4fSypg0pk", NULL, NULL, SW_SHOW);
}


BOOL CCleanDialog::OnSetCursor(CWnd* pWnd, UINT nHitTest, UINT message)
{
	if (pWnd->GetSafeHwnd() == m_makeYourOwn.GetSafeHwnd()) {
		SetCursor(LoadCursor(NULL, IDC_HAND));
		return TRUE;
	}
	return CDialogEx::OnSetCursor(pWnd, nHitTest, message);
}


HBRUSH CCleanDialog::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor)
{
	HBRUSH hbr = CDialogEx::OnCtlColor(pDC, pWnd, nCtlColor);

	if (nCtlColor == CTLCOLOR_STATIC && pWnd->GetSafeHwnd() == m_makeYourOwn.GetSafeHwnd()) pDC->SetTextColor(GetSysColor(COLOR_HOTLIGHT)); 
	return hbr;
}

void CCleanDialog::OnBnClickedStart()
{
	if (m_iothread) return;
	m_startButton.EnableWindow(false);

	if (m_iothread) {
		if (m_iothread->joinable()) m_iothread->join();
		delete m_iothread;
		m_iothread = nullptr;
	}
	
	m_iothread = new std::thread([this]() {
		
		resetProgress(1000);
		setProgress(0);

		ArduinoFloppyReader::ADFWriter reader;

		if (reader.openDevice(m_comPort)) {
			bool noDisk = reader.runClean([this](const uint16_t position, const uint16_t maxPosition) -> bool {
				if (position == 0) resetProgress(maxPosition);
				setProgress(position);
				return !m_abortPressed;
			}) == ArduinoFloppyReader::ADFResult::adfrCompletedWithErrors;
			reader.closeDevice();
			PostMessage(WM_USER, 100, noDisk?2:0);
		} else PostMessage(WM_USER, 100, 1);
	});

}
