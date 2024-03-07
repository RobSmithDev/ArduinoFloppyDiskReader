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
#include "../lib/ArduinoInterface.h"
#include "../lib/ADFWriter.h"

// CDiagnosticsDialog dialog used diagnostics status window
class CDiagnosticsDialog : public CDialogEx
{
public:
	CDiagnosticsDialog(const std::wstring& comport) : CDialogEx(IDD_DIAGNOSTICS), m_comPort(comport), m_mainThread(nullptr) {};
	virtual ~CDiagnosticsDialog() {
		if (m_mainThread) {
			if (m_mainThread->joinable()) m_mainThread->join();
			delete m_mainThread;
		}
	}

public:
	CEdit m_results;

	// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_DIAGNOSTICS };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX) {
		CDialogEx::DoDataExchange(pDX);
		DDX_Control(pDX, IDC_EDIT1, m_results);
	};


	BOOL OnInitDialog()
	{
		CDialogEx::OnInitDialog();
		m_threadRunning = true;

		WCHAR buffer[200];
		swprintf_s(buffer, L"Running Diagnostics on %s", m_comPort.c_str());
		SetWindowText(buffer);

		CMenu* mnu = this->GetSystemMenu(FALSE);
		mnu->EnableMenuItem(SC_CLOSE, MF_BYCOMMAND | MF_GRAYED | MF_DISABLED);

		// Main processing thread
		m_mainThread = new std::thread([this]()->void {
			runDiagnostics();
			m_threadRunning = false;
			});

		return true;
	}

protected:
	DECLARE_MESSAGE_MAP()

private:
	const std::wstring m_comPort;
	bool m_threadRunning = true;
	ArduinoFloppyReader::ADFWriter writer;
	std::thread* m_mainThread;

	// This is ran under a thread
	void runDiagnostics() {
		writer.runDiagnostics(m_comPort, [this](bool isError, const std::string message)->void {
			CString strLine;

			if (isError) strLine = "DIAGNOSTICS FAILED: ";
			strLine += message.c_str();
			strLine += "\r\n";

			// get the initial text length
			int nLength = m_results.GetWindowTextLength();
			// put the selection at the end of text
			m_results.SetSel(nLength, nLength);
			// replace the selection
			m_results.ReplaceSel(strLine);

			}, [this](bool isQuestion, const std::string question)->bool {
				if (isQuestion)
					return MessageBoxA(GetSafeHwnd(), (LPCSTR)question.c_str(), "Diagnostics Question", MB_YESNO | MB_ICONQUESTION) == IDYES;
				else
					return MessageBoxA(GetSafeHwnd(), (LPCSTR)question.c_str(), "Diagnostics Prompt", MB_OKCANCEL | MB_ICONINFORMATION) == IDOK;
			});

		writer.closeDevice();

		CMenu* mnu = this->GetSystemMenu(FALSE);
		mnu->EnableMenuItem(SC_CLOSE, MF_BYCOMMAND | MF_ENABLED);
	}

	// Prevent Enter and ESC closing the dialog
	BOOL PreTranslateMessage(MSG* pMsg)
	{
		if (pMsg->message == WM_KEYDOWN)
		{
			if (pMsg->wParam == VK_RETURN || pMsg->wParam == VK_ESCAPE)
			{
				return !m_threadRunning;                // Do not process further
			}
		}

		return CWnd::PreTranslateMessage(pMsg);
	}

};

BEGIN_MESSAGE_MAP(CDiagnosticsDialog, CDialogEx)
END_MESSAGE_MAP()