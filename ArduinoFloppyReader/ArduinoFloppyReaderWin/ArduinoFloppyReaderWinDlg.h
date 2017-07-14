/* ArduinoFloppyReaderWin
*
* Copyright (C) 2017 Robert Smith (@RobSmithDev)
* http://amiga.robsmithdev.co.uk
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

//////////////////////////////////////////////////////////////////////////////////////////
// Simple Windows application using the libraries used in the console app               //
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once
#include "afxwin.h"
#include "afxcmn.h"
#include <thread>


// CArduinoFloppyReaderWinDlg dialog
class CArduinoFloppyReaderWinDlg : public CDialogEx
{
// Construction
public:
	CArduinoFloppyReaderWinDlg(CWnd* pParent = NULL);	// standard constructor
	std::thread* m_reader;
	void* m_sfx;
	bool m_cancelButtonPressed;
	bool m_partial;

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ARDUINOFLOPPYREADERWIN_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedStartstop();
	afx_msg void OnBnClickedBrowse();
	afx_msg LRESULT OnUserMessage(WPARAM wparam, LPARAM lparam);
	// Com Port Edit Box
	CEdit m_comport;
	// Filename box
	CEdit m_filename;
	// Progress bar
	CProgressCtrl m_progressbar;
	CStatic m_statusTrack;
	CStatic m_statusSide;
	CStatic m_statusGood;
	CStatic m_statusPartial;
	CButton m_browseButton;
	CButton m_copyButton;

	// Main thread loop
	bool runThread();
	void threadFinished(bool successful);
	CStatic m_statusText;
	CSpinButtonCtrl m_spinner;
};
