/* ArduinoFloppyReaderWin
*
* Copyright (C) 2017-2018 Robert Smith (@RobSmithDev)
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
	~CArduinoFloppyReaderWinDlg(); 
	std::thread* m_iothread;
	void* m_sfx;
	bool m_cancelButtonPressed;
	bool m_partial;

	void enumComPorts();
	std::wstring getComPort();
	void setComPort(const std::wstring& comport);

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
	void enableDialog(bool enable);

	afx_msg LRESULT OnUserMessage(WPARAM wparam, LPARAM lparam);

	CComboBox m_comport;
	CEdit m_outputADF;
	CProgressCtrl m_progressbar;
	CStatic m_statusTrack;
	CStatic m_statusSide;
	CStatic m_statusGood;
	CStatic m_statusPartial;
	CButton m_browseButton;
	CButton m_copyButton;
	CStatic m_statusText;
	CEdit m_inputADF;
	CButton m_browseButton2;
	CButton m_writeButton;
	CButton m_verify;
	CButton m_trk80;
	CButton m_trk82;
	CButton m_precomp;

	// Main thread loop
	bool runThreadRead();
	void threadFinishedReading(bool successful);
	bool runThreadWrite();
	void threadFinishedWriting(bool successful);
	afx_msg void OnBnClickedBrowse2();
	afx_msg void OnBnClickedStartstop2();
	void saveComPort();
	afx_msg void OnBnClickedStartstop3();
	CButton m_diagnostics;
//	CButton m_precomp;
	CComboBox m_fileFormat;
protected:
	afx_msg LRESULT OnDevicechange(WPARAM wParam, LPARAM lParam);
public:

};
