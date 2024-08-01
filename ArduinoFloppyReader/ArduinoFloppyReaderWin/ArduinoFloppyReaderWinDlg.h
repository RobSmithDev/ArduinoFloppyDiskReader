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

//////////////////////////////////////////////////////////////////////////////////////////
// Simple Windows application using the libraries used in the console app               //
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once
#include "afxwin.h"
#include "afxcmn.h"
#include <thread>
#include <string>
#include "CReadFromDiskPage.h"
#include "CWriteToDiskPage.h"
#include "CCompleteDialog.h"
#include "CRunningDlg.h"
#include <vector>

// The resource used to play a 'complete' sfx if setup
// #define COMPLETE_SOUNDEFFECT

// CArduinoFloppyReaderWinDlg dialog
class CArduinoFloppyReaderWinDlg : public CDialogEx
{
// Construction
public:
	CArduinoFloppyReaderWinDlg(CWnd* pParent = NULL);	// standard constructor
	~CArduinoFloppyReaderWinDlg(); 
#ifdef COMPLETE_SOUNDEFFECT
	void* m_sfx;
#endif
	unsigned int m_updateStatus = 0;

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
	HCURSOR m_crHourGlass;
	HCURSOR m_crHandPointer;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

private:
	std::vector< CWnd*> m_pages;

	void addTab(CWnd* page, std::wstring name);
public:
	CReadFromDiskPage* m_readPage;
	CWriteToDiskPage* m_writePage;

	CComboBox m_comport;
	CButton m_diagnostics;
	std::thread* m_updateCheck = nullptr;

	// Main thread loop
	void showCompletedDialog(const CCompleteDialog::CompleteMessage message);
	void runThreadRead(CRunningDlg* dlg);
	void runThreadWrite(CRunningDlg* dlg);
	void WriteToDisk();
	void ReadFromDisk();
	void saveComPort();
	void findWhatIsUsingThePort();

	// Returns the COM port if its valid and displays an error if not
	std::wstring checkForComPort();
	afx_msg void OnBnClickedStartstop3();
protected:
	afx_msg LRESULT OnDevicechange(WPARAM wParam, LPARAM lParam);
public:

	CComboBox m_mediadensity;
	afx_msg void OnBnClickedStartstop4();
	CButton m_btnConfig;
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	CTabCtrl m_mode;
	afx_msg void OnSelchangeTab(NMHDR* pNMHDR, LRESULT* pResult);
	afx_msg void OnSelchangingTab(NMHDR* pNMHDR, LRESULT* pResult);
	afx_msg void OnClose();
	afx_msg void OnClickedFooter();
	CStatic m_footer;
	afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
	afx_msg BOOL OnSetCursor(CWnd* pWnd, UINT nHitTest, UINT message);
	afx_msg void OnBnClickedCleaning();
};
