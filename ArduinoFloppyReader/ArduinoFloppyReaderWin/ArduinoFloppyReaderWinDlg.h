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
#include <string>
#include "CReadFromDiskPage.h"
#include "CWriteToDiskPage.h"
#include "CCompleteDialog.h"
#include "CRunningDlg.h"
#include <vector>


// CArduinoFloppyReaderWinDlg dialog
class CArduinoFloppyReaderWinDlg : public CDialogEx
{
// Construction
public:
	CArduinoFloppyReaderWinDlg(CWnd* pParent = NULL);	// standard constructor
	~CArduinoFloppyReaderWinDlg(); 
	void* m_sfx;
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
};
