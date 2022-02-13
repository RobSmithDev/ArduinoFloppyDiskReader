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


#include "stdafx.h"
#include "ArduinoFloppyReaderWin.h"
#include "CWriteToDiskPage.h"
#include "afxdialogex.h"
#include "resource.h"


// CWriteToDiskPage dialog

//IMPLEMENT_DYNAMIC(CWriteToDiskPage, CDialog)

CWriteToDiskPage::CWriteToDiskPage(std::function<void()> onStart, CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_PANEL_WRITE_DISK), m_onStart(onStart)
{
	Create(IDD_PANEL_WRITE_DISK, pParent);
}

CWriteToDiskPage::~CWriteToDiskPage()
{
}

void CWriteToDiskPage::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_FILENAME2, m_filename);
	DDX_Control(pDX, IDC_CHECK1, m_verify);
	DDX_Control(pDX, IDC_CHECK3, m_precomp);
	DDX_Control(pDX, IDC_CHECK7, m_erase);
	DDX_Control(pDX, IDC_CHECK8, m_index);
}


void CWriteToDiskPage::saveSettings() {
	char buffer[20];

	_itoa_s(m_verify.GetCheck() ? 1 : 0, buffer, 10);
	RegSetValueA(HKEY_CURRENT_USER, "Software\\ArduinoFloppyReader\\verify", REG_SZ, buffer, strlen(buffer));

	_itoa_s(m_precomp.GetCheck() ? 1 : 0, buffer, 10);
	RegSetValueA(HKEY_CURRENT_USER, "Software\\ArduinoFloppyReader\\precomp", REG_SZ, buffer, strlen(buffer));
	
	_itoa_s(m_erase.GetCheck() ? 1 : 0, buffer, 10);
	RegSetValueA(HKEY_CURRENT_USER, "Software\\ArduinoFloppyReader\\erase", REG_SZ, buffer, strlen(buffer));
	
	_itoa_s(m_index.GetCheck() ? 1 : 0, buffer, 10);
	RegSetValueA(HKEY_CURRENT_USER, "Software\\ArduinoFloppyReader\\useindex", REG_SZ, buffer, strlen(buffer));
}

BOOL CWriteToDiskPage::OnInitDialog() {
	CDialog::OnInitDialog();

	WCHAR buf[256];
	buf[0] = '\0';
	LONG len = 256;

	buf[0] = '\0'; len = 10;
	RegQueryValueW(HKEY_CURRENT_USER, L"Software\\ArduinoFloppyReader\\verify", buf, &len);
	int index = ((wcstol(buf, nullptr, 10)!=0) || (wcslen(buf)<1)) ? 1 : 0;
	m_verify.SetCheck(index);

	buf[0] = '\0'; len = 10;
	RegQueryValueW(HKEY_CURRENT_USER, L"Software\\ArduinoFloppyReader\\precomp", buf, &len);
	index = ((wcstol(buf, nullptr, 10) != 0) || (wcslen(buf) < 1)) ? 1 : 0;
	m_precomp.SetCheck(index);

	buf[0] = '\0'; len = 10;
	RegQueryValueW(HKEY_CURRENT_USER, L"Software\\ArduinoFloppyReader\\erase", buf, &len);
	index = (wcstol(buf, nullptr, 10) != 0);  // not default to true
	m_erase.SetCheck(index);

	buf[0] = '\0'; len = 10;
	RegQueryValueW(HKEY_CURRENT_USER, L"Software\\ArduinoFloppyReader\\useindex", buf, &len);
	index = ((wcstol(buf, nullptr, 10) != 0) || (wcslen(buf) < 1)) ? 1 : 0;
	m_index.SetCheck(index);

	OnEnChangeFilename();

	return TRUE;
}

BEGIN_MESSAGE_MAP(CWriteToDiskPage, CDialogEx)
	ON_BN_CLICKED(IDC_BROWSE2, &CWriteToDiskPage::OnBnClickedBrowse2)
	ON_BN_CLICKED(IDC_STARTSTOP2, &CWriteToDiskPage::OnBnClickedStartstop2)
	ON_EN_CHANGE(IDC_FILENAME2, &CWriteToDiskPage::OnEnChangeFilename)
END_MESSAGE_MAP()


// CWriteToDiskPage message handlers

const std::wstring CWriteToDiskPage::getFilename() const {
	CString filename;
	m_filename.GetWindowText(filename);
	return filename.GetBuffer();
}

/*
bool isSCPFile = false;
	*/

void CWriteToDiskPage::OnBnClickedBrowse2()
{
	// szFilters is a text string that includes two file name filters:
	TCHAR szFilters[] = _T("Supported Files (*.adf, *.scp, *.ipf)|*.adf;*.scp;*.ipf|Amiga Disk Files (*.adf)|*.adf|Supercard Pro Files (*.scp)|*.adf|Amiga Disk Files (*.ipf)|*.ipf|All Files (*.*)|*.*||");

	// Create an Save dialog
	CString oldFileName;
	m_filename.GetWindowText(oldFileName);

	CFileDialog fileDlg(TRUE, _T("adf"), oldFileName, OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NODEREFERENCELINKS | OFN_ENABLESIZING | OFN_EXPLORER, szFilters, this);
	fileDlg.m_ofn.lpstrTitle = L"Select a Disk Image File to Write To Disk";

	// Display it
	if (fileDlg.DoModal() == IDOK)
		m_filename.SetWindowText(fileDlg.GetPathName());
}


void CWriteToDiskPage::OnBnClickedStartstop2()
{
	if (getFilename().length() < 1) {
		MessageBox(L"You need to specify an filename first", L"Error", MB_OK | MB_ICONEXCLAMATION);
		return;
	}

	if (m_onStart) m_onStart();
}

const bool CWriteToDiskPage::isIPFFile() const {
	bool scpFIle = false;
	std::wstring fleName = getFilename();
	for (wchar_t& c : fleName) c = toupper(c);
	const wchar_t* extension = wcsrchr(fleName.c_str(), L'.');
	if (extension) {
		extension++;
		scpFIle = wcscmp(extension, L"IPF") == 0;
	}
	return scpFIle;
}

const bool CWriteToDiskPage::isSCPFile() const {
	bool scpFIle = false;
	std::wstring fleName = getFilename();
	for (wchar_t& c : fleName) c = toupper(c);
	const wchar_t* extension = wcsrchr(fleName.c_str(), L'.');
	if (extension) {
		extension++;
		scpFIle = wcscmp(extension, L"SCP") == 0;
	}
	return scpFIle;
}


void CWriteToDiskPage::OnEnChangeFilename()
{
	const bool notADF = isSCPFile() || isIPFFile();
	m_verify.EnableWindow(!notADF);
	m_precomp.EnableWindow(!notADF);
	m_index.EnableWindow(!notADF);
}
