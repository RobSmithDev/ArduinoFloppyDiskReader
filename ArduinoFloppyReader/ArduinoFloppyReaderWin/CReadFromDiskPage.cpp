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
#include "CReadFromDiskPage.h"
#include "afxdialogex.h"
#include "resource.h"


// CReadFromDiskPage dialog

IMPLEMENT_DYNAMIC(CReadFromDiskPage, CDialog)

CReadFromDiskPage::CReadFromDiskPage(std::function<void()> onStart, CWnd* pParent /*=nullptr*/)
	: CDialog(IDD_PANEL_READ_DISK, pParent), m_onStart(onStart)
{
	Create(IDD_PANEL_READ_DISK, pParent);
}

CReadFromDiskPage::~CReadFromDiskPage()
{
}

void CReadFromDiskPage::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_DISKFORMAT, m_fileformat);
	DDX_Control(pDX, IDC_TRK80, m_tracks80);
	DDX_Control(pDX, IDC_TRK82, m_tracks82);
	DDX_Control(pDX, IDC_TRK83, m_tracks84);
	DDX_Control(pDX, IDC_FILENAME, m_filename);
	DDX_Control(pDX, IDC_SCPDD, m_scpdd);
	DDX_Control(pDX, IDC_SCPHD, m_scphd);
	DDX_Control(pDX, IDC_SCPHD2, m_scp2);
	DDX_Control(pDX, IDC_EXPERIMENTAL, m_experimental);
}


BEGIN_MESSAGE_MAP(CReadFromDiskPage, CDialog)
	ON_BN_CLICKED(IDC_BROWSE, &CReadFromDiskPage::OnBnClickedBrowse)
	ON_BN_CLICKED(IDC_STARTSTOP, &CReadFromDiskPage::OnBnClickedStartstop)
	ON_CBN_SELCHANGE(IDC_DISKFORMAT, &CReadFromDiskPage::OnCbnSelchangeDiskformat)
END_MESSAGE_MAP()

BOOL CReadFromDiskPage::OnInitDialog() {
	CDialog::OnInitDialog();

	WCHAR buf[256];
	buf[0] = '\0';
	LONG len = 256;

	m_fileformat.AddString(L"ADF (Amiga)");
	m_fileformat.AddString(L"IMA (IBM PC)");
	m_fileformat.AddString(L"ST (Atari ST)");
	m_fileformat.AddString(L"SCP (Any/Flux)");
	buf[0] = '\0';
	len = 10;
	RegQueryValueW(HKEY_CURRENT_USER, L"Software\\ArduinoFloppyReader\\fileFormat2", buf, &len);
	int index = wcstol(buf, nullptr, 10);
	m_fileformat.SetCurSel(index);

	buf[0] = '\0';
	len = 10;
	RegQueryValueW(HKEY_CURRENT_USER, L"Software\\ArduinoFloppyReader\\tracks", buf, &len);
	int numTracks = wcstol(buf, nullptr, 10);
	m_tracks84.SetCheck(numTracks == 84);
	m_tracks82.SetCheck(numTracks == 82);
	m_tracks80.SetCheck(numTracks < 82);

	buf[0] = '\0';
	len = 10;
	RegQueryValueW(HKEY_CURRENT_USER, L"Software\\ArduinoFloppyReader\\highflux", buf, &len);
	index = wcstol(buf, nullptr, 10);
	m_experimental.SetCheck(index != 0);

	OnCbnSelchangeDiskformat();

	return TRUE;
}

void CReadFromDiskPage::saveSettings() {
	char buffer[20];

	_itoa_s(m_fileformat.GetCurSel(), buffer, 10);
	RegSetValueA(HKEY_CURRENT_USER, "Software\\ArduinoFloppyReader\\fileFormat2", REG_SZ, buffer, strlen(buffer));

	_itoa_s(m_tracks84.GetCheck() ? 84 : (m_tracks82.GetCheck() ? 82 : 80), buffer, 10);
	RegSetValueA(HKEY_CURRENT_USER, "Software\\ArduinoFloppyReader\\tracks", REG_SZ, buffer, strlen(buffer));

	_itoa_s(m_experimental.GetCheck() ? 1 : 0, buffer, 10);
	RegSetValueA(HKEY_CURRENT_USER, "Software\\ArduinoFloppyReader\\highflux", REG_SZ, buffer, strlen(buffer));
}

// CReadFromDiskPage message handlers


void CReadFromDiskPage::OnBnClickedBrowse() {
	TCHAR* szFilters;
	TCHAR* szDefault;
	TCHAR* szTitle;
	
	switch (getImageType()) {
	case ImageType::itADF: szFilters = _T("Amiga Disk Files (*.adf)|*.adf|All Files (*.*)|*.*||"); szDefault = _T("adf"); szTitle = _T("Save Disk to ADF File"); break;
	case ImageType::itIBM: szFilters = _T("IBM PC Disk Files (*.ima)|*.ima|All Files (*.*)|*.*||"); szDefault = _T("ima"); szTitle = _T("Save Disk to IMA File");  break;
	case ImageType::itST: szFilters = _T("Atari ST Disk Files (*.st)|*.st|All Files (*.*)|*.*||"); szDefault = _T("st"); szTitle = _T("Save Disk to ST File");  break;
	case ImageType::itSCP: szFilters = _T("SCP Flux Files (*.scp)|*.scp|All Files (*.*)|*.*||"); szDefault = _T("scp"); szTitle = _T("Save Disk to SCP File");  break;
	default: return;
	}

	// Create an Save dialog
	CString oldFileName;
	m_filename.GetWindowText(oldFileName);

	CFileDialog fileDlg(FALSE, szDefault, oldFileName, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST | OFN_NODEREFERENCELINKS | OFN_ENABLESIZING | OFN_DONTADDTORECENT | OFN_EXPLORER, szFilters, this);
	fileDlg.m_ofn.lpstrTitle = szTitle;

	// Display it
	if (fileDlg.DoModal() == IDOK)
		m_filename.SetWindowText(fileDlg.GetPathName());
}


void CReadFromDiskPage::OnBnClickedStartstop()
{
	if (getFilename().length() < 1) {
		MessageBox(L"You need to specify an filename first", L"Error", MB_OK | MB_ICONEXCLAMATION);
		return;
	}

	if (m_onStart) m_onStart();
}


const std::wstring CReadFromDiskPage::getFilename() const {
	CString filename;
	m_filename.GetWindowText(filename);
	return filename.GetBuffer();
}


void CReadFromDiskPage::OnCbnSelchangeDiskformat()
{
	// Correct the file extension
	CString filename;

	m_filename.GetWindowText(filename);

	int pos = filename.ReverseFind(_T('.'));
	if (pos > 0) {
		filename = filename.Left(pos);

		switch (getImageType()) {
		case ImageType::itSCP: filename += _T(".scp"); break;
		case ImageType::itADF: filename += _T(".adf"); break;
		case ImageType::itST: filename += _T(".st"); break;
		case ImageType::itIBM: filename += _T(".ima"); break;
		}

		m_filename.SetWindowText(filename);
	}

	if (getImageType() == ImageType::itSCP) {
		m_scpdd.ShowWindow(SW_SHOW);
		m_scphd.ShowWindow(SW_SHOW);
		m_scp2.ShowWindow(SW_SHOW);
		m_experimental.ShowWindow(SW_SHOW);
		
	}
	else {
		m_scpdd.ShowWindow(SW_HIDE);
		m_scphd.ShowWindow(SW_HIDE);
		m_experimental.ShowWindow(SW_HIDE);
		m_scp2.ShowWindow(SW_HIDE);
	}
}
