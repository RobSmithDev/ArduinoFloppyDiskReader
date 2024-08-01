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

#include "stdafx.h"
#include <afxwin.h>
#include "ArduinoFloppyReaderWin.h"
#include "ArduinoFloppyReaderWinDlg.h"
#include "CDiagnosticsDlg.h"
#include "CAboutDialog.h"
#include "CEEpromDialog.h"
#include "CCompleteDialog.h"
#include "afxdialogex.h"
#include <mmsystem.h>
#include "..\lib\ADFWriter.h"
#include <Windows.h>
#include "CRunningDlg.h"
#include "CCompleteDialog.h"
#include <WinSock2.h>
#include <wtsapi32.h>
#include <WinDNS.h>
#include "CCleanDialog.h"
#pragma comment(lib, "Dnsapi.lib")

#define EVENT_ID_RESCAN			((UINT_PTR)100)
#define EVENT_ID_BLINK			((UINT_PTR)101)
#define EVENT_RESCAN_INTERVAL	100



static const WCHAR* footerStringNormal = L"DrawBridge %i.%i.%i, Created by Robert Smith (RobSmithDev) https://amiga.robsmithdev.co.uk";
static const WCHAR* footerStringUpdate = L"DrawBridge %i.%i.%i, Update Available (V%i.%i.%i) at https://amiga.robsmithdev.co.uk";

#pragma comment(lib,"Wtsapi32.lib")

// CArduinoFloppyReaderWinDlg dialog
CArduinoFloppyReaderWinDlg::CArduinoFloppyReaderWinDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_ARDUINOFLOPPYREADERWIN_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_crHourGlass = (HCURSOR)LoadCursor(NULL, IDC_WAIT);
	m_crHandPointer = (HCURSOR)LoadCursor(NULL, IDC_HAND);
}

// DDX
void CArduinoFloppyReaderWinDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMPORT, m_comport);
	DDX_Control(pDX, IDC_STARTSTOP3, m_diagnostics);
	DDX_Control(pDX, IDC_MEDIADENSITY, m_mediadensity);
	DDX_Control(pDX, IDC_STARTSTOP4, m_btnConfig);
	DDX_Control(pDX, IDC_TAB1, m_mode);
	DDX_Control(pDX, IDC_FOOTER, m_footer);
}

BEGIN_MESSAGE_MAP(CArduinoFloppyReaderWinDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_STARTSTOP3, &CArduinoFloppyReaderWinDlg::OnBnClickedStartstop3)
	ON_MESSAGE(WM_DEVICECHANGE, &CArduinoFloppyReaderWinDlg::OnDevicechange)
	ON_BN_CLICKED(IDC_STARTSTOP4, &CArduinoFloppyReaderWinDlg::OnBnClickedStartstop4)
	ON_WM_TIMER()
	ON_NOTIFY(TCN_SELCHANGE, IDC_TAB1, &CArduinoFloppyReaderWinDlg::OnSelchangeTab)
	ON_NOTIFY(TCN_SELCHANGING, IDC_TAB1, &CArduinoFloppyReaderWinDlg::OnSelchangingTab)
	ON_WM_CLOSE()
	ON_STN_CLICKED(IDC_FOOTER, &CArduinoFloppyReaderWinDlg::OnClickedFooter)
	ON_WM_CTLCOLOR()
	ON_WM_SETCURSOR()
	ON_BN_CLICKED(IDC_CLEANING, &CArduinoFloppyReaderWinDlg::OnBnClickedCleaning)
END_MESSAGE_MAP()


// Add a tab to the control
void CArduinoFloppyReaderWinDlg::addTab(CWnd* page, std::wstring name) {
	m_mode.InsertItem((int)m_pages.size(), (LPWSTR)name.c_str());
	m_pages.push_back(page);

	RECT rec, rec2;
	page->GetClientRect(&rec);
	page->GetClientRect(&rec2);
	m_mode.AdjustRect(FALSE, &rec);
	page->SetWindowPos(NULL, rec.left, rec.top, rec.right - rec.left, rec2.bottom - rec2.top, SWP_NOACTIVATE | SWP_NOZORDER);

	page->ShowWindow(m_pages.size() == 1);

	EnableThemeDialogTexture(page->GetSafeHwnd(), ETDT_ENABLETAB);
}

// Re-populate the com port list 
void CArduinoFloppyReaderWinDlg::enumComPorts() {
	std::vector<std::wstring> portList;
	ArduinoFloppyReader::ArduinoInterface::enumeratePorts(portList);

	bool ftdiNeedReRead = false;
	m_comport.ResetContent();
	for (const std::wstring& port : portList) {
		m_comport.AddString(port.c_str());
		if (port.find(L"FTDI: ()") != std::wstring::npos) ftdiNeedReRead = true;
	}

	if (ftdiNeedReRead) {
		OnDevicechange(0, 0);
	}
}

// Messy startup code
BOOL CArduinoFloppyReaderWinDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	

	AfxOleInit();
	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		pSysMenu->AppendMenu(MF_SEPARATOR);
		pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, _T("About DrawBridge"));
	}

	TCITEMW tab;
	memset(&tab, 0, sizeof(tab));

	m_readPage = new CReadFromDiskPage([this]() {
		ReadFromDisk();
		}, &m_mode);
	m_writePage = new CWriteToDiskPage([this]() {
		WriteToDisk();
		}, &m_mode);

	addTab(m_readPage, L"Read from Disk");
	addTab(m_writePage, L"Write to Disk");
	
	// Set the icons for this window
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	enumComPorts();

	WCHAR buffer[256];
	unsigned int major, minor, rev;
	CAboutDlg::GetProductVersion(major, minor, rev);
	swprintf_s(buffer, footerStringNormal, major, minor, rev);
	m_footer.SetWindowText(buffer);

	WCHAR buf[256];
	buf[0] = '\0';
	LONG len = 256;
	memset(buf, 0, len * 2);
	RegQueryValueW(HKEY_CURRENT_USER, L"Software\\ArduinoFloppyReader\\ComPort", buf, &len);
	int port = wcstol(buf, nullptr, 10);
	if (port) swprintf_s(buf, L"COM%i", port);
	setComPort(buf);

	m_mediadensity.AddString(L"Auto");
	m_mediadensity.AddString(L"DD (MUST Cover Hole for HD Disks)");
	m_mediadensity.AddString(L"HD");
	int w = SendDlgItemMessageW(IDC_MEDIADENSITY, CB_GETDROPPEDWIDTH, 0, 0);
	SendDlgItemMessageW(IDC_MEDIADENSITY, CB_SETDROPPEDWIDTH, w + 140, 0);
	buf[0] = '\0';
	len = 10;
	RegQueryValueW(HKEY_CURRENT_USER, L"Software\\ArduinoFloppyReader\\disksize", buf, &len);
	int index = ((buf) && (wcslen(buf))) ? wcstol(buf, nullptr, 10) : 1;
	m_mediadensity.SetCurSel(index);

	buf[0] = '\0';
	len = 10;
	RegQueryValueW(HKEY_CURRENT_USER, L"Software\\ArduinoFloppyReader\\currentmode", buf, &len);
	index = ((buf) && (wcslen(buf))) ? wcstol(buf, nullptr, 10) : 0;
	m_mode.SetCurSel(index);
	m_pages[0]->ShowWindow(index == 0 ? TRUE : FALSE);
	m_pages[1]->ShowWindow(index == 1 ? TRUE : FALSE);

#ifdef COMPLETE_SOUNDEFFECT
	HRSRC res = FindResource(NULL, MAKEINTRESOURCE(IDR_COMPLETE), L"WAVE");
	m_sfx = nullptr;
	if (res) {
		HGLOBAL resource = LoadResource(NULL, res);
		if (resource) {
			int resSize = SizeofResource(NULL, res);
			m_sfx = malloc(resSize);

			if (m_sfx) {
				void* mem = LockResource(resource);
				if (mem) memcpy(m_sfx, mem, resSize);
				UnlockResource(resource);
			}
			FreeResource(resource);
		}
	}
#endif
	DWORD dwStyle = GetWindowLong(m_footer.GetSafeHwnd(), GWL_STYLE);
	SetWindowLong(m_footer.GetSafeHwnd(), GWL_STYLE, dwStyle | SS_NOTIFY);

	m_updateCheck = new std::thread([this]() {
		// Start winsock
		WSADATA data;
		WSAStartup(MAKEWORD(2, 0), &data);

		DNS_RECORD* dnsRecord;
		std::wstring versionString;

		// Look up TXT record 
		DNS_STATUS error = DnsQuery(L"drawbridge-amiga.robsmithdev.co.uk", DNS_TYPE_TEXT, DNS_QUERY_BYPASS_CACHE, NULL, &dnsRecord, NULL);
		if (!error) {
			const DNS_RECORD* record = dnsRecord;
			while (record) {
				if (record->wType == DNS_TYPE_TEXT) {
					const DNS_TXT_DATAW* pData = &record->Data.TXT;
					for (DWORD string = 0; string < pData->dwStringCount; string++) {
						const std::wstring text = pData->pStringArray[string];
						if ((text.length() >= 9) && (text.substr(0, 2) == L"v=")) versionString = text.substr(2);
					}
				}
				record = record->pNext;
			}
			DnsRecordListFree(dnsRecord, DnsFreeRecordList);  //DnsFreeRecordListDeep
		}

		if (!versionString.empty()) {
			// A little hacky to convert a.b.c.d into an array
			sockaddr_in tmp;
			INT len = sizeof(tmp);
			if (WSAStringToAddress((wchar_t*)versionString.c_str(), AF_INET, NULL, (sockaddr*)&tmp, &len) == 0) {
				const in_addr add = tmp.sin_addr;
				unsigned int newmajor, newminor, newrev;
				newmajor = add.S_un.S_un_b.s_b1;
				newminor = add.S_un.S_un_b.s_b2;
				newrev = add.S_un.S_un_b.s_b3;
				unsigned int major, minor, rev;
				if (CAboutDlg::GetProductVersion(major, minor, rev)) {
					if ((newmajor > major) ||
						((newmajor == major) && (newminor > minor)) ||
						((newmajor == major) && (newminor == minor) && (newrev > rev))) {
						// update available!
						WCHAR tmp[256];
						swprintf_s(tmp, footerStringUpdate, major, minor, rev, newmajor, newminor, newrev);
						m_footer.SetWindowTextW(tmp);
						m_updateStatus = 1;
						SetTimer(EVENT_ID_BLINK, 600, NULL);
					}
				}
			}
		}
	});

	return TRUE;  // return TRUE  unless you set the focus to a control
}

// Return the current com port
std::wstring CArduinoFloppyReaderWinDlg::getComPort() {
	int index = m_comport.GetCurSel();
	if (index < 0) return L"";
	CString buf;

	m_comport.GetWindowTextW(buf);

	return std::wstring(buf);
}

// Set the comport selector based on the input passed
void CArduinoFloppyReaderWinDlg::setComPort(const std::wstring& comport) {
	int index = m_comport.FindString(0, comport.c_str());
	if (index < 0) index = 0;
	m_comport.SetCurSel(index);
}

// Free
CArduinoFloppyReaderWinDlg::~CArduinoFloppyReaderWinDlg() {
#ifdef COMPLETE_SOUNDEFFECT
	if (m_sfx) free(m_sfx);
#endif
}

// On Sys command
void CArduinoFloppyReaderWinDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.
void CArduinoFloppyReaderWinDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// Attempt to work out what is using the com port we want
void CArduinoFloppyReaderWinDlg::findWhatIsUsingThePort() {
	std::wstring procname;

	WTS_PROCESS_INFO* pWPIs = NULL;
	DWORD dwProcCount = 0;
	if (WTSEnumerateProcesses(WTS_CURRENT_SERVER_HANDLE, NULL, 1, &pWPIs, &dwProcCount))
	{
		for (DWORD i = 0; i < dwProcCount; i++) {
			std::wstring procn = pWPIs[i].pProcessName;
			for (auto& c : procn) c = towupper(c);
			
			if (procn == L"HXCFLOPPYEMULATOR.EXE") {
				procname = L"HxCFloppyEmulator";
				break;
			}
			if (procn == L"CURA.EXE") {
				procname = L"Ultimaker Cura";
				break;
			}

		}
	}
	if (pWPIs) WTSFreeMemory(pWPIs);

	if (procname.length()) {
		procname = L"Unable to connect to this COM port.\n\n" + procname + L" is using it.\n\nPlease close this application and try again.";
		MessageBox(procname.c_str(), L"Error", MB_OK | MB_ICONEXCLAMATION);
	}
	else {
		MessageBox(L"Unable to connect to this COM port. Another application is using it.", L"Error", MB_OK | MB_ICONEXCLAMATION);
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CArduinoFloppyReaderWinDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

// Main thread
void CArduinoFloppyReaderWinDlg::runThreadRead(CRunningDlg* dlg) {
	ArduinoFloppyReader::ADFWriter writer;

	const std::wstring comPort = getComPort();

	// Try to open the com port and talk to the device
	dlg->setStatus(L"Opening COM port and setting up device...");
	if (!writer.openDevice(comPort)) {
		if (writer.getLastErrorCode() == ArduinoFloppyReader::DiagnosticResponse::drPortInUse) {
			findWhatIsUsingThePort();
		}
		else {
			std::string msg = "Unable to open COM port:\r\n\r\n";
			msg += writer.getLastError();
			MessageBoxA(GetSafeHwnd(), msg.c_str(), "Error", MB_OK | MB_ICONEXCLAMATION);
		}
		return;
	}

	// Get the current firmware version.  Only valid if openDevice is successful
	const ArduinoFloppyReader::FirmwareVersion v = writer.getFirwareVersion();
	if ((v.major == 1) && (v.minor < 9)) {
		static bool hasBeenWarnedAboutVersion = false;
		// improved disk timings in 1.8, so make them aware
		if (!hasBeenWarnedAboutVersion) {
			hasBeenWarnedAboutVersion = true;
			MessageBox(L"Rob strongly recommends updating the firmware on your DrawBridge to at least V1.9.\nThat version is even better at reading old disks.", L"Firmware Upgrade Note", MB_OK | MB_ICONINFORMATION);
		}
	}

	// Analysis was complete and found some data.  Run the reader
	unsigned int lastTrack;
	switch (m_readPage->getNumTracks()) {
	case CReadFromDiskPage::NumTracks::nt84: lastTrack = 84; break;
	case CReadFromDiskPage::NumTracks::nt82: lastTrack = 82; break;
	default: lastTrack = 80; break;
	}
	bool fluxMode = m_readPage->isExperimentalMode();
	dlg->resetProgress(lastTrack * 2);

	auto callback = [this, dlg](const int currentTrack, const ArduinoFloppyReader::DiskSurface currentSide, const int retryCounter, const int sectorsFound, const int badSectorsFound, const int maxSectors, ArduinoFloppyReader::CallbackOperation operation) -> ArduinoFloppyReader::WriteResponse {

		if (dlg->wasAbortPressed()) return ArduinoFloppyReader::WriteResponse::wrAbort;

		dlg->setOperation(operation);
		dlg->setSide(currentSide);
		dlg->setCylinder(currentTrack);
		dlg->setGoodSectors(sectorsFound, maxSectors);
		dlg->setPartialSectors(badSectorsFound);

		dlg->setProgress((currentTrack * 2) + ((currentSide == ArduinoFloppyReader::DiskSurface::dsUpper) ? 1 : 0));

		if (retryCounter > 20) {
			switch (MessageBox(L"Disk has checksum errors/missing/damaged data.\r\n\r\n", L"Disk Read Errors", MB_ABORTRETRYIGNORE)) {
			case IDABORT: return ArduinoFloppyReader::WriteResponse::wrAbort;
			case IDRETRY: return ArduinoFloppyReader::WriteResponse::wrRetry;
			case IDIGNORE: return ArduinoFloppyReader::WriteResponse::wrSkipBadChecksums;
			}
		}

		// Just continue
		return ArduinoFloppyReader::WriteResponse::wrContinue;
	};

	ArduinoFloppyReader::ADFResult readerResult;

	bool hdMode = false;


	// Detect disk speed
	if(m_mediadensity.GetCurSel() == 0) {
		if (((v.major == 1) && (v.minor >= 9)) || (v.major > 1)) {
			dlg->setStatus(L"Checking disk density...");
			if (writer.GuessDiskDensity(hdMode) != ArduinoFloppyReader::ADFResult::adfrComplete) {
				MessageBox(L"Unable to work out the density of the disk inserted.", L"Error", MB_OK | MB_ICONEXCLAMATION);
				return;
			}
		}
	}
	else hdMode = m_mediadensity.GetCurSel() == 2;

	std::wstring msg = L"Reading ";
	if (hdMode) msg += L"HD "; else msg += L"DD ";
	msg += L"disk and saving to ";
	switch (m_readPage->getImageType()) {
	case ImageType::itSCP: msg += L"SCP"; break;
	case ImageType::itADF: msg += L"ADF"; break;
	case ImageType::itIBM: msg += L"IMA"; break;
	case ImageType::itST: msg += L"ST"; break;
	}
	msg += L" image file...";

	dlg->setStatus(msg);

	

	switch (m_readPage->getImageType()) {
	case ImageType::itADF: readerResult = writer.DiskToADF(m_readPage->getFilename(), hdMode, lastTrack, callback); break;
	case ImageType::itSCP:
#ifdef _DEBUG
		readerResult = writer.DiskToSCP(m_readPage->getFilename(), hdMode, lastTrack, 2, callback, fluxMode);
#else
		readerResult = writer.DiskToSCP(m_readPage->getFilename(), hdMode, lastTrack, 3, callback, fluxMode);
#endif		
		break;
	case ImageType::itIBM:
	case ImageType::itST:
		readerResult = writer.diskToIBMST(m_readPage->getFilename(), hdMode, callback);
		break;
	}
	std::string lastError = writer.getLastError();
	writer.closeDevice();

#ifdef COMPLETE_SOUNDEFFECT
	void* sfx = m_sfx;
#else
	void* sfx = nullptr;
#endif

	// Handle the result
	switch (readerResult) {
	case ArduinoFloppyReader::ADFResult::adfrComplete: {
		CCompleteDialog::CompleteMessage msg;
		switch (m_readPage->getImageType()) {
		case ImageType::itSCP:
			msg = CCompleteDialog::CompleteMessage::cmReadOKSCP; break;
			break;
		default: msg = CCompleteDialog::CompleteMessage::cmReadOK; break;
		}
		CCompleteDialog dlg(sfx,msg );
		dlg.DoModal();
		return;
	}

	case ArduinoFloppyReader::ADFResult::adfrAborted:					return;
	case ArduinoFloppyReader::ADFResult::adfrFileError:					MessageBox(L"Unable to open the specified file to write to it.", L"Output File Error", MB_OK | MB_ICONEXCLAMATION);
	case ArduinoFloppyReader::ADFResult::adfrFileIOError:				MessageBox(L"An error occured writing to the specified file.", L"Output File Error", MB_OK | MB_ICONEXCLAMATION); 
	case ArduinoFloppyReader::ADFResult::adfrFirmwareTooOld:			MessageBox(L"This requires firmware V1.8 or newer.", L"Firmware out of date", MB_OK | MB_ICONEXCLAMATION);
	case ArduinoFloppyReader::ADFResult::adfrCompletedWithErrors: {
		CCompleteDialog dlg(sfx, CCompleteDialog::CompleteMessage::cmReadErrors);
		dlg.DoModal();
		return;
	}
	case ArduinoFloppyReader::ADFResult::adfrDriveError: {
		std::string msg = "An error occured communicating with the Arduino interface:\r\n\r\n";
		msg += lastError;
		MessageBoxA(GetSafeHwnd(), msg.c_str(), "I/O Error", MB_OK | MB_ICONEXCLAMATION);
		return;
	}
	}
}

// Main thread
void CArduinoFloppyReaderWinDlg::runThreadWrite(CRunningDlg* dlg) {
	ArduinoFloppyReader::ADFWriter writer;
	const std::wstring comPort = getComPort();

	// Try to open the com port and talk to the device
	dlg->setStatus(L"Opening COM port and setting up device...");
	if (!writer.openDevice(comPort)) {
		if (writer.getLastErrorCode() == ArduinoFloppyReader::DiagnosticResponse::drPortInUse) {
			findWhatIsUsingThePort();
		}
		else {
			std::string msg = "Unable to open COM port:\r\n\r\n";
			msg += writer.getLastError();
			MessageBoxA(GetSafeHwnd(), msg.c_str(), "Error", MB_OK | MB_ICONEXCLAMATION);
		}
		return;
	}

	// Analysis was complete and found some data.  Run the reader	
	const unsigned int lastTrack = 80;
	dlg->resetProgress(lastTrack * 2);

	bool hdMode = false;

	// Detect disk speed
	const ArduinoFloppyReader::FirmwareVersion v = writer.getFirwareVersion();

	if ((m_writePage->fileType() == ImageType::itSCP) && (m_mediadensity.GetCurSel() == 2)) {
		MessageBox(L"SCP writing is only supported for DD media.", L"Error", MB_OK | MB_ICONEXCLAMATION);
		return;
	}

	if ((m_writePage->fileType() == ImageType::itIPF) && (m_mediadensity.GetCurSel() == 2)) {
		MessageBox(L"IPF writing is only supported for DD media.", L"Error", MB_OK | MB_ICONEXCLAMATION);
		return;
	}

	const bool notFluxFile = m_writePage->fileType() < ImageType::itSCP;
	if (m_mediadensity.GetCurSel() == 0) {
		if (notFluxFile) {
			if (((v.major == 1) && (v.minor >= 9)) || (v.major > 1)) {
				dlg->setStatus(L"Checking disk density...");
				if (writer.GuessDiskDensity(hdMode) != ArduinoFloppyReader::ADFResult::adfrComplete) {
					MessageBox(L"Unable to work out the density of the disk inserted.", L"Error", MB_OK | MB_ICONEXCLAMATION);
					return;
				}
			}
		}
	}
	else hdMode = m_mediadensity.GetCurSel() == 2;
	std::wstring msg = L"Writing ";
	switch (m_writePage->fileType()) {
	case ImageType::itADF:msg = L"ADF"; break;
	case ImageType::itSCP:msg = L"SCP"; break;
	case ImageType::itST:msg = L"ST"; break;
	case ImageType::itIBM:msg = L"IMA"; break;
	case ImageType::itIPF:msg = L"IPF"; break;
	}
	msg += L" file to ";
	if (notFluxFile) 
		if (hdMode) msg += L"HD "; else msg += L"DD ";
	msg += L"disk...";

	dlg->setStatus(msg); 

	ArduinoFloppyReader::ADFResult readerResult;

	switch (m_writePage->fileType()) {
	case ImageType::itIPF: {
		readerResult = writer.IPFToDisk(m_writePage->getFilename(), m_writePage->isErase(),
			[this, dlg](const int currentTrack, const ArduinoFloppyReader::DiskSurface currentSide, const bool isVerifyError, ArduinoFloppyReader::CallbackOperation operation) -> ArduinoFloppyReader::WriteResponse {

				if (dlg->wasAbortPressed()) return ArduinoFloppyReader::WriteResponse::wrAbort;

				dlg->setOperation(operation);

				CString str;
				str.Format(L"%i", currentTrack);
				dlg->setSide(currentSide);
				dlg->setCylinder(currentTrack);
				dlg->setProgress((currentTrack * 2) + ((currentSide == ArduinoFloppyReader::DiskSurface::dsUpper) ? 1 : 0));

				// Just continue
				return ArduinoFloppyReader::WriteResponse::wrContinue;
			});
		}
		break;
	case ImageType::itSCP: {
		readerResult = writer.SCPToDisk(m_writePage->getFilename(), m_writePage->isErase(),
			[this, dlg](const int currentTrack, const ArduinoFloppyReader::DiskSurface currentSide, const bool isVerifyError, ArduinoFloppyReader::CallbackOperation operation) -> ArduinoFloppyReader::WriteResponse {

				if (dlg->wasAbortPressed()) return ArduinoFloppyReader::WriteResponse::wrAbort;

				dlg->setOperation(operation);

				CString str;
				str.Format(L"%i", currentTrack);
				dlg->setSide(currentSide);
				dlg->setCylinder(currentTrack);
				dlg->setProgress((currentTrack * 2) + ((currentSide == ArduinoFloppyReader::DiskSurface::dsUpper) ? 1 : 0));

				// Just continue
				return ArduinoFloppyReader::WriteResponse::wrContinue;
			});
		}
		break;
	case ImageType::itADF: {
		readerResult = writer.ADFToDisk(m_writePage->getFilename(), hdMode, m_writePage->isVerify(), m_writePage->isPrecomp(), m_writePage->isErase(), m_writePage->isIndex(),
			[this, dlg](const int currentTrack, const ArduinoFloppyReader::DiskSurface currentSide, const bool isVerifyError, ArduinoFloppyReader::CallbackOperation operation) -> ArduinoFloppyReader::WriteResponse {

				if (dlg->wasAbortPressed()) return ArduinoFloppyReader::WriteResponse::wrAbort;

				dlg->setOperation(operation);

				CString str;
				str.Format(L"%i", currentTrack);
				dlg->setSide(currentSide);
				dlg->setCylinder(currentTrack);
				dlg->setProgress((currentTrack * 2) + ((currentSide == ArduinoFloppyReader::DiskSurface::dsUpper) ? 1 : 0));

				if (isVerifyError) {
					switch (MessageBox(L"Verify error writing track.", L"Disk Write Error", MB_ABORTRETRYIGNORE)) {
					case IDABORT: return ArduinoFloppyReader::WriteResponse::wrAbort;
					case IDRETRY: return ArduinoFloppyReader::WriteResponse::wrRetry;
					case IDIGNORE: return ArduinoFloppyReader::WriteResponse::wrSkipBadChecksums;
					}
				}

				// Just continue
				return ArduinoFloppyReader::WriteResponse::wrContinue;
			});
		}
		break;
	case ImageType::itIBM:
	case ImageType::itST: {
		readerResult = writer.sectorFileToDisk(m_writePage->getFilename(), hdMode, m_writePage->isVerify(), m_writePage->isPrecomp(), m_writePage->isErase(), m_writePage->fileType()== ImageType::itST,
			[this, dlg](const int currentTrack, const ArduinoFloppyReader::DiskSurface currentSide, const bool isVerifyError, ArduinoFloppyReader::CallbackOperation operation) -> ArduinoFloppyReader::WriteResponse {

				if (dlg->wasAbortPressed()) return ArduinoFloppyReader::WriteResponse::wrAbort;

				dlg->setOperation(operation);

				CString str;
				str.Format(L"%i", currentTrack);
				dlg->setSide(currentSide);
				dlg->setCylinder(currentTrack);
				dlg->setProgress((currentTrack * 2) + ((currentSide == ArduinoFloppyReader::DiskSurface::dsUpper) ? 1 : 0));

				if (isVerifyError) {
					switch (MessageBox(L"Verify error writing track.", L"Disk Write Error", MB_ABORTRETRYIGNORE)) {
					case IDABORT: return ArduinoFloppyReader::WriteResponse::wrAbort;
					case IDRETRY: return ArduinoFloppyReader::WriteResponse::wrRetry;
					case IDIGNORE: return ArduinoFloppyReader::WriteResponse::wrSkipBadChecksums;
					}
				}

				// Just continue
				return ArduinoFloppyReader::WriteResponse::wrContinue;
			});
		}
		break;
	}

	std::string lastError = writer.getLastError();
	writer.closeDevice();

#ifdef COMPLETE_SOUNDEFFECT
	void* sfx = m_sfx;
#else
	void* sfx = nullptr;
#endif

	switch (readerResult) {
	case ArduinoFloppyReader::ADFResult::adfrBadSCPFile:			MessageBox(L"Unsupported or incompatiable SCP file", L"File Error", MB_OK | MB_ICONEXCLAMATION); return;
	case ArduinoFloppyReader::ADFResult::adfrComplete:					
	{
		CCompleteDialog dlg(sfx, (m_writePage->isVerify()) ? CCompleteDialog::CompleteMessage::cmWroteOK : CCompleteDialog::CompleteMessage::cmWroteNoVerify);
		dlg.DoModal();
		return;
	}
	case ArduinoFloppyReader::ADFResult::adfrCompletedWithErrors: {
		CCompleteDialog dlg(sfx, CCompleteDialog::CompleteMessage::cmWroteWithErrors);
		dlg.DoModal();
		return;
	}
	case ArduinoFloppyReader::ADFResult::adfrAborted:					return;
	case ArduinoFloppyReader::ADFResult::adfrFirmwareTooOld:            MessageBox(L"Cannot write this file, you need to upgrade the firmware first.", L"File Error", MB_OK | MB_ICONEXCLAMATION); return;
	case ArduinoFloppyReader::ADFResult::adfrExtendedADFNotSupported:	MessageBox(L"Extended ADF files are not currently supported.", L"ADF File Error", MB_OK | MB_ICONEXCLAMATION); return;
	case ArduinoFloppyReader::ADFResult::adfrMediaSizeMismatch:			if (m_writePage->fileType() == ImageType::itSCP) 
																			MessageBox(L"SCP writing is only supported for DD disks and images.", L"SCP File Error", MB_OK | MB_ICONEXCLAMATION); 
																		else
																		if (m_mediadensity.GetCurSel()) {
																			if (hdMode) MessageBox(L"You have forced the disk to be HD, but a DD file supplied.", L"File Mismatch", MB_OK | MB_ICONEXCLAMATION); else
																				MessageBox(L"You have forced the disk to be DD, but an HD file supplied.", L"File Mismatch", MB_OK | MB_ICONEXCLAMATION);
																		} else {
																			if (hdMode) MessageBox(L"Disk in drive was detected as HD, but a DD file supplied.", L"File Mismatch", MB_OK | MB_ICONEXCLAMATION); else
																				MessageBox(L"Disk in drive was detected as DD, but an HD file supplied.", L"File Mismatch", MB_OK | MB_ICONEXCLAMATION);
																		}
																		return;
	case ArduinoFloppyReader::ADFResult::adfrIPFLibraryNotAvailable:	MessageBox(L"IPF CAPSImg from Software Preservation Society Library is Missing or there was an error with the file", L"IPF Error", MB_OK | MB_ICONEXCLAMATION); break;
	case ArduinoFloppyReader::ADFResult::adfrFileError:					MessageBox(L"Unable to open the specified file to read from it.", L"Input File Error", MB_OK | MB_ICONEXCLAMATION); return;
	case ArduinoFloppyReader::ADFResult::adfrDriveError: {
		std::string msg = "An error occured communicating with the Arduino interface:\r\n\r\n";
		msg += lastError;
		MessageBoxA(GetSafeHwnd(), msg.c_str(), "I/O Error", MB_OK | MB_ICONEXCLAMATION);
		return;
	}
	case ArduinoFloppyReader::ADFResult::adfrDiskWriteProtected:		MessageBox(L"Unable to write to the disk.  Disk is write protected.", L"Write Protection Error", MB_OK | MB_ICONEXCLAMATION); return;
	default: {
		std::wstring tmp = std::wstring(L"File writing aborted with unknown error (");
		tmp += std::to_wstring((int)readerResult) + L"," + std::to_wstring((int)writer.getLastErrorCode()) + L")\r\n\r\n";
		tmp += L"If this happens again please post a screen shot of this dialog on Discord.";
		MessageBox(tmp.c_str(), L"Write Error", MB_OK | MB_ICONEXCLAMATION); return;
		return;
	}
	}
}

// Show ending message
void CArduinoFloppyReaderWinDlg::showCompletedDialog(const CCompleteDialog::CompleteMessage message) {
#ifdef COMPLETE_SOUNDEFFECT
	void* sfx = m_sfx;
#else
	void* sfx = nullptr;
#endif

	CCompleteDialog dlg(sfx, message);
	dlg.DoModal();
}

// Save the comp port to the registry
void CArduinoFloppyReaderWinDlg::saveComPort() {
	const std::wstring port = getComPort();

	char buffer[20];

	if (port.length() > 3) {
		RegSetValueW(HKEY_CURRENT_USER, L"Software\\ArduinoFloppyReader\\ComPort", REG_SZ, (LPCWSTR)port.c_str(), port.length());
	}

	_itoa_s(m_mediadensity.GetCurSel(), buffer, 10);
	RegSetValueA(HKEY_CURRENT_USER, "Software\\ArduinoFloppyReader\\disksize", REG_SZ, buffer, strlen(buffer));

	_itoa_s(m_mode.GetCurSel(), buffer, 10);
	RegSetValueA(HKEY_CURRENT_USER, "Software\\ArduinoFloppyReader\\currentmode", REG_SZ, buffer, strlen(buffer));

	m_readPage->saveSettings();
	m_writePage->saveSettings();
}

// Disk to ADF/SCP file start and stop button
void CArduinoFloppyReaderWinDlg::ReadFromDisk()
{
	saveComPort();
	std::wstring comPort = checkForComPort();
	if (comPort.length() < 1) return;

	SetCursor(m_crHourGlass);

	std::wstring title = L"Read Disk to ";
	switch (m_readPage->getImageType()) {
	case ImageType::itADF: title += L"ADF"; break;
	case ImageType::itSCP: title += L"SCP"; break;
	case ImageType::itIBM: title += L"IMA"; break;
	case ImageType::itST: title += L"ST"; break;
	default: title += L" File"; break;
	}

	CRunningDlg dlg(title, [this](CRunningDlg* dlg) {
		this->runThreadRead(dlg);
	}, this);

	dlg.DoModal();
}

// Returns the COM port if its valid and displays an error if not
std::wstring CArduinoFloppyReaderWinDlg::checkForComPort() {
	std::wstring comPort = getComPort();
	if (comPort.length() < 1) {
		MessageBox(_T("A COM port must be selected first"), _T("Error"), MB_OK | MB_ICONEXCLAMATION);
		return L"";
	}

	return comPort;
}

// Disk write button callback
void CArduinoFloppyReaderWinDlg::WriteToDisk()
{
	saveComPort();
	std::wstring comPort = checkForComPort();
	if (comPort.length() < 1) return;

	SetCursor(m_crHourGlass);
	std::wstring title = L"Write ";

	switch (m_writePage->fileType()) {
	case ImageType::itADF: title += L"ADF"; break;
	case ImageType::itSCP: title += L"SCP"; break;
	case ImageType::itIBM: title += L"IMA"; break;
	case ImageType::itST: title += L"ST"; break;
	case ImageType::itIPF: title += L"IPF"; break;
	default: title += L"File";
	}
	title += L" to Disk";
	CRunningDlg dlg(title, [this](CRunningDlg* dlg) {
		this->runThreadWrite(dlg);
	}, this);

	dlg.DoModal();
}

// The Run Diagnostics button
void CArduinoFloppyReaderWinDlg::OnBnClickedStartstop3()
{
	saveComPort();
	std::wstring comPort = checkForComPort();
	if (comPort.length() < 1) return;

	SetCursor(m_crHourGlass);
	CDiagnosticsDialog dlgDiagnostics(comPort);
	dlgDiagnostics.DoModal();
}
 
// Device change
afx_msg LRESULT CArduinoFloppyReaderWinDlg::OnDevicechange(WPARAM wParam, LPARAM lParam)
{
	// Trigger a timer to run, this is to allow the FTDI library to catch up
	KillTimer(EVENT_ID_RESCAN);
	SetTimer(EVENT_ID_RESCAN, EVENT_RESCAN_INTERVAL, NULL);
	return 0;
}

// Timer related to device change
void CArduinoFloppyReaderWinDlg::OnTimer(UINT_PTR nIDEvent)
{
	if (nIDEvent == EVENT_ID_RESCAN) {
		KillTimer(EVENT_ID_RESCAN);
		std::wstring port = getComPort();
		enumComPorts();
		setComPort(port);
	}

	if (nIDEvent == EVENT_ID_BLINK) {
		if (m_updateStatus == 1) m_updateStatus = 2; else m_updateStatus = 1;
		m_footer.Invalidate();
	}

	CDialogEx::OnTimer(nIDEvent);
}

// EEPROM
void CArduinoFloppyReaderWinDlg::OnBnClickedStartstop4()
{
	saveComPort();
	std::wstring comPort = checkForComPort();
	if (comPort.length() < 1) return;

	// Show a busy cursor
	SetCursor(m_crHourGlass);
	ArduinoFloppyReader::ArduinoInterface io;

	ArduinoFloppyReader::DiagnosticResponse response = io.openPort(comPort);
	if (response == ArduinoFloppyReader::DiagnosticResponse::drOK) {

		ArduinoFloppyReader::FirmwareVersion version = io.getFirwareVersion();
		if ((version.major == 1) && (version.minor < 9)) {
			MessageBox(_T("This feature requires at least V1.9 of the DrawBridge firmware"), _T("Sorry"), MB_OK | MB_ICONEXCLAMATION);
			return;
		}


		bool checkDisk, checkPlus, checkDD, checkSlow, error, indexAlign;
		error = io.eeprom_IsAdvancedController(checkDisk) != ArduinoFloppyReader::DiagnosticResponse::drOK;
		error |= io.eeprom_IsDrawbridgePlusMode(checkPlus) != ArduinoFloppyReader::DiagnosticResponse::drOK;
		error |= io.eeprom_IsDensityDetectDisabled(checkDD) != ArduinoFloppyReader::DiagnosticResponse::drOK;
		error |= io.eeprom_IsSlowSeekMode(checkSlow) != ArduinoFloppyReader::DiagnosticResponse::drOK;
		error |= io.eeprom_IsIndexAlignMode(indexAlign) != ArduinoFloppyReader::DiagnosticResponse::drOK;
			
		if (error) {
			MessageBox(_T("An error occured reading the settings from DrawBridge.  Please try again."), _T("Sorry"), MB_OK | MB_ICONEXCLAMATION);
			return;
		}

		// Create dialog
		CEEPromDialog dlgSettings([checkDisk, checkPlus, checkDD, checkSlow, indexAlign](CEEPromDialog& dlgSettings) {
			// Init Dialog
			dlgSettings.m_checkDiskChange.SetCheck(checkDisk);
			dlgSettings.m_checkPlus.SetCheck(checkPlus);
			dlgSettings.m_checkDD.SetCheck(checkDD);
			dlgSettings.m_checkSlow.SetCheck(checkSlow);
			dlgSettings.m_forceindex.SetCheck(indexAlign);
			
			}, [&io, this](CEEPromDialog& dlgSettings) {
				// Save settings
				SetCursor(m_crHourGlass);
				bool error = io.eeprom_SetAdvancedController(dlgSettings.m_checkDiskChange.GetCheck()) != ArduinoFloppyReader::DiagnosticResponse::drOK;
				error |= io.eeprom_SetDrawbridgePlusMode(dlgSettings.m_checkPlus.GetCheck()) != ArduinoFloppyReader::DiagnosticResponse::drOK;
				error |= io.eeprom_SetDensityDetectDisabled(dlgSettings.m_checkDD.GetCheck()) != ArduinoFloppyReader::DiagnosticResponse::drOK;
				error |= io.eeprom_SetSlowSeekMode(dlgSettings.m_checkSlow.GetCheck()) != ArduinoFloppyReader::DiagnosticResponse::drOK;
				error |= io.eeprom_SetIndexAlignMode(dlgSettings.m_forceindex.GetCheck()) != ArduinoFloppyReader::DiagnosticResponse::drOK;

				if (error) {
					MessageBox(_T("An error occured saving these settings.  Please try again."), _T("Sorry"), MB_OK | MB_ICONEXCLAMATION);
					return;
				}
			});

		dlgSettings.DoModal();
	}
	else {
		if (response == ArduinoFloppyReader::DiagnosticResponse::drPortInUse)
			findWhatIsUsingThePort(); 
		else 
			MessageBox(_T("Unable to communicate with the COM port.  Please check port or run diagnostics."), _T("Sorry"), MB_OK | MB_ICONEXCLAMATION);
		return;
	}
}

// Tab change
void CArduinoFloppyReaderWinDlg::OnSelchangeTab(NMHDR* pNMHDR, LRESULT* pResult)
{
	*pResult = 0;

	int i = TabCtrl_GetCurSel(pNMHDR->hwndFrom);
	if ((i >= 0) && ((size_t)i < m_pages.size())) m_pages[i]->ShowWindow(TRUE);
}

// Tab Changing
void CArduinoFloppyReaderWinDlg::OnSelchangingTab(NMHDR* pNMHDR, LRESULT* pResult)
{
	*pResult = 0;

	int i = TabCtrl_GetCurSel(pNMHDR->hwndFrom);
	if ((i >= 0) && ((size_t)i < m_pages.size())) m_pages[i]->ShowWindow(FALSE);
}

// On window close
void CArduinoFloppyReaderWinDlg::OnClose()
{
	saveComPort();
	CDialogEx::OnClose();
}

// Click the text at the bottom of the window, go on...!
void CArduinoFloppyReaderWinDlg::OnClickedFooter()
{
	ShellExecute(::GetDesktopWindow(), _T("OPEN"), _T("https://amiga.robsmithdev.co.uk"), NULL, NULL, SW_SHOW);
}


HBRUSH CArduinoFloppyReaderWinDlg::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor)
{
	HBRUSH hbr = CDialogEx::OnCtlColor(pDC, pWnd, nCtlColor);

	if (nCtlColor == CTLCOLOR_STATIC && pWnd->GetSafeHwnd() == GetDlgItem(IDC_FOOTER)->GetSafeHwnd())
		switch (m_updateStatus) {
		case 0: 
		case 1: pDC->SetTextColor(GetSysColor(COLOR_HOTLIGHT)); break;
		case 2: pDC->SetTextColor(RGB(255,0,0)); break;
		}

	return hbr;
}


BOOL CArduinoFloppyReaderWinDlg::OnSetCursor(CWnd* pWnd, UINT nHitTest, UINT message)
{
	if (pWnd->GetSafeHwnd() == m_footer.GetSafeHwnd()) {
		SetCursor(m_crHandPointer);
		return TRUE;
	}

	return CDialogEx::OnSetCursor(pWnd, nHitTest, message);
}


void CArduinoFloppyReaderWinDlg::OnBnClickedCleaning()
{
	saveComPort();
	std::wstring comPort = checkForComPort();
	if (comPort.length() < 1) return;

	CCleanDialog dlgClean(comPort);
	dlgClean.DoModal();
}
