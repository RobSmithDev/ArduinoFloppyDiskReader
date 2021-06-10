/* ArduinoFloppyReaderWin
*
* Copyright (C) 2017-2021 Robert Smith (@RobSmithDev)
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

//////////////////////////////////////////////////////////////////////////////////////////
// Simple Windows application using the libraries used in the console app               //
//////////////////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include <afxwin.h>
#include "ArduinoFloppyReaderWin.h"
#include "ArduinoFloppyReaderWinDlg.h"
#include "afxdialogex.h"
#include <mmsystem.h>
#include "..\lib\ADFWriter.h"
#include <Windows.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


#pragma region ABOUT_DIALOG

// CAboutDlg dialog used for App About
class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg() : CDialogEx(IDD_ABOUTBOX) {};

	// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif
protected:
	virtual void DoDataExchange(CDataExchange* pDX) { CDialogEx::DoDataExchange(pDX); };

protected:
	DECLARE_MESSAGE_MAP()
};

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()

#pragma endregion ABOUT_DIALOG


#pragma region DIAGNOSTICS_DIALOG

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

#pragma endregion DIAGNOSTICS_DIALOG


#pragma region COMPLETE_DIALOG

// CCompleteDialog dialog used to show copy/write completion
class CCompleteDialog : public CDialogEx
{
private:
	void* m_sfx;
	HBITMAP m_currentBitmap;
public:
	CCompleteDialog(void* sfx, bool partial, bool writeMode, bool SCPMode);
	virtual ~CCompleteDialog();

	// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_DIALOG };
#endif

protected:
	virtual BOOL OnInitDialog();
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	LRESULT OnNcHitTest(CPoint point);
	// Implementation
protected:
	DECLARE_MESSAGE_MAP()
public:

	virtual INT_PTR DoModal() override;
	CStatic m_dialogImage;
};

CCompleteDialog::~CCompleteDialog() {
	if (m_currentBitmap) DeleteObject(m_currentBitmap);
}

CCompleteDialog::CCompleteDialog(void* sfx, bool partial, bool writeMode, bool SCPMode) : CDialogEx(IDD_DIALOG)
{
	m_sfx = sfx;
	int id;
	if (SCPMode) id = IDB_SCPREAD; else
		if (writeMode)
			id = partial ? IDB_DIALOG_WARNING_WRITE : IDB_DIALOG_OK_WRITE;
		else id = partial ? IDB_DIALOG_WARNING : IDB_DIALOG_OK;
	m_currentBitmap = LoadBitmap(AfxGetInstanceHandle(), MAKEINTRESOURCE(id));
}

BOOL CCompleteDialog::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	m_dialogImage.SetBitmap(m_currentBitmap);
	return true;
}

void CCompleteDialog::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_DIALOGIMAGE, m_dialogImage);
}

INT_PTR CCompleteDialog::DoModal()
{
	sndPlaySound((LPCWSTR)m_sfx, SND_ASYNC | SND_MEMORY | SND_NODEFAULT);
	return CDialogEx::DoModal();
}

LRESULT CCompleteDialog::OnNcHitTest(CPoint point)
{
	CPoint pt = point;
	ScreenToClient(&pt);

	if (pt.y < 24) return HTCAPTION;
	return CDialog::OnNcHitTest(point);
}
BEGIN_MESSAGE_MAP(CCompleteDialog, CDialogEx)
	ON_WM_NCHITTEST()
END_MESSAGE_MAP()


#pragma endregion COMPLETE_DIALOG

#pragma region MAIN_DIALOG


// CArduinoFloppyReaderWinDlg dialog
CArduinoFloppyReaderWinDlg::CArduinoFloppyReaderWinDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_ARDUINOFLOPPYREADERWIN_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_iothread = nullptr;
}

void CArduinoFloppyReaderWinDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMPORT, m_comport);
	DDX_Control(pDX, IDC_FILENAME, m_outputADF);
	DDX_Control(pDX, IDC_PROGRESS, m_progressbar);
	DDX_Control(pDX, IDC_TRACKNUMBER, m_statusTrack);
	DDX_Control(pDX, IDC_DISKSIDE, m_statusSide);
	DDX_Control(pDX, IDC_GOODSECTORS, m_statusGood);
	DDX_Control(pDX, IDC_BADSECTORS, m_statusPartial);
	DDX_Control(pDX, IDC_BROWSE, m_browseButton);
	DDX_Control(pDX, IDC_STARTSTOP, m_copyButton);
	DDX_Control(pDX, IDC_STATUS, m_statusText);
	DDX_Control(pDX, IDC_FILENAME2, m_inputADF);
	DDX_Control(pDX, IDC_BROWSE2, m_browseButton2);
	DDX_Control(pDX, IDC_STARTSTOP2, m_writeButton);
	DDX_Control(pDX, IDC_CHECK1, m_verify);
	DDX_Control(pDX, IDC_CHECK3, m_precomp);
	DDX_Control(pDX, IDC_STARTSTOP3, m_diagnostics);
	DDX_Control(pDX, IDC_DISKFORMAT, m_fileFormat);
	DDX_Control(pDX, IDC_TRK80, m_trk80);
	DDX_Control(pDX, IDC_TRK82, m_trk82);
}

BEGIN_MESSAGE_MAP(CArduinoFloppyReaderWinDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_STARTSTOP, &CArduinoFloppyReaderWinDlg::OnBnClickedStartstop)
	ON_BN_CLICKED(IDC_BROWSE, &CArduinoFloppyReaderWinDlg::OnBnClickedBrowse)
	ON_MESSAGE(WM_USER, &CArduinoFloppyReaderWinDlg::OnUserMessage)
	ON_BN_CLICKED(IDC_BROWSE2, &CArduinoFloppyReaderWinDlg::OnBnClickedBrowse2)
	ON_BN_CLICKED(IDC_STARTSTOP2, &CArduinoFloppyReaderWinDlg::OnBnClickedStartstop2)
	ON_BN_CLICKED(IDC_STARTSTOP3, &CArduinoFloppyReaderWinDlg::OnBnClickedStartstop3)
	ON_MESSAGE(WM_DEVICECHANGE, &CArduinoFloppyReaderWinDlg::OnDevicechange)
	ON_CBN_SELCHANGE(IDC_DISKFORMAT, &CArduinoFloppyReaderWinDlg::OnCbnSelchangeDiskformat)
END_MESSAGE_MAP()


// CArduinoFloppyReaderWinDlg message handlers

void CArduinoFloppyReaderWinDlg::enumComPorts() {
	std::vector<std::wstring> portList;
	ArduinoFloppyReader::ArduinoInterface::enumeratePorts(portList);

	m_comport.ResetContent();
	for (const std::wstring& port : portList)
		m_comport.AddString(port.c_str());
}

BOOL CArduinoFloppyReaderWinDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	enumComPorts();

	WCHAR buf[10];
	buf[0] = '\0';
	LONG len = 10;
	memset(buf, 0, len * 2);
	RegQueryValueW(HKEY_CURRENT_USER, L"Software\\ArduinoFloppyReader\\ComPort", buf, &len);
	int port = wcstol(buf, nullptr, 10);
	swprintf_s(buf, L"COM%i", port);
	setComPort(buf);

	m_fileFormat.AddString(L"ADF");
	m_fileFormat.AddString(L"SCP");
	buf[0] = '\0';
	len = 10;
	RegQueryValueW(HKEY_CURRENT_USER, L"Software\\ArduinoFloppyReader\\fileFormat", buf, &len);
	int index = wcstol(buf, nullptr, 10);
	m_fileFormat.SetCurSel(index);

	buf[0] = '\0';
	len = 10;
	RegQueryValueW(HKEY_CURRENT_USER, L"Software\\ArduinoFloppyReader\\tracks", buf, &len);
	int numTracks = wcstol(buf, nullptr, 10);
	m_trk82.SetCheck(numTracks == 82);
	m_trk80.SetCheck(numTracks != 82);


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
		}
		FreeResource(resource);
	}
	m_precomp.SetCheck(TRUE);
	m_verify.SetCheck(TRUE);


	return TRUE;  // return TRUE  unless you set the focus to a control
}

std::wstring CArduinoFloppyReaderWinDlg::getComPort() {
	int index = m_comport.GetCurSel();
	if (index < 0) return L"";
	CString buf;

	m_comport.GetWindowTextW(buf);

	return std::wstring(buf);
}

void CArduinoFloppyReaderWinDlg::setComPort(const std::wstring& comport) {
	int index = m_comport.FindString(0, comport.c_str());
	if (index < 0) index = 0;
	m_comport.SetCurSel(index);
}


CArduinoFloppyReaderWinDlg::~CArduinoFloppyReaderWinDlg() {
	m_cancelButtonPressed = true;

	if (m_sfx) free(m_sfx);
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

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CArduinoFloppyReaderWinDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

// Main thread
bool CArduinoFloppyReaderWinDlg::runThreadRead() {
	ArduinoFloppyReader::ADFWriter writer;

	const std::wstring comPort = getComPort();
	bool isSCP = m_fileFormat.GetCurSel() == 1;

	// Try to open the com port and talk to the device
	m_statusText.SetWindowText(L"Opening COM port and setting up device...");
	if (!writer.openDevice(comPort)) {
		std::string msg = "Unable to open COM port:\r\n\r\n";
		msg += writer.getLastError();
		MessageBoxA(GetSafeHwnd(), msg.c_str(), "Error", MB_OK | MB_ICONEXCLAMATION);
		return false;
	}

	// Get the current firmware version.  Only valid if openDevice is successful
	const ArduinoFloppyReader::FirmwareVersion v = writer.getFirwareVersion();
	if ((v.major == 1) && (v.minor < 8)) {
		static bool hasBeenWarnedAboutVersion = false;
		// improved disk timings in 1.8, so make them aware
		if (!hasBeenWarnedAboutVersion) {
			hasBeenWarnedAboutVersion = true;
			MessageBox(L"Rob strongly recommends updating the firmware on your Arduino to at least V1.8.\nThat version is even better at reading old disks.", L"Firmware Upgrade Note", MB_OK | MB_ICONINFORMATION);
		}
	}

	// Analysis was complete and found some data.  Run the reader
	m_statusText.SetWindowText(isSCP ? L"Reading disk to SCP file..." : L"Reading disk to ADF file...");
	CString filename;
	const unsigned int lastTrack = m_trk82.GetCheck() ? 82 : 80;
	m_progressbar.SetRange(0, lastTrack * 2);
	m_progressbar.SetPos(0);

	auto callback = [this](const int currentTrack, const ArduinoFloppyReader::DiskSurface currentSide, const int retryCounter, const int sectorsFound, const int badSectorsFound) -> ArduinoFloppyReader::WriteResponse {

		if (m_cancelButtonPressed) return ArduinoFloppyReader::WriteResponse::wrAbort;

		CString str;
		str.Format(L"%i", currentTrack);
		m_statusTrack.SetWindowText(str);
		m_statusSide.SetWindowText((currentSide == ArduinoFloppyReader::DiskSurface::dsUpper) ? L"Upper" : L"Lower");
		str.Format(L"%i", sectorsFound);
		m_statusGood.SetWindowText(str);
		str.Format(L"%i", badSectorsFound);
		m_statusPartial.SetWindowText(str);

		m_progressbar.SetPos((currentTrack * 2) + ((currentSide == ArduinoFloppyReader::DiskSurface::dsUpper) ? 1 : 0));

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

	m_outputADF.GetWindowText(filename);

	ArduinoFloppyReader::ADFResult readerResult;


	readerResult = isSCP ? writer.DiskToSCP(filename.GetBuffer(), lastTrack, 3, callback) : writer.DiskToADF(filename.GetBuffer(), lastTrack, callback);

	// Handle the result
	switch (readerResult) {
	case ArduinoFloppyReader::ADFResult::adfrComplete:					m_partial = false;  return true;
	case ArduinoFloppyReader::ADFResult::adfrAborted:					return false;
	case ArduinoFloppyReader::ADFResult::adfrFileError:					MessageBox(L"Unable to open the specified file to write to it.", L"Output File Error", MB_OK | MB_ICONEXCLAMATION); return false;
	case ArduinoFloppyReader::ADFResult::adfrFileIOError:				MessageBox(L"An error occured writing to the specified file.", L"Output File Error", MB_OK | MB_ICONEXCLAMATION); return false;
	case ArduinoFloppyReader::ADFResult::adfrFirmwareTooOld:			MessageBox(L"This requires firmware V1.8 or newer.", L"Firmware out of date", MB_OK | MB_ICONEXCLAMATION); return false;
	case ArduinoFloppyReader::ADFResult::adfrCompletedWithErrors:		m_partial = true; return true;
	case ArduinoFloppyReader::ADFResult::adfrDriveError: {
		std::string msg = "An error occured communicating with the Arduino interface:\r\n\r\n";
		msg += writer.getLastError();
		MessageBoxA(GetSafeHwnd(), msg.c_str(), "I/O Error", MB_OK | MB_ICONEXCLAMATION);
		return false;
	}
	}

	return false;
}



// Main thread
bool CArduinoFloppyReaderWinDlg::runThreadWrite() {
	ArduinoFloppyReader::ADFWriter writer;

	const std::wstring comPort = getComPort();

	// Try to open the com port and talk to the device
	m_statusText.SetWindowText(L"Opening COM port and setting up device...");
	if (!writer.openDevice(comPort)) {
		std::string msg = "Unable to open COM port:\r\n\r\n";
		msg += writer.getLastError();
		MessageBoxA(GetSafeHwnd(), msg.c_str(), "Error", MB_OK | MB_ICONEXCLAMATION);
		return false;
	}

	// Analysis was complete and found some data.  Run the reader
	m_statusText.SetWindowText(L"Writing ADF file to disk...");
	CString filename;
	const unsigned int lastTrack = 80;
	m_progressbar.SetRange(0, lastTrack * 2);
	m_progressbar.SetPos(0);

	m_inputADF.GetWindowText(filename);

	ArduinoFloppyReader::ADFResult readerResult = writer.ADFToDisk(filename.GetBuffer(), m_verify.GetCheck() != 0, m_precomp.GetCheck() != 0,
		[this](const int currentTrack, const ArduinoFloppyReader::DiskSurface currentSide, const bool isVerifyError) -> ArduinoFloppyReader::WriteResponse {

			if (m_cancelButtonPressed) return ArduinoFloppyReader::WriteResponse::wrAbort;

			CString str;
			str.Format(L"%i", currentTrack);
			m_statusTrack.SetWindowText(str);
			m_statusSide.SetWindowText((currentSide == ArduinoFloppyReader::DiskSurface::dsUpper) ? L"Upper" : L"Lower");

			m_progressbar.SetPos((currentTrack * 2) + ((currentSide == ArduinoFloppyReader::DiskSurface::dsUpper) ? 1 : 0));

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


	switch (readerResult) {
	case ArduinoFloppyReader::ADFResult::adfrComplete:					m_partial = false;  return true;
	case ArduinoFloppyReader::ADFResult::adfrCompletedWithErrors:		m_partial = true; return true;
	case ArduinoFloppyReader::ADFResult::adfrAborted:					return false;
	case ArduinoFloppyReader::ADFResult::adfrFileError:					MessageBox(L"Unable to open the specified file to read from it.", L"Input File Error", MB_OK | MB_ICONEXCLAMATION); return false;
	case ArduinoFloppyReader::ADFResult::adfrDriveError: {
		std::string msg = "An error occured communicating with the Arduino interface:\r\n\r\n";
		msg += writer.getLastError();
		MessageBoxA(GetSafeHwnd(), msg.c_str(), "I/O Error", MB_OK | MB_ICONEXCLAMATION);
		return false;
	}
	case ArduinoFloppyReader::ADFResult::adfrDiskWriteProtected:		MessageBox(L"Unable to write to the disk.  Disk is write protected.", L"Write Protection Error", MB_OK | MB_ICONEXCLAMATION); return false;
	}

	return false;
}

afx_msg LRESULT CArduinoFloppyReaderWinDlg::OnUserMessage(WPARAM wparam, LPARAM lparam) {
	if (m_iothread) {
		if (m_iothread->joinable()) m_iothread->join();
		delete m_iothread;
		m_iothread = nullptr;
	}
	m_copyButton.SetWindowText(L"Copy Disk");
	m_writeButton.SetWindowText(L"Write Disk");
	return 0;
}

// Enabel the selectable items on the dialog
void CArduinoFloppyReaderWinDlg::enableDialog(bool enable) {
	m_browseButton.EnableWindow(enable);
	m_browseButton2.EnableWindow(enable);
	m_outputADF.EnableWindow(enable);
	m_inputADF.EnableWindow(enable);
	m_comport.EnableWindow(enable);
	m_verify.EnableWindow(enable);
	m_precomp.EnableWindow(enable);
	m_diagnostics.EnableWindow(enable);
	m_fileFormat.EnableWindow(enable);
	m_precomp.EnableWindow(enable);
	m_trk80.EnableWindow(enable);
	m_trk82.EnableWindow(enable);

	if (enable) {
		m_statusText.SetWindowText(L"Ready");
		m_progressbar.SetPos(0);
		m_statusTrack.SetWindowText(L"0");
		m_statusSide.SetWindowText(L"Upper");
		m_statusGood.SetWindowText(L"0");
		m_statusPartial.SetWindowText(L"0");
		m_copyButton.EnableWindow(true);
		m_writeButton.EnableWindow(true);
	}
}

// Called when the thread for reading ifinishes
void CArduinoFloppyReaderWinDlg::threadFinishedReading(bool successful) {
	enableDialog(true);

	if (successful) {
		if (m_partial) {
			CCompleteDialog dlg(m_sfx, true, false, false);
			dlg.DoModal();
		}
		else {
			CCompleteDialog dlg(m_sfx, false, false, m_fileFormat.GetCurSel() == 1);
			dlg.DoModal();
		}
	}

	// Free the thread
	PostMessage(WM_USER, 0, 0);
}


// Called when the thread for reading ifinishes
void CArduinoFloppyReaderWinDlg::threadFinishedWriting(bool successful) {
	enableDialog(true);

	if (successful) {
		if (m_partial) {
			CCompleteDialog dlg(m_sfx, true, true, false);
			dlg.DoModal();
		}
		else {
			CCompleteDialog dlg(m_sfx, false, true, false);
			dlg.DoModal();
		}
	}

	// Free the thread
	PostMessage(WM_USER, 0, 0);
}


// Save the comp port to the registry
void CArduinoFloppyReaderWinDlg::saveComPort() {
	const std::wstring port = getComPort();

	char buffer[20];

	if (port.length() > 3) {
		RegSetValueW(HKEY_CURRENT_USER, L"Software\\ArduinoFloppyReader\\ComPort", REG_SZ, (LPCWSTR)&port[3], port.length()-3);
	}

	_itoa_s(m_fileFormat.GetCurSel(), buffer, 10);
	RegSetValueA(HKEY_CURRENT_USER, "Software\\ArduinoFloppyReader\\fileFormat", REG_SZ, buffer, strlen(buffer));

	_itoa_s(m_trk82.GetCheck() ? 82 : 80, buffer, 10);
	RegSetValueA(HKEY_CURRENT_USER, "Software\\ArduinoFloppyReader\\tracks", REG_SZ, buffer, strlen(buffer));
}

// Disk to ADF file start and stop button
void CArduinoFloppyReaderWinDlg::OnBnClickedStartstop()
{
	if (m_iothread) {
		m_cancelButtonPressed = true;
	}
	else {
		saveComPort();

		CString filename;
		m_outputADF.GetWindowText(filename);
		if (filename.GetLength() < 1) {
			MessageBox(L"You need to specify an ADF filename first", L"Error", MB_OK | MB_ICONEXCLAMATION);
			return;
		}

		m_cancelButtonPressed = false;
		enableDialog(false);

		m_copyButton.SetWindowText(L"Abort!");
		m_writeButton.EnableWindow(false);

		m_iothread = new std::thread([this]() {
			this->threadFinishedReading(this->runThreadRead());
			});
	}
}

// ADF to Disk write button
void CArduinoFloppyReaderWinDlg::OnBnClickedStartstop2()
{
	if (m_iothread) {
		m_cancelButtonPressed = true;
	}
	else {
		saveComPort();

		CString filename;
		m_inputADF.GetWindowText(filename);
		if (filename.GetLength() < 1) {
			MessageBox(L"You need to specify an ADF filename first", L"Error", MB_OK | MB_ICONEXCLAMATION);
			return;
		}

		m_cancelButtonPressed = false;
		enableDialog(false);

		m_copyButton.EnableWindow(false);
		m_writeButton.SetWindowText(L"Abort!");

		m_iothread = new std::thread([this]() {
			this->threadFinishedWriting(this->runThreadWrite());
			});
	}
}


// Browse to select ADF file to save to
void CArduinoFloppyReaderWinDlg::OnBnClickedBrowse()
{
	// szFilters is a text string that includes two file name filters:
	TCHAR szFilters[] = _T("Amiga Disk Files (*.adf)|*.adf|All Files (*.*)|*.*||");
	TCHAR szFiltersSCP[] = _T("Supercard Pro Files (*.scp)|*.scp|All Files (*.*)|*.*||");

	bool isSCP = m_fileFormat.GetCurSel() == 1;

	// Create an Save dialog
	CString oldFileName;
	m_outputADF.GetWindowText(oldFileName);

	CFileDialog fileDlg(FALSE, isSCP ? _T("scp") : _T("adf"), oldFileName, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST | OFN_NODEREFERENCELINKS | OFN_ENABLESIZING | OFN_DONTADDTORECENT | OFN_EXPLORER, isSCP ? szFiltersSCP : szFilters, this);
	if (isSCP) fileDlg.m_ofn.lpstrTitle = L"Save Disk to SCP File"; else fileDlg.m_ofn.lpstrTitle = L"Save Disk to ADF File";

	// Display it
	if (fileDlg.DoModal() == IDOK)
		m_outputADF.SetWindowText(fileDlg.GetPathName());
}

// Browse to load and copy to disk
void CArduinoFloppyReaderWinDlg::OnBnClickedBrowse2()
{
	// szFilters is a text string that includes two file name filters:
	TCHAR szFilters[] = _T("Amiga Disk Files (*.adf)|*.adf|All Files (*.*)|*.*||");

	// Create an Save dialog
	CString oldFileName;
	m_inputADF.GetWindowText(oldFileName);

	CFileDialog fileDlg(TRUE, _T("adf"), oldFileName, OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NODEREFERENCELINKS | OFN_ENABLESIZING | OFN_EXPLORER, szFilters, this);
	fileDlg.m_ofn.lpstrTitle = L"Select an ADF File to Write To Disk";

	// Display it
	if (fileDlg.DoModal() == IDOK)
		m_inputADF.SetWindowText(fileDlg.GetPathName());
}


#pragma endregion MAIN_DIALOG

// The Run Diagnostics button
void CArduinoFloppyReaderWinDlg::OnBnClickedStartstop3()
{
	std::wstring comPort = getComPort();
	saveComPort();

	CDiagnosticsDialog dlgDiagnostics(comPort);
	dlgDiagnostics.DoModal();
}


afx_msg LRESULT CArduinoFloppyReaderWinDlg::OnDevicechange(WPARAM wParam, LPARAM lParam)
{
	std::wstring port = getComPort();
	enumComPorts();
	setComPort(port);
	return 0;
}


void CArduinoFloppyReaderWinDlg::OnCbnSelchangeDiskformat()
{
	// Correct the file extension
	bool isSCP = m_fileFormat.GetCurSel() == 1;
	CString filename;

	m_outputADF.GetWindowText(filename);

	int pos = filename.ReverseFind(_T('.'));
	if (pos > 0) {
		filename = filename.Left(pos);

		if (isSCP) filename += _T(".scp"); else filename += _T(".adf");

		m_outputADF.SetWindowText(filename);

	}
}
