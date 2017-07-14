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

#include "stdafx.h"
#include "ArduinoFloppyReaderWin.h"
#include "ArduinoFloppyReaderWinDlg.h"
#include "afxdialogex.h"
#include <mmsystem.h>
#include "..\lib\ADFWriter.h"
#include "afxwin.h"
#include <Windows.h>


#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()




// CAboutDlg dialog used for App About

class CCompleteDialog : public CDialogEx
{
private:
	void* m_sfx;
	HBITMAP m_currentBitmap;
public:
	CCompleteDialog(void* sfx, bool partial);
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

CCompleteDialog::CCompleteDialog(void* sfx, bool partial) : CDialogEx(IDD_DIALOG)
{
	m_sfx = sfx;
	m_currentBitmap = LoadBitmap(AfxGetInstanceHandle(), MAKEINTRESOURCE(partial ? IDB_DIALOG_WARNING : IDB_DIALOG_OK));
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




// CArduinoFloppyReaderWinDlg dialog



CArduinoFloppyReaderWinDlg::CArduinoFloppyReaderWinDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_ARDUINOFLOPPYREADERWIN_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_reader = nullptr;
}

void CArduinoFloppyReaderWinDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMPORT, m_comport);
	DDX_Control(pDX, IDC_FILENAME, m_filename);
	DDX_Control(pDX, IDC_PROGRESS, m_progressbar);
	DDX_Control(pDX, IDC_TRACKNUMBER, m_statusTrack);
	DDX_Control(pDX, IDC_DISKSIDE, m_statusSide);
	DDX_Control(pDX, IDC_GOODSECTORS, m_statusGood);
	DDX_Control(pDX, IDC_BADSECTORS, m_statusPartial);
	DDX_Control(pDX, IDC_BROWSE, m_browseButton);
	DDX_Control(pDX, IDC_STARTSTOP, m_copyButton);
	DDX_Control(pDX, IDC_STATUS, m_statusText);
	DDX_Control(pDX, IDC_SPIN2, m_spinner);
}

BEGIN_MESSAGE_MAP(CArduinoFloppyReaderWinDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()	
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_STARTSTOP, &CArduinoFloppyReaderWinDlg::OnBnClickedStartstop)
	ON_BN_CLICKED(IDC_BROWSE, &CArduinoFloppyReaderWinDlg::OnBnClickedBrowse)
	ON_MESSAGE(WM_USER, &CArduinoFloppyReaderWinDlg::OnUserMessage)
END_MESSAGE_MAP()


// CArduinoFloppyReaderWinDlg message handlers

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

	WCHAR buf[10];
	LONG len = 10;
	memset(buf, 0, len*2);
	RegQueryValueW(HKEY_CURRENT_USER, L"Software\\ArduinoFloppyReader\\ComPort", buf, &len);
	if ((len)&&(wcslen(buf))) {
		m_comport.SetWindowTextW(buf);
	} else m_comport.SetWindowTextW(L"1");
	
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
	m_spinner.SetRange(1, 999);


	return TRUE;  // return TRUE  unless you set the focus to a control
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
bool CArduinoFloppyReaderWinDlg::runThread() {
	ArduinoFloppyReader::ADFWriter writer;

	CString tmpText;
	m_comport.GetWindowText(tmpText);
	unsigned int comPort = _ttoi(tmpText);
	m_progressbar.SetRange(0, 100);
	m_progressbar.SetPos(0);
	
	// Try to open the com port and talk to the device
	m_statusText.SetWindowText(L"Opening COM port and setting up device...");
	if (!writer.openDevice(comPort)) {
		MessageBox(L"Unable to open COM port\r\n\r\nPlease check COM port number and connection and try again.", L"Error", MB_OK | MB_ICONEXCLAMATION);
		return false;
	}

	m_statusText.SetWindowText(L"Step 1 of 2: Performing disk analysis...");	

	ArduinoFloppyReader::AnalysisResult analysisresult = writer.analyseDisk(
		[this](int progress) -> bool {
			// Update the progress bar
			m_progressbar.SetPos(progress);
			if (m_cancelButtonPressed) return false;
			return true;
		});

	// See what happened
	switch (analysisresult) {
		case ArduinoFloppyReader::AnalysisResult::arComplete:				// Anaysis is complete and ready for use
		// We allow this to continue!
		break;

		case ArduinoFloppyReader::AnalysisResult::arFailed:                  // Anaysis failed to read a disk
			MessageBox(L"No valid data found on the disk.  Is this disk AmigaDOS formatted?", L"Disk Read Error", MB_OK | MB_ICONEXCLAMATION);
			return false;
			break;

		case ArduinoFloppyReader::AnalysisResult::arAborted:                 // Analysis was aborted
			MessageBox(L"Disk copy was aborted.", L"Aborted", MB_OK | MB_ICONINFORMATION);
			return false;
			break;

		case ArduinoFloppyReader::AnalysisResult::arDriveError:              // Something wrong talking to thre drive
			MessageBox(L"Communication error with the Arduino during read.\r\nPlease powercycle the Arduino and try again.", L"Arduino IO Error", MB_OK | MB_ICONEXCLAMATION);
			return false;
			break;			
	}

	// Analysis was complete and found some data.  Run the reader
	m_statusText.SetWindowText(L"Step 2 of 2: Reading disk to ADF file...");
	CString filename;
	const unsigned int lastTrack = 80;
	m_progressbar.SetRange(0, lastTrack*2);
	m_progressbar.SetPos(0);

	m_filename.GetWindowText(filename);
	ArduinoFloppyReader::ADFResult readerResult = writer.writeADF(filename.GetBuffer(), lastTrack,
		 [this](const int currentTrack, const ArduinoFloppyReader::DiskSurface currentSide, const int retryCounter, const int sectorsFound, const int badSectorsFound) -> ArduinoFloppyReader::WriteResponse {
		
		 if (m_cancelButtonPressed) return ArduinoFloppyReader::WriteResponse::wrAbort;

		 CString str;
		 str.Format(L"%i", currentTrack);
		 m_statusTrack.SetWindowText(str);
		 m_statusSide.SetWindowText((currentSide == ArduinoFloppyReader::DiskSurface::dsUpper) ? L"Upper" : L"Lower");
		 str.Format(L"%i", sectorsFound);
		 m_statusGood.SetWindowText(str);
		 str.Format(L"%i", badSectorsFound);
		 m_statusPartial.SetWindowText(str);

		 m_progressbar.SetPos((currentTrack*2)+((currentSide == ArduinoFloppyReader::DiskSurface::dsUpper)?1:0));

		 if (retryCounter > 15) {
			 switch (MessageBox(L"Disk has checksum errors/missing/damaged data.\r\n\r\n", L"Disk Read Errors", MB_ABORTRETRYIGNORE)) {
				 case IDABORT: return ArduinoFloppyReader::WriteResponse::wrAbort;
				 case IDRETRY: break;
				 case IDIGNORE: return ArduinoFloppyReader::WriteResponse::wrSkipBadChecksums;				 
			 }
		}
		
		// Just continue
		return ArduinoFloppyReader::WriteResponse::wrContinue;
	});

	// Handle the result
	switch (readerResult) {
		case ArduinoFloppyReader::ADFResult::adfrComplete:					m_partial = false;  return true;
		case ArduinoFloppyReader::ADFResult::adfrAborted:					return false;
		case ArduinoFloppyReader::ADFResult::adfrFileError:					MessageBox(L"Unable to open the specified file to write to it.", L"Output File Error", MB_OK | MB_ICONEXCLAMATION); return false;
		case ArduinoFloppyReader::ADFResult::adfrFileIOError:				MessageBox(L"An error occured writing to the specified file.", L"Output File Error", MB_OK | MB_ICONEXCLAMATION); return false;
		case ArduinoFloppyReader::ADFResult::adfrCompletedWithErrors:		m_partial = true; return true;
		case ArduinoFloppyReader::ADFResult::adfrDriveError:				MessageBox(L"An error occured communicating with the Arduino interface.", L"I/O Error", MB_OK | MB_ICONEXCLAMATION); return false;
	}

	return false;
}

afx_msg LRESULT CArduinoFloppyReaderWinDlg::OnUserMessage(WPARAM wparam, LPARAM lparam) {
	if (m_reader) {
		m_reader->join();
		delete m_reader;
		m_reader = nullptr;
	}
	m_copyButton.SetWindowText(L"Start Copy");
	return 0;
}

void CArduinoFloppyReaderWinDlg::threadFinished(bool successful) {
	m_browseButton.EnableWindow(true);
	m_filename.EnableWindow(true);
	m_comport.EnableWindow(true);

	m_statusText.SetWindowText(L"Ready");
	m_progressbar.SetPos(0);
	m_statusTrack.SetWindowText(L"0");
	m_statusSide.SetWindowText(L"Upper");
	m_statusGood.SetWindowText(L"0");
	m_statusPartial.SetWindowText(L"0");

	if (successful) {
		if (m_partial) {
			CCompleteDialog dlg(m_sfx, true);
			dlg.DoModal();
		}
		else {
			CCompleteDialog dlg(m_sfx, false);
			dlg.DoModal();
		}
	}

	// Free the thread
	PostMessage(WM_USER, 0, 0);
}

void CArduinoFloppyReaderWinDlg::OnBnClickedStartstop()
{
	if (m_reader) {
		m_cancelButtonPressed = true;
	}
	else {		
		CString port;
		m_comport.GetWindowText(port);
		RegSetValueW(HKEY_CURRENT_USER, L"Software\\ArduinoFloppyReader\\ComPort", REG_SZ, port.GetBuffer(), port.GetLength());
		m_filename.GetWindowText(port);
		if (port.GetLength() < 1) {
			MessageBox(L"You need to specify an ADF filename first", L"Error", MB_OK | MB_ICONEXCLAMATION);
			return;
		}

		m_cancelButtonPressed = false;
		m_browseButton.EnableWindow(false);
		m_filename.EnableWindow(false);
		m_comport.EnableWindow(false);
		m_copyButton.SetWindowText(L"Abort!");

		m_reader = new std::thread([this]() {
			this->threadFinished(this->runThread());
		});
	}
}

// Choose file to save as
void CArduinoFloppyReaderWinDlg::OnBnClickedBrowse()
{
	// szFilters is a text string that includes two file name filters:
	TCHAR szFilters[] = _T("Amiga Disk Files (*.adf)|*.adf|All Files (*.*)|*.*||");

	// Create an Save dialog
	CString oldFileName;
	m_filename.GetWindowText(oldFileName);

	CFileDialog fileDlg(FALSE, _T("adf"), oldFileName, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST | OFN_NODEREFERENCELINKS | OFN_ENABLESIZING | OFN_DONTADDTORECENT | OFN_EXPLORER, szFilters, this);

	// Display it
	if (fileDlg.DoModal() == IDOK)
		m_filename.SetWindowText(fileDlg.GetPathName());
}
