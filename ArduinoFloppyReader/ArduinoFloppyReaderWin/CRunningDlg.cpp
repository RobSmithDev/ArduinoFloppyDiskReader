// CRunningDlg.cpp : implementation file
//

#include "stdafx.h"
#include "ArduinoFloppyReaderWin.h"
#include "CRunningDlg.h"
#include "afxdialogex.h"



// CRunningDlg dialog

IMPLEMENT_DYNAMIC(CRunningDlg, CDialogEx)

CRunningDlg::CRunningDlg(const std::wstring windowTitle, std::function<void(CRunningDlg* dlg)> onRun, CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_RUNNING, pParent), m_onRun(onRun), m_title(windowTitle)
{

}

CRunningDlg::~CRunningDlg()
{
	if (m_iothread) {
		if (m_iothread->joinable()) m_iothread->join();
		delete m_iothread;
		m_iothread = nullptr;
	}
}

void CRunningDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_TRACKNUMBER, m_cylinder);
	DDX_Control(pDX, IDC_DISKSIDE, m_side);
	DDX_Control(pDX, IDC_GOODSECTORS, m_good);
	DDX_Control(pDX, IDC_BADSECTORS, m_partial);
	DDX_Control(pDX, IDC_MESSAGE, m_message);
	DDX_Control(pDX, IDCANCEL, m_cancelbtn);
	DDX_Control(pDX, IDC_MESSAGE2, m_operation);
}


BEGIN_MESSAGE_MAP(CRunningDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_BN_CLICKED(IDCANCEL, &CRunningDlg::OnBnClickedCancel)
	ON_MESSAGE(WM_USER, &CRunningDlg::OnUser)
END_MESSAGE_MAP()

void CRunningDlg::resetProgress(int max) {
	m_maxValue = max;
	m_stateNeedsToChange = true;
	if (m_spTaskbarList) m_spTaskbarList->SetProgressState(GetSafeHwnd(), TBPF_INDETERMINATE);
};

void CRunningDlg::setProgress(int position) {
	if (m_stateNeedsToChange) {
		m_stateNeedsToChange = false;
		if (m_spTaskbarList) m_spTaskbarList->SetProgressState(GetSafeHwnd(), TBPF_NORMAL);
	}
	if (m_spTaskbarList) m_spTaskbarList->SetProgressValue(GetSafeHwnd(), position, m_maxValue);
}

void CRunningDlg::updateDiskGrid(ArduinoFloppyReader::CallbackOperation op, int currentCylinder, ArduinoFloppyReader::DiskSurface currentSide, bool isverifyError)
{
	COLORREF colorToUse;

	switch (op) {
	case ArduinoFloppyReader::CallbackOperation::coStarting: return; break;
	case ArduinoFloppyReader::CallbackOperation::coReadingFile: return; break;
	case ArduinoFloppyReader::CallbackOperation::coReading: colorToUse = RGB(0, 128, 0); break;
	case ArduinoFloppyReader::CallbackOperation::coWriting: colorToUse = RGB(255, 255, 0); break;
	case ArduinoFloppyReader::CallbackOperation::coVerifying: colorToUse = RGB(0, 128, 0); break;
	case ArduinoFloppyReader::CallbackOperation::coErasing: colorToUse = RGB(100, 100, 255); break;
	case ArduinoFloppyReader::CallbackOperation::coRetryReading: colorToUse = RGB(0, 128, 0); break;
	case ArduinoFloppyReader::CallbackOperation::coRetryWriting: colorToUse = RGB(255, 255, 0); break;
	case ArduinoFloppyReader::CallbackOperation::coReVerifying: colorToUse = RGB(0, 128, 0); break;
	}

	if (isverifyError)
	{
		colorToUse = RGB(255, 0, 0);
	}

	if (currentSide == ArduinoFloppyReader::DiskSurface::dsLower)
	{
		m_lowerSide[currentCylinder].SetTextColor(colorToUse);
		m_lowerSide[currentCylinder].SetWindowTextW(_T("0"));
	}
	else
	{
		m_upperSide[currentCylinder].SetTextColor(colorToUse);
		m_upperSide[currentCylinder].SetWindowTextW(_T("0"));
	}

}


void CRunningDlg::setOperation(ArduinoFloppyReader::CallbackOperation op) {
	switch (op) {
	case ArduinoFloppyReader::CallbackOperation::coStarting: m_operation.SetWindowTextW(L"Preparing Interface..."); break;
	case ArduinoFloppyReader::CallbackOperation::coReading: m_operation.SetWindowTextW(L"Reading cylinder..."); break;
	case ArduinoFloppyReader::CallbackOperation::coWriting: m_operation.SetWindowTextW(L"Writing cylinder..."); break;
	case ArduinoFloppyReader::CallbackOperation::coVerifying: m_operation.SetWindowTextW(L"Verifying cylinder..."); break;
	case ArduinoFloppyReader::CallbackOperation::coRetryReading: m_operation.SetWindowTextW(L"Re-reading cylinder..."); break;
	case ArduinoFloppyReader::CallbackOperation::coRetryWriting: m_operation.SetWindowTextW(L"Re-writing cylinder..."); break;
	case ArduinoFloppyReader::CallbackOperation::coReVerifying: m_operation.SetWindowTextW(L"Re-verifying cylinder..."); break;
	case ArduinoFloppyReader::CallbackOperation::coReadingFile: m_operation.SetWindowTextW(L"Reading file..."); break;
	case ArduinoFloppyReader::CallbackOperation::coErasing: m_operation.SetWindowTextW(L"Erasing cylinder..."); break;

	}
}

// CRunningDlg message handlers
BOOL CRunningDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	SetWindowText(m_title.c_str());

	HRESULT hr = ::CoCreateInstance(CLSID_TaskbarList, NULL, CLSCTX_INPROC_SERVER, __uuidof(ITaskbarList3), reinterpret_cast<void**>(&m_spTaskbarList));
	if (SUCCEEDED(hr))
	{
		hr = m_spTaskbarList->HrInit();
	}
	else m_spTaskbarList = nullptr;

	DrawDiskGrid();

	// Main processing thread
	m_iothread = new std::thread([this]()->void {
		m_threadRunning = true;

		if (m_onRun) m_onRun(this);

		m_threadRunning = false;

		PostMessage(WM_USER, 0, 0);
		});

	return TRUE;
}



BOOL CRunningDlg::PreTranslateMessage(MSG* pMsg)
{
	if (pMsg->message == WM_KEYDOWN)
	{
		if (pMsg->wParam == VK_RETURN || pMsg->wParam == VK_ESCAPE)
		{
			return !m_threadRunning;                // Do not process further
		}
	}

	if (pMsg->message == WM_CLOSE) {
		if (m_threadRunning) return 1;
	}

	return CWnd::PreTranslateMessage(pMsg);
}


void CRunningDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == SC_CLOSE)
	{
		if (m_threadRunning) return;
	}

	CDialogEx::OnSysCommand(nID, lParam);
}


void CRunningDlg::OnBnClickedCancel()
{
	m_abortPressed = true;
	m_cancelbtn.EnableWindow(FALSE);

	if (!m_threadRunning) CDialogEx::OnCancel();
}


afx_msg LRESULT CRunningDlg::OnUser(WPARAM wParam, LPARAM lParam)
{
	CDialogEx::OnCancel();
	return 0;
}


CRect CalculateCRect(int startX, int startY, int sizeX, int sizeY, int headerOffSetX, int headerOffSetY, int x, int y, bool isHeader, bool isTopHeader)
{
	if (isHeader)
		if (isTopHeader)
			return CRect(headerOffSetX + startX + x * sizeX, startY + y * sizeY, headerOffSetX + startX + (x + 1) * sizeX, startY + (y + 1) * sizeY);
		else
			return CRect(startX + x * sizeX, headerOffSetY + startY + y * sizeY, startX + (x + 1) * sizeX, headerOffSetY + startY + (y + 1) * sizeY);
	else
		return CRect(headerOffSetX + startX + (x * sizeX), headerOffSetY + startY + y * sizeY, headerOffSetX + startX + (x + 1) * sizeX, headerOffSetY + startY + (y + 1) * sizeY);
}

void CRunningDlg::DrawDiskGrid()
{

	CFont* font = new CFont();

	font->CreateFont(
		24,                       // nHeight
		0,                        // nWidth
		0,                        // nEscapement
		0,                        // nOrientation
		FW_BOLD,                // nWeight
		FALSE,                    // bItalic
		FALSE,                    // bUnderline
		0,                        // cStrikeOut
		ANSI_CHARSET,             // nCharSet
		OUT_DEFAULT_PRECIS,       // nOutPrecision
		CLIP_DEFAULT_PRECIS,      // nClipPrecision
		DEFAULT_QUALITY,          // nQuality
		DEFAULT_PITCH | FF_SWISS, // nPitchAndFamily
		_T("Courier"));            // lpszFacename

	int startX, startY, headerOffSetX, headerOffSetY, sizeX, sizeY;
	startX = 10;
	startY = 100;
	headerOffSetX = 24;
	headerOffSetY = 24;
	sizeX = 24;
	sizeY = 24;
	int nId = 15000;

	DWORD style = WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_AUTOHSCROLL | SS_CENTERIMAGE | SS_SUNKEN | WS_BORDER | SS_CENTER;
	DWORD styleHeader = WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_AUTOHSCROLL | SS_CENTERIMAGE | SS_CENTER;

	CString tempString;

	for (int x = 0; x < 10; x++)
	{
		tempString.Format(_T("%d"), x);

		m_lowerSideHeaderTop[x].Create(tempString, styleHeader, CalculateCRect(startX, startY, sizeX, sizeY, headerOffSetX, headerOffSetY, x, 0, true, true), this, nId++);
		m_lowerSideHeaderTop[x].setTransparent(true);
		m_lowerSideHeaderTop[x].SetFont(font, false);
	}

	for (int y = 0; y < 9; y++)
	{
		tempString.Format(_T("%d"), y);

		m_lowerSideHeaderLeft[y].Create(tempString, styleHeader, CalculateCRect(startX, startY, sizeX, sizeY, headerOffSetX, headerOffSetY, 0, y, true, false), this, nId++);
		m_lowerSideHeaderLeft[y].setTransparent(true);
		m_lowerSideHeaderLeft[y].SetFont(font, false);
	}

	for (int y = 0; y < 8; y++)
	{
		for (int x = 0; x < 10; x++)
		{
			m_lowerSide[x + (y * 10)].Create(_T(""), style, CalculateCRect(startX, startY, sizeX, sizeY, headerOffSetX, headerOffSetY, x, y, false, false), this, nId++);
			m_lowerSide[x + (y * 10)].setTransparent(false);
			m_lowerSide[x + (y * 10)].SetBackgroundColor(RGB(0, 0, 0));
			m_lowerSide[x + (y * 10)].SetTextColor(RGB(255, 0, 0));
			m_lowerSide[x + (y * 10)].SetFont(font, false);
		}
	}

	m_lowerSide[80].Create(_T(""), style, CalculateCRect(startX, startY, sizeX, sizeY, headerOffSetX, headerOffSetY, 0, 8, false, false), this, nId++);
	m_lowerSide[0 + (8 * 10)].setTransparent(false);
	m_lowerSide[0 + (8 * 10)].SetBackgroundColor(RGB(0, 0, 0));
	m_lowerSide[0 + (8 * 10)].SetFont(font, false);

	m_lowerSide[81].Create(_T(""), style, CalculateCRect(startX, startY, sizeX, sizeY, headerOffSetX, headerOffSetY, 1, 8, false, false), this, nId++);
	m_lowerSide[1 + (8 * 10)].setTransparent(false);
	m_lowerSide[1 + (8 * 10)].SetBackgroundColor(RGB(0, 0, 0));
	m_lowerSide[1 + (8 * 10)].SetFont(font, false);

	startX = 290;
	startY = 100;

	for (int x = 0; x < 10; x++)
	{
		tempString.Format(_T("%d"), x);

		m_upperSideHeaderTop[x].Create(tempString, styleHeader, CalculateCRect(startX, startY, sizeX, sizeY, headerOffSetX, headerOffSetY, x, 0, true, true), this, nId++);
		m_upperSideHeaderTop[x].setTransparent(true);
		m_upperSideHeaderTop[x].SetFont(font, false);
	}

	for (int y = 0; y < 9; y++)
	{
		tempString.Format(_T("%d"), y);

		m_upperSideHeaderLeft[y].Create(tempString, styleHeader, CalculateCRect(startX, startY, sizeX, sizeY, headerOffSetX, headerOffSetY, 0, y, true, false), this, nId++);
		m_upperSideHeaderLeft[y].setTransparent(true);
		m_upperSideHeaderLeft[y].SetFont(font, false);
	}

	for (int y = 0; y < 8; y++)
	{
		for (int x = 0; x < 10; x++)
		{
			m_upperSide[x + (y * 10)].Create(_T(""), style, CalculateCRect(startX, startY, sizeX, sizeY, headerOffSetX, headerOffSetY, x, y, false, false), this, nId++);
			m_upperSide[x + (y * 10)].setTransparent(false);
			m_upperSide[x + (y * 10)].SetBackgroundColor(RGB(0, 0, 0));
			m_upperSide[x + (y * 10)].SetFont(font, false);
		}
	}

	m_upperSide[80].Create(_T(""), style, CalculateCRect(startX, startY, sizeX, sizeY, headerOffSetX, headerOffSetY, 0, 8, false, false), this, nId++);
	m_upperSide[0 + (8 * 10)].setTransparent(false);
	m_upperSide[0 + (8 * 10)].SetBackgroundColor(RGB(0, 0, 0));
	m_upperSide[0 + (8 * 10)].SetFont(font, false);

	m_upperSide[81].Create(_T(""), style, CalculateCRect(startX, startY, sizeX, sizeY, headerOffSetX, headerOffSetY, 1, 8, false, false), this, nId++);
	m_upperSide[1 + (8 * 10)].setTransparent(false);
	m_upperSide[1 + (8 * 10)].SetBackgroundColor(RGB(0, 0, 0));
	m_upperSide[1 + (8 * 10)].SetFont(font, false);
}
