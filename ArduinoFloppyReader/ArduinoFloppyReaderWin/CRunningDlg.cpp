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
	DDX_Control(pDX, IDC_PROGRESS, m_progress);
	DDX_Control(pDX, IDCANCEL, m_cancelbtn);
	DDX_Control(pDX, IDC_MESSAGE2, m_operation);
}


BEGIN_MESSAGE_MAP(CRunningDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_BN_CLICKED(IDCANCEL, &CRunningDlg::OnBnClickedCancel)
	ON_MESSAGE(WM_USER, &CRunningDlg::OnUser)
END_MESSAGE_MAP()

void CRunningDlg::resetProgress(int max) { 
	m_progress.SetPos(0); 
	m_progress.SetRange(0, max); 
	m_maxValue = max;
	m_stateNeedsToChange = true;
	if (m_spTaskbarList) m_spTaskbarList->SetProgressState(GetSafeHwnd(), TBPF_INDETERMINATE);
};

void CRunningDlg::setProgress(int position) { 
	m_progress.SetPos(position); 
	if (m_stateNeedsToChange) {
		m_stateNeedsToChange = false;
		if (m_spTaskbarList) m_spTaskbarList->SetProgressState(GetSafeHwnd(), TBPF_NORMAL);
	}
	if (m_spTaskbarList) m_spTaskbarList->SetProgressValue(GetSafeHwnd(), position, m_maxValue);
};

void CRunningDlg::setOperation(ArduinoFloppyReader::CallbackOperation op) {
	switch (op) {
	case ArduinoFloppyReader::CallbackOperation::coStarting : m_operation.SetWindowTextW(L"Preparing Interface..."); break;
	case ArduinoFloppyReader::CallbackOperation::coReading : m_operation.SetWindowTextW(L"Reading cylinder..."); break;
	case ArduinoFloppyReader::CallbackOperation::coWriting : m_operation.SetWindowTextW(L"Writing cylinder..."); break;
	case ArduinoFloppyReader::CallbackOperation::coVerifying : m_operation.SetWindowTextW(L"Verifying cylinder..."); break;
	case ArduinoFloppyReader::CallbackOperation::coRetryReading : m_operation.SetWindowTextW(L"Re-reading cylinder..."); break;
	case ArduinoFloppyReader::CallbackOperation::coRetryWriting : m_operation.SetWindowTextW(L"Re-writing cylinder..."); break;
	case ArduinoFloppyReader::CallbackOperation::coReVerifying: m_operation.SetWindowTextW(L"Re-verifying cylinder..."); break;
	case ArduinoFloppyReader::CallbackOperation::coReadingFile: m_operation.SetWindowTextW(L"Reading file..."); break;
		
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
