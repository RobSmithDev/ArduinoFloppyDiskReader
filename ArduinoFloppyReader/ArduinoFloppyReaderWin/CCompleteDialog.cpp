
#include "stdafx.h"
#include "ArduinoFloppyReaderWin.h"
#include "CCompleteDialog.h"
#include "afxdialogex.h"

CCompleteDialog::CCompleteDialog(void* sfx, CompleteMessage message) : CDialogEx(IDD_DIALOG) {
	m_sfx = sfx;
	int id;

	switch (message) {
	case CompleteMessage::cmReadOK: id = IDB_DIALOG_OK; break;
	case CompleteMessage::cmReadErrors: id = IDB_DIALOG_WARNING; break;
	case CompleteMessage::cmReadOKSCP: id = IDB_SCPREAD; break;
	case CompleteMessage::cmWroteOK: id = IDB_DIALOG_OK_WRITE; break;
	case CompleteMessage::cmWroteWithErrors: id = IDB_DIALOG_WARNING_WRITE; break;
	case CompleteMessage::cmWroteNoVerify: id = IDB_DIALOG_OK_NOVERIFY; break;
	}

	m_currentBitmap = LoadBitmap(AfxGetInstanceHandle(), MAKEINTRESOURCE(id));
}

CCompleteDialog::~CCompleteDialog() {
	if (m_currentBitmap) DeleteObject(m_currentBitmap);
}

BOOL CCompleteDialog::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	m_dialogImage.SetBitmap(m_currentBitmap);
	RECT rec;
	GetClientRect(&rec);
	m_dialogImage.SetWindowPos(NULL, 0, 0, rec.right, rec.bottom, SWP_NOACTIVATE | SWP_NOZORDER);

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
	ON_BN_CLICKED(IDOK, &CCompleteDialog::OnBnClickedOk)
END_MESSAGE_MAP()


void CCompleteDialog::OnBnClickedOk()
{
	// TODO: Add your control notification handler code here
	CDialogEx::OnOK();
}
