#pragma once
#include "stdafx.h"
#include <afxwin.h>
#include <functional>
#include "afxwin.h"
#include "afxcmn.h"
#include <thread>
#include <string>
#include "resource.h"


// CDiagnosticsDialog dialog used diagnostics status window
class CEEPromDialog : public CDialogEx
{
public:
	CEEPromDialog(std::function<void(CEEPromDialog& dialog)> onInitDialog, std::function<void(CEEPromDialog& dialog)> onSave) : CDialogEx(IDD_EEPROM_CONFIG), m_onSave(onSave), m_onInitDialog(onInitDialog) {};
	virtual ~CEEPromDialog() {

	}

public:
	// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_EEPROM_CONFIG };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX) {
		CDialogEx::DoDataExchange(pDX);
		DDX_Control(pDX, IDC_CHECK1, m_checkDiskChange);
		DDX_Control(pDX, IDC_CHECK4, m_checkPlus);
		DDX_Control(pDX, IDC_CHECK5, m_checkDD);
		DDX_Control(pDX, IDC_CHECK6, m_checkSlow);
		DDX_Control(pDX, IDC_FORCEINDEX, m_forceindex);
	};


	BOOL OnInitDialog()
	{
		CDialogEx::OnInitDialog();
		if (m_onInitDialog) m_onInitDialog(*this);
		return true;
	}

protected:
	DECLARE_MESSAGE_MAP()

private:
	std::function<void(CEEPromDialog& dialog)> m_onInitDialog, m_onSave;
public:
	CButton m_checkDiskChange;
	CButton m_checkPlus;
	CButton m_checkDD;
	CButton m_checkSlow;
	afx_msg void OnBnClickedOk() {
		if (m_onSave) m_onSave(*this);
		CDialogEx::OnOK();
	};
	CButton m_forceindex;
};

BEGIN_MESSAGE_MAP(CEEPromDialog, CDialogEx)
	ON_BN_CLICKED(IDOK, &CEEPromDialog::OnBnClickedOk)
END_MESSAGE_MAP()