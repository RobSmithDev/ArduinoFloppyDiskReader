#pragma once


#include "afxwin.h"
#include "afxcmn.h"
#include <thread>
#include <string>
#include "resource.h"
#include <mmsystem.h>

// CCompleteDialog dialog used to show copy/write completion
class CCompleteDialog : public CDialogEx
{
private:
	void* m_sfx;
	HBITMAP m_currentBitmap;

public:
	enum class CompleteMessage { cmReadOK, cmReadErrors, cmReadOKSCP, cmWroteOK, cmWroteWithErrors, cmWroteNoVerify };

	CCompleteDialog(void* sfx, CompleteMessage message);
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
	afx_msg void OnBnClickedOk();
};



