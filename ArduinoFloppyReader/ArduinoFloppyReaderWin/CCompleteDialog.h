#pragma once

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



