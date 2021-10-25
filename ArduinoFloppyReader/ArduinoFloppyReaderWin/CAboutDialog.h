#pragma once

#include <Windows.h>
#include "resource.h"

#pragma comment(lib,"Version.lib")


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
	virtual void DoDataExchange(CDataExchange* pDX) {
		CDialogEx::DoDataExchange(pDX); DDX_Control(pDX, IDC_VERSIONNUMBER, m_version);
	};

protected:
	DECLARE_MESSAGE_MAP()
public:
	CStatic m_version;

    BOOL CAboutDlg::OnInitDialog()
    {
        CDialogEx::OnInitDialog();

        unsigned int major, minor, rev;

        if (GetProductVersion(major, minor, rev)) {
            WCHAR buffer[30];
            swprintf_s(buffer, L"V%i.%i (%i)", major, minor, rev);

            m_version.SetWindowText(buffer);
        }

        return TRUE; 
    }

    static bool GetProductVersion(unsigned int& major, unsigned int& minor, unsigned int& rev)
    {
        TCHAR szFilename[MAX_PATH + 1] = { 0 };
        if (!GetModuleFileName(NULL, szFilename, MAX_PATH)) return false;

        DWORD size = GetFileVersionInfoSize(szFilename, NULL);
        std::vector<unsigned char> data(size);

        if (!GetFileVersionInfo(szFilename, NULL, data.size(), &data[0])) return false;

        UINT length;
        VS_FIXEDFILEINFO* verInfo = NULL;

        if (VerQueryValue(&data[0], _T("\\"), (LPVOID*)&verInfo, &length)) {
            major = HIWORD(verInfo->dwProductVersionMS);
            minor = LOWORD(verInfo->dwProductVersionMS);
            rev = HIWORD(verInfo->dwProductVersionLS);
            return true;
        }

        return false;
    }

    void OnClickUrl1() {
        ShellExecute(::GetDesktopWindow(), _T("OPEN"), _T("https://amiga.robsmithdev.co.uk"), NULL, NULL, SW_SHOW);
    }
    void OnClickUrl2() {
        ShellExecute(::GetDesktopWindow(), _T("OPEN"), _T("https://discord.gg/HctVgSFEXu"), NULL, NULL, SW_SHOW);
    }
};

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
    ON_STN_CLICKED(IDC_ABOUTURL1, &CAboutDlg::OnClickUrl1)
    ON_STN_CLICKED(IDC_ABOUTURL2, &CAboutDlg::OnClickUrl2)
END_MESSAGE_MAP()
