#include <wx/wx.h>
#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include "../lib/ArduinoInterface.h"
#include "../lib/ADFWriter.h"

class WarduinoApp : public wxApp
{
public:
    virtual bool OnInit();
};

class MyFrame : public wxFrame
{
public:
    MyFrame();

private:
    // wxTextCtrl *textctrl;
    wxComboBox *portCombo;
    wxButton *diagnosticsButton;
    void OnDiagnostics(wxCommandEvent &WXUNUSED(event));
    void run_diagnostics(std::string serial);
    void showStatus(bool isError, std::string status);
    bool showQuestion(bool isQuestion, const std::string question);
};

enum
{
    BUTTON_Diagnostics = wxID_HIGHEST + 1,
    COMBO_Ports = BUTTON_Diagnostics + 1,
};

bool WarduinoApp::OnInit()
{
    MyFrame *frame = new MyFrame();
    frame->Show(true);
    return true;
}
MyFrame::MyFrame()
    : wxFrame(NULL, wxID_ANY, "Arduino Amiga Floppy Disk Reader and Writer")
{
    CreateStatusBar();
    SetStatusText("Welcome to wxWidgets!");

    portCombo = new wxComboBox(this, COMBO_Ports);
    std::vector<std::wstring> portList;
    ArduinoFloppyReader::ArduinoInterface::enumeratePorts(portList);
    for (std::wstring port : portList)
    {
        const std::string portString(port.begin(), port.end());
        portCombo->Append(portString);
    }
    if (portList.size() > 0) {
        portCombo->SetSelection(0);
    }
    diagnosticsButton = new wxButton(this, BUTTON_Diagnostics, "Run Diagnostics");
    Connect(BUTTON_Diagnostics, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(MyFrame::OnDiagnostics));
    wxBoxSizer *vbox = new wxBoxSizer(wxVERTICAL);
    wxBoxSizer *hbox1 = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer *hbox2 = new wxBoxSizer(wxHORIZONTAL);

    hbox1->Add(new wxStaticText(this, -1, "Arduino connected on"));
    hbox1->Add(portCombo);
    hbox1->Add(diagnosticsButton);
    vbox->Add(hbox1, 1, wxEXPAND);


    vbox->Add(hbox2, 0, wxALIGN_CENTER, 10);
    this->SetSizer(vbox);

    Centre();
    Show(true);
}
void MyFrame::OnDiagnostics(wxCommandEvent &WXUNUSED(event))
{
    auto serial = this->portCombo->GetValue();
    run_diagnostics(serial.c_str().AsChar());
}

bool MyFrame::showQuestion(bool isQuestion, const std::string question)
{
    long style = (wxOK | wxCANCEL);
    if (isQuestion)
    {
        style = (wxYES | wxNO);
    }

    wxMessageDialog dial(NULL, question.c_str(), wxT("Question"), style | wxICON_QUESTION);
    auto Answer = dial.ShowModal();
    // Process user choice
    bool isYes = false;
    switch (Answer)
    {
    case (wxID_OK):
        isYes = true;
        break;
    case (wxID_CANCEL):
        std::cout << "Cancel clicked." << std::endl;
        break;
    case (wxID_YES):
        std::cout << "Yes clicked." << std::endl;
        isYes = true;
        break;
    case (wxID_NO):
        std::cout << "No clicked." << std::endl;
        break;
    default:
        std::cout << "Unexpected answer: " << Answer << std::endl;
        break;
    }
    return isYes;
}

void MyFrame::showStatus(bool isError, std::string status)
{
    std::string strLine;

    if (isError)
        strLine = "DIAGNOSTICS FAILED: ";
    strLine += (status + "\n").c_str();
    std::cerr << strLine;
    auto statusBar = GetStatusBar();
    statusBar->SetStatusText(strLine);
}

void MyFrame::run_diagnostics(std::string serial)
{
    ArduinoFloppyReader::ADFWriter writer;
    writer.runDiagnostics(
        std::wstring(serial.begin(), serial.end()), [this, serial](bool isError, const std::string message) -> void
        { this->showStatus(isError, message); },
        [this, serial](bool isQuestion, const std::string question) -> bool
        {
            return this->showQuestion(isQuestion, question);
        });
}

wxIMPLEMENT_APP(WarduinoApp);
