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
    wxButton *readButton;
    wxButton *writeButton;
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
    : wxFrame(NULL, wxID_ANY, "Arduino Amiga Floppy Disk Reader and Writer", wxDefaultPosition, wxSize(600, 650))
{
    CreateStatusBar();
    SetStatusText("Welcome to wxWidgets!");

    wxBoxSizer *vbox = new wxBoxSizer(wxVERTICAL);
    wxBoxSizer *hbox1 = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer *hbox2 = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer *hbox3 = new wxBoxSizer(wxHORIZONTAL);
    wxStaticBox *frameDiag = new wxStaticBox(this, wxID_ANY, "Arduino Configuration", wxDefaultPosition, wxSize(600, 200));
    wxStaticBox *frameCopy = new wxStaticBox(this, wxID_ANY, "Copy Disk to ADF or SCP File (read)", wxDefaultPosition, wxSize(600, 200));
    wxStaticBox *frameWrite = new wxStaticBox(this, wxID_ANY, "Write ADF File to Disk (write)", wxDefaultPosition, wxSize(600, 200));

    auto arduinoLabel = new wxStaticText(this, -1, "Arduino connected on");
    frameDiag->SetSizer(hbox1);
    frameCopy->SetSizer(hbox2);
    frameWrite->SetSizer(hbox3);
    hbox1->Add(arduinoLabel);
    vbox->Add(frameDiag, 1);
    vbox->Add(frameCopy, 1);
    vbox->Add(frameWrite, 1);
    auto saveLabel = new wxStaticText(this, -1, "Save to");
    hbox2->Add(saveLabel);
    auto writeLabel = new wxStaticText(this, -1, "ADF File");
    hbox3->Add(writeLabel);
    this->SetSizer(vbox);

    portCombo = new wxComboBox(this, COMBO_Ports);
    hbox1->Add(portCombo);
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
    diagnosticsButton = new wxButton(frameDiag, BUTTON_Diagnostics, "Run Diagnostics");
    Connect(BUTTON_Diagnostics, wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(MyFrame::OnDiagnostics));
    hbox1->Add(diagnosticsButton);
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
