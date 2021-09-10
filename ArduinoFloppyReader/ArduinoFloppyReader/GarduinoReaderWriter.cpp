#include <gtkmm.h>
#include <glibmm.h>
#include <iostream>
#include <thread>
#include <string>
#include <sigc++/sigc++.h>
#include "../lib/ArduinoInterface.h"
#include "../lib/ADFWriter.h"

enum Tracks
{
    eighty = 80,
    eightytwo = 82,
};

enum ReadType
{
    ADF = 1,
    SCP = 2,
};

class WindowState
{
public:
    Glib::ustring serialPort;
    ReadType type;
    Tracks tracks;
    bool verify;
    bool writeCompensation;
    Glib::ustring readFileName;
    Glib::ustring readFolder;
    Glib::ustring writeFileName;
};

class MainWindow : public Gtk::ApplicationWindow
{
public:
    MainWindow(BaseObjectType *obj, Glib::RefPtr<Gtk::Builder> const &builder)
        : Gtk::ApplicationWindow(obj), builder{builder}
    {
        get_widgets();
        std::vector<std::wstring> portList;
        ArduinoFloppyReader::ArduinoInterface::enumeratePorts(portList);
        for (std::wstring port : portList)
        {
            const std::string portString(port.begin(), port.end());
            Glib::ustring text = Glib::ustring(portString.c_str(), port.size());
            portsCombo->append(text);
        }
        if (portList.size() > 0)
        {
            portsCombo->set_active(0);
            diagnosticsButton->set_sensitive(true);
        }
        diagnosticsButton->signal_clicked().connect([this]()
                                                    {
                                                        Glib::ustring serial = portsCombo->get_active_text();
                                                        run_diagnostics(serial);
                                                    });

        auto readerCallback = [this](const int currentTrack, const ArduinoFloppyReader::DiskSurface currentSide, const int retryCounter, const int sectorsFound, const int badSectorsFound) -> ArduinoFloppyReader::WriteResponse
        {
            std::string side = (currentSide == ArduinoFloppyReader::DiskSurface::dsUpper ? "Upper" : "Lower");
            std::cout << "currentTrack: " << currentTrack << " side: " << side << " retryCount: " << retryCounter << std::endl;
            cylinderCount->set_text(Glib::ustring::sprintf("%i", currentTrack));
            goodCount->set_text(Glib::ustring::sprintf("%i", sectorsFound));
            partialCount->set_text(Glib::ustring::sprintf("%i", badSectorsFound));
            sideLabel->set_text(side);
            while (gtk_events_pending())
                gtk_main_iteration();

            if (retryCounter > 20)
            {
                if (alwaysIgnore->get_active())
                {
                    return ArduinoFloppyReader::WriteResponse::wrSkipBadChecksums;
                }
                Gtk::MessageDialog dialog = Gtk::MessageDialog(*this, "Verify error writing track.\nDisk Write Error", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_NONE);
                dialog.add_button("Abort", 1);
                dialog.add_button("Retry", 2);
                dialog.add_button("Ignore", 3);
                int response = dialog.run();
                dialog.hide();

                switch (response)
                {
                case 1:
                    return ArduinoFloppyReader::WriteResponse::wrAbort;
                case 2:
                    return ArduinoFloppyReader::WriteResponse::wrRetry;
                case 3:
                    return ArduinoFloppyReader::WriteResponse::wrSkipBadChecksums;
                }

            }

            // Just continue
            return ArduinoFloppyReader::WriteResponse::wrContinue;
        };

        fileEntry->signal_changed().connect([this]()
                                            {
                                                if (fileEntry->get_text_length() > 0 && diagnosticsButton->get_sensitive())
                                                {
                                                    copyButton->set_sensitive(true);
                                                }
                                                else
                                                {
                                                    copyButton->set_sensitive(false);
                                                }
                                            });
        copyButton->signal_clicked().connect([this, readerCallback]()
                                             {
                                                 Glib::ustring folder = folderButton->get_filename();
                                                 Glib::ustring filename = folder + "/" + fileEntry->get_buffer()->get_text();
                                                 ArduinoFloppyReader::ADFWriter writer;
                                                 Glib::ustring serial = portsCombo->get_active_text();
                                                 writer.openDevice(std::wstring(serial.begin(), serial.end()));
                                                 bool tracks80 = trackButton->get_active();
                                                 ArduinoFloppyReader::ADFResult readerResult;
                                                 if (typeSelector->get_active_text() == "ADF")
                                                 {
                                                     readerResult = writer.DiskToADF(std::wstring(filename.begin(), filename.end()), tracks80 ? 80 : 82, readerCallback);
                                                 }
                                                 else
                                                 {
                                                     readerResult = writer.DiskToSCP(std::wstring(filename.begin(), filename.end()), tracks80 ? 80 : 82, 3, readerCallback);
                                                 }
                                                 // Handle the result
                                                 handleResult(writer, readerResult);
                                             });

        writeFolderSelector->signal_file_set().connect([this]()
                                                       {
                                                           std::cout<< "got filename" << writeFolderSelector->get_filename() << std::endl;
                                                           if (this->writeFolderSelector->get_filename() != "" && diagnosticsButton->get_sensitive())
                                                           {
                                                               writeButton->set_sensitive(true);
                                                           }
                                                           else
                                                           {
                                                               writeButton->set_sensitive(false);
                                                           }
                                                       });
        auto writerCallback = [this](const int currentTrack, const ArduinoFloppyReader::DiskSurface currentSide, const bool isVerifyError) -> ArduinoFloppyReader::WriteResponse
        {
            std::string side = (currentSide == ArduinoFloppyReader::DiskSurface::dsUpper ? "Upper" : "Lower");
            std::cout << "writing: currentTrack: " << currentTrack << " side: " << side << std::endl;

            cylinderCount->set_text(Glib::ustring::sprintf("%i", currentTrack));
            sideLabel->set_text(side);
            if (isVerifyError)
            {
                Gtk::MessageDialog dialog = Gtk::MessageDialog(*this, "Verify error writing track.\nDisk Write Error", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_NONE);
                dialog.add_button("Abort", 1);
                dialog.add_button("Retry", 2);
                dialog.add_button("Ignore", 3);
                int response = dialog.run();
                dialog.hide();

                switch (response)
                {
                case 1:
                    return ArduinoFloppyReader::WriteResponse::wrAbort;
                case 2:
                    return ArduinoFloppyReader::WriteResponse::wrRetry;
                case 3:
                    return ArduinoFloppyReader::WriteResponse::wrSkipBadChecksums;
                }
            }
            // Just continue
            return ArduinoFloppyReader::WriteResponse::wrContinue;
        };
        writeButton->signal_clicked().connect([this, writerCallback]()
                                              {
                                                  Glib::ustring filename = writeFolderSelector->get_filename();
                                                  ArduinoFloppyReader::ADFWriter writer;
                                                  Glib::ustring serial = portsCombo->get_active_text();
                                                  writer.openDevice(std::wstring(serial.begin(), serial.end()));
                                                  ArduinoFloppyReader::ADFResult readerResult;
                                                  readerResult = writer.ADFToDisk(std::wstring(filename.begin(), filename.end()), verify->get_active(), writeCompensation->get_active(), writerCallback);
                                                  // Handle the result
                                                  handleResult(writer, readerResult);
                                              });
    }

    virtual ~MainWindow() = default;

private:
    Glib::RefPtr<Gtk::Builder> builder;
    Gtk::ComboBoxText *portsCombo = nullptr;
    Gtk::Button *diagnosticsButton = nullptr;
    Gtk::Button *copyButton = nullptr;
    Gtk::Label *cylinderCount = nullptr;
    Gtk::Label *sideLabel = nullptr;
    Gtk::Label *goodCount = nullptr;
    Gtk::Label *partialCount = nullptr;
    Gtk::CheckButton *verify = nullptr;
    Gtk::CheckButton *alwaysIgnore = nullptr;
    Gtk::CheckButton *writeCompensation = nullptr;
    Gtk::FileChooserButton *writeFolderSelector = nullptr;
    Gtk::Button *writeButton = nullptr;
    Gtk::Entry *fileEntry = nullptr;
    Gtk::FileChooserButton *folderButton = nullptr;
    Gtk::ComboBoxText *typeSelector = nullptr;
    Gtk::RadioButton *trackButton = nullptr;
    Gtk::Statusbar *statusBar = nullptr;

    void get_widgets()
    {
        builder->get_widget("serialPortsCombo", portsCombo);
        builder->get_widget("diagnosticsButton", diagnosticsButton);
        builder->get_widget("copyButton", copyButton);
        builder->get_widget("cylinderCount", cylinderCount);
        builder->get_widget("sideLabel", sideLabel);
        builder->get_widget("goodCount", goodCount);
        builder->get_widget("partialCount", partialCount);
        builder->get_widget("fileNameEntry", fileEntry);
        builder->get_widget("folderSelector", folderButton);
        builder->get_widget("typeSelector", typeSelector);
        builder->get_widget("radio80", trackButton);
        builder->get_widget("alwaysIgnore", alwaysIgnore);
        builder->get_widget("verify", verify);
        builder->get_widget("writeCompensation", writeCompensation);
        builder->get_widget("writeFolderSelector", writeFolderSelector);
        builder->get_widget("writeButton", writeButton);
        builder->get_widget("statusBar", statusBar);
    }

    void reset()
    {
        cylinderCount->set_text("0");
        goodCount->set_text("0");
        partialCount->set_text("0");
        sideLabel->set_text("Upper");
    }

    void handleResult(ArduinoFloppyReader::ADFWriter writer, ArduinoFloppyReader::ADFResult readerResult)
    {
        switch (readerResult)
        {
        case ArduinoFloppyReader::ADFResult::adfrComplete:
        {
            Gtk::MessageDialog dialog = Gtk::MessageDialog(*this, "Completed", false, Gtk::MESSAGE_INFO, Gtk::BUTTONS_OK);
            dialog.run();
            dialog.hide();
            break;
        }
        case ArduinoFloppyReader::ADFResult::adfrAborted:
        {
            Gtk::MessageDialog dialog = Gtk::MessageDialog(*this, "Aborted", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK);
            dialog.run();
            dialog.hide();
            break;
        }
        case ArduinoFloppyReader::ADFResult::adfrFileError:
        {
            Gtk::MessageDialog dialog = Gtk::MessageDialog(*this, "Unable to open the specified file to write to it.\nOutput File Error", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK);
            dialog.run();
            dialog.hide();
            break;
        }
        case ArduinoFloppyReader::ADFResult::adfrFileIOError:
        {
            Gtk::MessageDialog dialog = Gtk::MessageDialog(*this, "An error occured writing to the specified file.\nOutput File Error", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK);
            dialog.run();
            dialog.hide();
            break;
        }
        case ArduinoFloppyReader::ADFResult::adfrFirmwareTooOld:
        {
            Gtk::MessageDialog dialog = Gtk::MessageDialog(*this, "This requires firmware V1.8 or newer.\nFirmware out of date", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK);
            dialog.run();
            dialog.hide();
            break;
        }
        case ArduinoFloppyReader::ADFResult::adfrDiskWriteProtected:
        {
            Gtk::MessageDialog dialog = Gtk::MessageDialog(*this, "The disk is write protected", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK);
            dialog.run();
            dialog.hide();
            break;
        }
        case ArduinoFloppyReader::ADFResult::adfrCompletedWithErrors:
        {
            Gtk::MessageDialog dialog = Gtk::MessageDialog(*this, "Completed with some errors", false, Gtk::MESSAGE_INFO, Gtk::BUTTONS_OK);
            dialog.run();
            dialog.hide();
            break;
        }
        case ArduinoFloppyReader::ADFResult::adfrDriveError:
        {
            std::string msg = "An error occured communicating with the Arduino interface:\r\n\r\n";
            msg += writer.getLastError();
            Gtk::MessageDialog dialog = Gtk::MessageDialog(*this, msg.c_str(), false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK);
            dialog.run();
            dialog.hide();
            break;
        }
        }
        reset();
    }

    bool showQuestion(bool isQuestion, const std::string question)
    {
        Gtk::ButtonsType buttons = Gtk::BUTTONS_OK_CANCEL;
        if (isQuestion)
        {
            buttons = Gtk::BUTTONS_YES_NO;
        }

        Gtk::MessageDialog dialog = Gtk::MessageDialog(*this, question, false, Gtk::MESSAGE_QUESTION, buttons);
        int Answer = dialog.run();
        dialog.hide();

        // Process user choice
        bool isYes = false;
        switch (Answer)
        {
        case (Gtk::RESPONSE_OK):
            isYes = true;
            break;
        case (Gtk::RESPONSE_CANCEL):
            std::cout << "Cancel clicked." << std::endl;
            break;
        case (Gtk::RESPONSE_YES):
            std::cout << "Yes clicked." << std::endl;
            isYes = true;
            break;
        case (Gtk::RESPONSE_NO):
            std::cout << "No clicked." << std::endl;
            break;
        default:
            std::cout << "Unexpected button clicked." << std::endl;
            break;
        }
        return isYes;
    }

    void showStatus(bool isError, std::string status)
    {
        std::string strLine;

        if (isError)
            strLine = "DIAGNOSTICS FAILED: ";
        strLine += status.c_str();
        std::cerr << strLine << std::endl;
        statusBar->pop();
        statusBar->push(strLine);
    }

    void run_diagnostics(Glib::ustring serial)
    {
        ArduinoFloppyReader::ADFWriter writer;
        writer.runDiagnostics(
            std::wstring(serial.begin(), serial.end()), [this, serial](bool isError, const std::string message) -> void
            {
                while (gtk_events_pending())
                    gtk_main_iteration();
                this->showStatus(isError, message);
            },
            [this, serial](bool isQuestion, const std::string question) -> bool
            {
                while (gtk_events_pending())
                    gtk_main_iteration();
                return this->showQuestion(isQuestion, question);
            });
    }
};

int main(int argc, char *argv[])
{
    auto app = Gtk::Application::create(argc, argv, "be.sourcery.Garduino");
    auto builder = Gtk::Builder::create();

    builder->add_from_file("Garduino.glade");

    MainWindow *wnd = nullptr;

    builder->get_widget_derived("MainWindow", wnd);

    auto r = app->run(*wnd);

    delete wnd;

    return r;
}
