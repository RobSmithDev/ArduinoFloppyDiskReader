#include "ADFWriter.h"

class BulkWriter
{

public:
    void writeDirectory(const std::wstring& port, const std::wstring& directory, const bool inHDMode, bool verify);

private:
    ArduinoFloppyReader::ADFWriter m_adfWriter;

    int adf2Disk(const std::wstring& filename, bool verify);


#ifndef _WIN32
#include <stdio.h>
#include <termios.h>

    /* Initialize new terminal i/o settings */
    void initTermios(int echo);

    /* Restore old terminal i/o settings */
    void resetTermios(void);

    char _getChar();

    std::wstring atw(const std::string& str);
#endif

};