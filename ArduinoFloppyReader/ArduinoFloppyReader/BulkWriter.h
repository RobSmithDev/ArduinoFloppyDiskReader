#pragma once

#include "ADFWriter.h"

class BulkWriter
{

public:
    void writeDirectory(const std::wstring& port, const std::wstring& directory, const bool inHDMode, bool verify);

private:
    ArduinoFloppyReader::ADFWriter m_adfWriter;

    int adf2Disk(const std::wstring& filename, const bool inHDMode, bool verify);
};