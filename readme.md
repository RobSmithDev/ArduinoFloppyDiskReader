# Arduino Powered Amiga Floppy Disk Reader and Writer
Created by Robert Smith @RobSmithDev 

# What is it?
This project uses an Arduino to interface with a floppy disk drive and communicate with a PC in order to recover the data from Amiga formatted AmigaDOS disks.
It also allows you to write a backed up ADF file back onto a floppy disk!

# ArduinoFloppyReader
This Visual Studio 2019 project contains two applications, a command line, and a Windows dialog based application

# FloppyDriverController.sketch
This is the Ardunio source code/sketch

# AVR Firmware
If you want to use the AVR directly instead of within the Arduino environment, then jump to [https://github.com/jtsiomb/usbamigafloppy] where John Tsiombikas has ported the code.

# Help and Instructions 
For further details including how to wire this up please visit [http://amiga.robsmithdev.co.uk]

# Whats changed?
v2.31 Upgraded the PC code side to work with Visual Studio 2019 resolving issue #11 (ourIThome) and merging pull request #13 (bassclefstudio)
      Fixed a few typos in ArduinoInterface.cpp from pull request #12 (Crkk)
V2.2  Fixed 99% of checksum errors when writing by erasing the track first
V2.1  Diagnostics and potential write bug fixed
V2.0  Disk reading has been vastly improved and you can now also write disks!
V1.0  Initial release, can read disks fairly well

# Licence
This entire project is available under the GNU General Public License v3 licence.  See licence.txt for more details.