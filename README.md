# Arduino Powered Floppy Disk Reader and Writer
Created by Robert Smith @RobSmithDev
with interfaces for Amiga, ATARI ST and DOS/PC Disk formats.

# What is it?
This project uses an Arduino to interface with a floppy disk drive and 
communicate with a PC in order to recover the data from any formatted 
disks. The drive can be either a 8", 5 1/4" or 3 1/2" standard floppy drive.
It was tested on a 3 1/2" standard PC floppy drive. Others (like the 
5 1/4" standard PC floppy drive) my also work without modifications.

This configration can read SD, DD and HD floppy disk formats 
(AMIGA, ATARI ST, PC DOS, COMMODORE C64) 
and maybe more. 

The Arduino firmware allows to read the raw data from each track of the
floppy. Decoding of the sector data is done on the PC. Usually a floppy image
file is created (ADF for AMIGA, .img for ATARI ST and PC/DOS).

It also allows you to write a backed up ADF file back onto a floppy disk!

# ArduinoFloppyReader
This Visual Studio 2017 project contains two applications, a command line, 
and a Windows dialog based application

# Scripts for linux
The ATARI ST and DOS/PC floppy formats can be decoded whith these scripts.
9,10,11 or 18 Sectors per track. Up to 82 tracks, DD (ca. 800 kBytes) or 
HD (1.4 MBytes). The images usually contain a FAT12 file system which can be 
directly mounted by linuy without any additional driver.   
Sorry, no interface to WINDOWS yet. 

# FloppyDriverController.sketch
This is the Ardunio source code/sketch for all Floppy formats.
* Detect disk density (SD/DD or HD)
* Motor ON/OFF
* Seek to Track 0
* Seek to any track (up to 82)
* read write protection status
* Read index pulse
* read raw track data (FM, MFM; SD, DD or HD)
* write track data (unbuffered, only DD yet)

# AVR Firmware
If you want to use the AVR directly instead of within the Arduino environment, 
then jump to [https://github.com/jtsiomb/usbamigafloppy] where John Tsiombikas 
has ported the code.

# Help and Instructions 
For further details including how to wire this up please visit 
[http://amiga.robsmithdev.co.uk]

# Whats changed?
V2.2 Fixed 99% of checksum errors when writing by erasing the track first
V2.1 Diagnostics and potential write bug fixed
V2.0 Disk reading has been vastly improved and you can now also write disks!
V1.0 Initial release, can read disks fairly well

# Licence

This entire project is available under the GNU General Public License v3
licence.  See licence.txt for more details.

