# DRAWBRIDGE aka Arduino Powered Amiga Floppy Disk Reader and Writer
Created by Robert Smith @RobSmithDev 

# What is it?
This project uses an Arduino to interface with a floppy disk drive and communicate with a PC in order to recover the data from Amiga formatted AmigaDOS disks.
It also allows you to write a backed up ADF file back onto a floppy disk!
This project started life in 2017 for fun and has grown into a useful and easy to build tool!

# ArduinoFloppyReader
This Visual Studio 2019 project contains two applications, a command line, and a Windows dialog based application

#FloppyDriverController.sketch
This is the Ardunio source code/sketch

# Help and Instructions 
For further details including how to wire this up please visit [https://amiga.robsmithdev.co.uk]

# Source Code
The source code for these are available on GitHub at
https://github.com/RobSmithDev/ArduinoFloppyDiskReader

# Whats changed?
* v2.5  A whole load of changes including:
	Fixed an encoding issue which prevented disks being read under Kickstart 1.3 or lower.
	Added support for read "streaming" with index sync support        
        Changed read timings slightly which means more disks can now be recovered!
	Added support for PRECOMP disk writing to improve readability as you go past track 40
        Added some new functions which allow for more direct control of the drive        
* v2.4  Improved support for Usb to Serial devices based on findings from GitHub user "prickle" - firmware is now V1.7
* v2.33 Merged with Pull Request #9 (Detect and read out HD floppy disks 1.44M by kollokollo) - firmware is now V1.6
* v2.32 Merged with Pull Request #6 (Modified the behavior of the current track location on Arduino boot - paulofduarte) which also addresses issues with some drives and updated firmware to 1.4
      Made a small change to the diagnostics code to also erase the track before writing it
* v2.31 Upgraded the PC code side to work with Visual Studio 2019 resolving issue #11 (ourIThome) and merging pull request #13 (bassclefstudio)
      Fixed a few typos in ArduinoInterface.cpp from pull request #12 (Crkk)
* V2.2  Fixed 99% of checksum errors when writing by erasing the track first
* V2.1  Diagnostics and potential write bug fixed
* V2.0  Disk reading has been vastly improved and you can now also write disks!
* V1.0  Initial release, can read disks fairly well

# Licence
The majority of the source code is available multi-licensed under the terms of the Mozilla Public License Version 2.0
as published by Mozilla Corporation and the GNU General Public License, version 2 or later, as published by the Free
Software Foundation