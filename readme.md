# DrawBridge aka Arduino Powered Floppy Disk Reader and Writer
...with 3rd party interfaces for Amiga, ATARI ST and DOS/PC Disk formats
and designed to work with WinUAE and Amiberry!

Created by Robert Smith @RobSmithDev
https://amiga.robsmithdev.co.uk

# What is it?
This project uses an Arduino to interface with a floppy disk drive and 
communicate with a PC in order to recover the data from any formatted 
disks. This includes Windows software to copy from and to ADF files.

This project is what is behind getting real floppy disks loading in real time into emulators 
like WinUAE and Amiberry

# Formats
The Arduino firmware allows to read the raw MFM data from each track of the
floppy. Decoding of the sector data is done on the PC. Usually a floppy image
file is created (ADF for AMIGA, .img for ATARI ST and PC/DOS).

The 'firmware' can read MFM data from SD, DD and HD disks, although HD is untested.
This would therefore be able to read data from AMIGA, ATARI ST, PC DOS, COMMODORE C64 etc.

# ArduinoFloppyReader
This **Visual Studio 2019** and **Linux** project contains two applications, a command line, 
and a Windows dialog based application allow reading and writing of Amiga 
formatted DD floppy disks.
Using the supplied makefile you should be able to **compile this on Linux**.  It has
been tested with Raspberry Pi OS (Raspbian - Debian-based)

# FloppyDriverController.sketch
This is the Arduino source code/sketch for all Floppy formats.
* Detect disk density (SD/DD or HD)
* Motor ON/OFF
* Seek to Track 0
* Seek to any track (up to 82 - be careful, this can damage some drives!)
* read write protection status
* Read index pulse
* read raw track data (its RAW, so FM, MFM; SD, DD or HD)
* write track data (in DD with precompensation and HD)
* Evaulate drive quality

# Help and Instructions 
For further details including how to wire this up please visit 
[https://amiga.robsmithdev.co.uk] or join the discussion on discord at [https://discord.gg/HctVgSFEXu]

# Whats changed?
* v2.8.8 Added support for reading and writing track 82 and 83 (firmware update needed too, not all drives support this anyway)
         Fixed a mistake in diagnostics suggesting pin A0 was write protect, it's A1!
         Now hides any CH340 com ports from the list as they're incompatible anyway
		 Fixed compiling under Linux
* v2.8.7 Improved support for long track disks and general compatability improvements mainly for UAE! - Make sure you update the firmware
* v2.8.6 Better extraction of disk revolutions and a few bugs fixed
		 Accurate flux extraction available but experimental so you can choose to use it or not (can't handle some long-track disks)
         Modification of older flux reading method for more better compatability
		 New firmware *1.9.23* includes basic PLL support for UAE to provide better support for games using Weak/Flaky bits
         Improved IPF & SCP writing by changing some of the parameters
         Console application now has an icon and version information
		 Tidied up some of the assembly language in the Arduino Sketch
	     Improvements and testing with the help of Dimitris Panokostas aka MiDWaN (Amiberry)
* v2.8.5 Improved IPF support for non-index aligned images (SCP and IPF writing needs 1.9.21 firmware)
         Added warning if known applications are running that may be using the serial port
* v2.8.4 Support for forcing Index Alignment of all writes, even in emulators (only for use if you know what you are doing)
         Added a timeout when connecting and syncing with the port
* v2.8.3 Better handling with the 'gap' or 'splice' within IPF files
* v2.8.2 Fixed a few more issues with SCP
         Added experimental support for writing IPF files
* v2.8.1 Fixed and improved some of the SCP support
* v2.8  Firmware 1.9.18 (see sketch notes)
        Added support for writing SCP files back to disk.  If more than one revolution exists in the file, the software will pick the second one.
* v2.7  Firmware 1.9.15 (see sketch notes)
	Added support for reading and writing HD Amiga floppy disks
	Added 'auto-detect' HD floppy disk inserted
	Fixed race condition with aborting reading
	Overhalled the UI to make it much cleaner
        Automatic update check (this is done in a way that is 100% private, there is no way I will ever know you checked for update)
	Slightly updated logo
	New EEPROM options to control how DrawBridge behaves
	Massively improved diagnostics:
		Can now detect DrawBridge vs DrawBridge Plus
		Fixed issue with read diagnostics
		Can detect the RPM of the drive for faults
		Will perform diagnostics for DD and HD
		Lists enabled and supported features of your DrawBridge setup
	Added optional support for direct access via the FTDI driver instead of COM ports
	Added ability to perform erase cycles before writing to a disk
	Added option for ADFs to be written index aligned
	Improved accuracy for *UAE disk reading	and writing
* v2.6  (firmware 1.8a) - Add support for 'noclick'
        Fixed issue with SCP/ADF file extension being incorrect if you change the type after choosing the filename
	Added some more tests during diagnostics
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

# Scripts for linux
The above application apparently works under WINE, however,
Github user "kollokollo" made some scripts for reading other formats on Linux 
too as follows:
	The ATARI ST and DOS/PC floppy formats can be decoded whit these scripts.
	9,10,11 or 18 Sectors per track. Up to 82 tracks, DD (ca. 800 kBytes) or 
	HD (1.4 MBytes). The images usually contain a FAT12 file system which can be 
	directly mounted by linux without any additional driver.   
	For more information see 
		https://github.com/kollokollo/ArduinoFloppyDiskReader/tree/master/for_linux
		They need the X11-Basic interpreter from http://x11-basic.sourceforge.net/

# Commodore 1581 Disks
	To read commodore 1581 disks, check out the project at: 
		https://github.com/hpingel/pyAccess1581

# AVR Firmware
If you want to use the AVR directly instead of within the Arduino environment, 
then jump to [https://github.com/jtsiomb/usbamigafloppy] where John Tsiombikas 
has ported the code.


# Licence
This entire project is available under the GNU General Public License v3
licence.  See licence.txt for more details.
