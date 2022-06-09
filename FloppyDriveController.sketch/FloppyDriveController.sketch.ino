/* DrawBridge a.k.a ArduinoFloppyReader (and writer)
*
* Copyright (C) 2017-2022 Robert Smith (@RobSmithDev)
* https://amiga.robsmithdev.co.uk
*
* This sketch is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public 
* License as published by the Free Software Foundation; either
* version 3 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Library General Public License for more details.
*
* You should have received a copy of the GNU Library General Public
* License along with this sketch; if not, see http://www.gnu.org/licenses
*/

/*******************************************************************************************************************
 *******************************************************************************************************************
 *** IF YOU HAVE WIRED THE BOARD FOR DRAWBRIDGE PLUS YOU MUST ENABLE/SET THIS OPTION FROM THE WINDOW APPLICATION ***
 *** OR THIS BOARD WILL NOT OPERATE CORRECTLY. ALTERNATIVELY, RUN DIAGNOSTICS, WHICH WILL DO THIS FOR YOU        ***
 *******************************************************************************************************************
 *******************************************************************************************************************/
 
/* Latest History: Last Modified: 07/06/2022
    Firmware V1.4: Merged with Pull Request #6 (Modified the behavior of the current track location on Arduino boot - paulofduarte) which also addresses issues with some drives
    Firmware V1.5: Merged with Pull Request #9 (Detect and read out HD floppy disks 1.44M by kollokollo)
    Firmware V1.6: Added experimental unbuffered writing HD disk support
    Firmware V1.7: Added suggestion from GitHub user "prickle" regarding the CHECK_SERIAL function which should reoslve issues with some of the USB to SERIAL converters
    Firmware V1.8: Added support for read streaming with index sync support which includes timing data (for perfect copy of flux data for SCP image files)
                   Changed read timings slightly which means more disks can now be recovered!
                   Added support for PRECOMP disk writing (using PWM in 'one-shot' mode) to improve readability as you go past track 40
                   Added some new functions which allow for more direct control of the drive
                   Remove the Erase function as discovered you just need to write a longer track to ensure you clear the gap.  Workbench writes 13630 bytes per track. The first part is filler at 0xAA
                   Added support for disk change notifications support (requires small hardware modification below)
                   (a) Added 'no-click' support
    Firmware V1.9.0: Improved accuracy for flux capture for SCP and *UAE
             V1.9.2: Full HD floppy disk support for DrawBridge Classic
                     Support for Drawbridge Plus hardware design with higher accuracy capture with less jitter, allows reading of really bad HD disks recorded with terrible flutter.
                     Added support for slowing down disk seek form slow drives
                     Added extra commands for accessing the EEPROM to apply settings from the applications
                     Added extra commands for calculating disk RPM
                     Added extra commands for detecting DD/HD based on either the HD pin (usually doesnt work) or by sampling data from a disk
                     Adjusted TIMING_OVERHEAD for DrawBridge Classic design for increased disk read-ability
                     Added support for PLL in the DD read code for increased reliability.
              V1.9.3 Adjusted the PLL settings for DD disks for better compatability
              V1.9.4 Fixed auto-detection of DB vs DB Plus
                     Fixed a few bugs in the HD code - silly me.  
              v1.9.5 Fixed small issue with HD mode incorrectly detecting the index pulse     
              v1.9.6 Increased time used to measure disk to test for DD vs HD
              v1.9.7 Small changes to the HD reading code
                     Fixed a problem where a disk with unknown data could cause the 'Check HD/DD' to get stuck
                     Added a timeout process for reading HD disks whilst in DB Classic without affecting jitter
              v1.9.8 Removed DD PLL code, was actually reducing performance.
              v1.9.9 Improved the DD and HD writing code, its much simpler, faster now and guaranteed 100% no jitter.  Timings are EXACT
              v1.9.10 Updated the HD read for DrawBridge Classic
              v1.9.11 Small modification to improve reading DD and HD, this is probably at its best now.
              v1.9.12 Small modification/correction of the DD and HD write timings
              v1.9.13 Halloween Edition. Small change to delay function to speed up drive stepping
              v1.9.14 Much faster seek times, makes it sound like a PC drive, but we want speed right?
              v1.9.15 Added a 'reset' function
              v1.9.16 Put the buffering back into the DD write function to support some of the dodgy FTDI devices
              v1.9.17 Added buffering to the HD write function to support some of the dodgy FTDI devices
              v1.9.18 Added support for 'flux level' writing at arbitrary intervals and a new Flux Wipe, that doesnt write any transitions and makes the disk unformatted
              v1.9.19 Added a 'terminate at index' option when writing flux
              v1.9.20 Added EEPROM setting for 'always index align writes'
              v1.9.21 Added an 'initial delay' option to the write flux command       
              v1.9.22 Changed the flux write to be more forgiving for slower USB interfaces, improved stream reading for better accuracy 
                      Improved standard reading routines for better accuracy and compatability
              v1.9.23 Added new accurate Flux level read for both classic and drawbridge Plus! This is very accurate for Plus, +/-125ns for classic written entirely in assembly language, the C version was too slow
                      Added basic pll version of read command and updated the old method.  TIMING_OVERHEAD is no longer relevant, this is too slow for DrawBridge Classic
                      Created ASM version for DrawBridge Plus! Classic gets a more accurate version but couldnt get the code stable
              v1.9.24 More Accurate PLL for both DrawBridge Classic and Plus!
              v1.9.25 Added support for tracks 82 and 83
                      
*/    

/////////////////////////////////////////////////////////////////////////////////////////////////////
// This sketch manages the interface between the floppy drive and the computer as well as the     //
// low-level disk reading and writing.  For more information and how to connect your Arduino     //
// to a floppy drive and computer visit https://amiga.robsmithdev.co.uk                         //
/////////////////////////////////////////////////////////////////////////////////////////////////
// This code doesnt actually do any decoding, and is mearly reading pulses, so can be used to //
// read data from other disk formats too.                                                    //
//////////////////////////////////////////////////////////////////////////////////////////////

// ** Hardware Modifications for DrawBridge Plus
//    Swap Pin 4 and 8 on the Arduino, and use the Windows Tool to set the 'DrawBridge Plus' mode in the EEPROM. Note in this configuration, the 1K resistor goes between pin 8 and 5V

// ** Hardware Modification Changes to get the best support for disk change notifications **
//    Pin 34 on the floppy drive connector (Disk Ready/Change) must be connected to Pin 10 on the Arduino
//    Pin 12 on the floppy drive connector (Select Disk B) must be *disconnected* from pin 5 on the Arduino and connected to Pin 11 on the Arduino.  Note you *must* leave the connection between Arduino Pin 5 and Floppy Connector 16 in place
//    On the Arduino, connect Pin 12 to GND (0v) - this enables this advanced mode automatically.  Alternativly, use the Windows Tool to set this via EEPROM

#define BAUDRATE 2000000                 // The baudrate that we want to communicate over (2M)
#define BAUD_PRESCALLER_NORMAL_MODE      (((F_CPU / (BAUDRATE * 16UL))) - 1)
#define BAUD_PRESCALLER_DOUBLESPEED_MODE (((F_CPU / (BAUDRATE * 8UL))) - 1)
#define UART_USE_DOUBLESPEED_MODE        // We're using double speed mode

#define MOTOR_TRACK_DECREASE   HIGH      // Motor directions for PIN settings
#define MOTOR_TRACK_INCREASE   LOW

// PIN 2 - INDEX PULSE PIN - used to detect a specific point on the track for sync.  Not used by standard Amiga disks but some copy protection uses it.
#define PIN_INDEX_DETECTED       2          // Pin used to detect the index pulse
#define PIN_INDEX_PORT           PIND
#define PIN_INDEX_MASK           B00000100

// PIN 3 - WRITE DATA
#define PIN_WRITE_DATA           3        // Raw triggering of writing data to the disk
#define PIN_WRITE_DATA_PORT      PORTD     // The actual port the above pin is on
#define PIN_WRITE_DATA_MASK      B00001000 // The mask used to set this pin high or low

// PIN 4 - READ DATA
#define PIN_READ_DATA            4          // Reads RAW floppy data on this pin.  In DrawBridge Plus mode, this is PIN_DETECT_TRACK_0
#define PIN_READ_DATA_MASK       B00010000  // The mask for the port
#define PIN_READ_DATA_PORT       PIND       // The port the above pin is on
#define PIN_READ_DATA_PORT_WRITE PORTD
#define PIN_READ_DATA_IO_DIR     DDRD

// PIN 8 - READ DATA (Drawbridge+)
#define PIN_READ_DATA_PLUSMODE            8          // Reads RAW floppy data on this pin.  In DrawBridge Plus mode, this is PIN_DETECT_TRACK_0
#define PIN_READ_DATA_PLUSMODE_MASK       B00000001  // The mask for the port
#define PIN_READ_DATA_PLUSMODE_PORT       PINB       // The port the above pin is on

// PIN 5, 6 and 7 - DRIVE, HEAD MOTOR DIRECTION and CONTROL
#define PIN_DRIVE_ENABLE_MOTOR   5        // Turn on and off the motor on the drive
#define PIN_MOTOR_DIR            6        // Stepper motor output to choose the direction the head should move
#define PIN_MOTOR_STEP           7        // Stepper motor step line to move the head position

// PIN 8 - Used to detect track 0 while moving the head
#define PIN_DETECT_TRACK_0       8        // Used to see if the drive is at track 0. In DrawBridge+ this is PIN_READ_DATA

// PIN 9 - HEAD SELECTION
#define PIN_HEAD_SELECT          9        // Choose upper and lower head on the drive

// PIN A0 - WRITE GATE (Floppy Write Enable)
#define PIN_WRITE_GATE           A0        // This pin enables writing to the disk
#define PIN_WRITE_GATE_PORT      PORTC    // The actual port the above pin is on
#define PIN_WRITE_GATE_MASK      B00000001 // The port pin mask for the gate

// PIN A1 - CHECK WRITE PROTECTION
#define PIN_WRITE_PROTECTED      A1         // To check if the disk is write protected

// PIN A2 - CTS Pin from UART
#define PIN_CTS                  A2         // Pin linked to the CTS pin
#define PIN_CTS_PORT             PORTC      // Port the CTS pin is on
#define PIN_CTS_MASK             B00000100  // Binary mask to control it with

// PIN A3 - Connected to the Density Select Pin on the drive.  On normal PC drives this doesnt do anything, on slimline drives it works as it should
#define PIN_HD                   A3

// Reserved for future use
#define PIN_RDY                  A4

// PIN 13 - Activity LED
#define PIN_ACTIVITY_LED         13       // Standard LED on Arduinos.  We're just using this as a read/write status flag

//  ** Hardware Modification Changes to get the best support for disk change notifications **  We control these regardless
#define PIN_DISK_CHANGE          10   // This is actually disk exists pin.  Some drives have a resistor that needs to be added/removed to get this operation.  This is usually the default on PC drives.
#define PIN_SELECT_DRIVE         11
#define PIN_DETECT_ADVANCED_MODE 12

// These are read from the EEPROM so do not modifiy them
bool advancedControllerMode     = false;   // DO NOT CHANGE THIS, its automatically detected. If you can't connect pin12 to GND because you want to use the ISP headers, then see https://amiga.robsmithdev.co.uk/isp
bool drawbridgePlusMode         = false;   // DO NOT CHANGE THIS, its automatically set via EEPROM.  See the Windows Tool for more information
bool disableDensityDetection    = false;   // DO NOT CHANGE THIS, its automatically set vis EEPROM.  See the Windows Tool for more information
bool slowerDiskSeeking          = false;   // DO NOT CHANGE THIS, its automatically set vis EEPROM.  See the Windows Tool for more information
bool alwaysIndexAlignWrites     = false;   // DO NOT CHANGE THIS, its automatically set vis EEPROM.  See the Windows Tool for more information

// Paula on the Amiga used to find the SYNC WORDS and then read 0x1900 further WORDS.  
// A dos track is 11968 bytes in size, theritical revolution is 12800 bytes. 
// The ATARI ST could format a track with up to 11 Sectors, so the AMIGA settings are OK.
#define RAW_TRACKDATA_LENGTH   (0x1900*2+0x440)  // Paula assumed it was 12868 bytes, so we read that, plus the size of a sector, to find overlap

// The current track that the head is over. Starts with -1 to identify an unknown head position.
int currentTrack = -1;

// If the drive has been switched on or not
bool driveEnabled  = 0;

/* Where there should be a HD Disk been read (1) or a DD and SD Disk (0).*/
bool disktypeHD = 0;

// The timings here could be changed.  These are based on F_CPU=16Mhz, which leaves the resolution at 1 tick = 0.0625usec, hence 16=1uSec

// There's approx 4 clock ticks on average between noticing the flux transition and the counter value being read/reset
// In DrawBridge Plus mode this is not used.
#define TIMING_OVERHEAD               -4

// Calculate the bit-timing windows.  These are the ideal exact centre of the next flux transition since the previous.
#define TIMING_DD_MIDDLE_2us     (2 * 16)
#define TIMING_DD_MIDDLE_4us     (4 * 16)
#define TIMING_DD_MIDDLE_6us     (6 * 16)
#define TIMING_DD_MIDDLE_8us     (8 * 16)

// Work out the upper window of the timing.  Most PLL allow for about 10% drift, but we're not interested in that and just want to recover the data
#define TIMING_DD_UPPER_2us     (TIMING_DD_MIDDLE_2us + 16 + TIMING_OVERHEAD) 
#define TIMING_DD_UPPER_4us     (TIMING_DD_MIDDLE_4us + 16 + TIMING_OVERHEAD) 
#define TIMING_DD_UPPER_6us     (TIMING_DD_MIDDLE_6us + 16 + TIMING_OVERHEAD) 
#define TIMING_DD_UPPER_8us     (TIMING_DD_MIDDLE_8us + 16 + TIMING_OVERHEAD) 

// HD versions - but these are just used for measure disk etc
#define TIMING_HD_UPPER_2us     ((TIMING_DD_MIDDLE_4us/2) + 8 + TIMING_OVERHEAD) 
#define TIMING_HD_UPPER_3us     ((TIMING_DD_MIDDLE_6us/2) + 8 + TIMING_OVERHEAD) 

// This isn't how a proper PLL should work, but its as close as we can get to
// The deviation from the clock centre is calculated and added over several samples.  If we're at the right rate, the jitter will cancel its self out.
// If the total reaches this number we adjust the clock speed +/- 1 (62.5us) at a time until it corrects.
// The smaller number, the faster it adapts, but the more likely general jitter could cause an issue and it not work at all!
// The larger then it may not correct quick enough.  For bad disks, the speed can actually vary during a single rotation.
// There's no science behind this number, its just a trial and error with a really bad disk I have
#define PLL_HD_THRESHOLD     48
// The default position for the PLL to start in. -8 is the best position for a 100% ideal disk
#define PLL_HD_START_VALUE   -8
// Center position reading HD disks in DB classic mode (this is TIMING_OVERHEAD+DB_CLASSIC_HD_MIDDLE) - this was checked to give the best result with jitter
#define DB_CLASSIC_HD_MIDDLE -2




// 256 byte circular buffer - don't change this, we abuse the unsigned char to overflow back to zero!
#define SERIAL_BUFFER_SIZE 256
#define SERIAL_BUFFER_START (SERIAL_BUFFER_SIZE - 16)
// Only used here
unsigned char SERIAL_BUFFER[SERIAL_BUFFER_SIZE];

#include <EEPROM.h>
#include <avr/wdt.h>

// Because we turned off interrupts delay() doesnt work! This is accurate at the millisecond level
void smalldelay(unsigned long delayTime) {
    // Use timer 0 to count the correct number of ms
    TCCR0A = 0;         // Simple counter
    TCCR0B = bit(CS01) | bit(CS00); // Prescaler of divide by 64.  So if F_CPU=16000000, 250 clock ticks occur in 1ms second

    for (unsigned long i=0; i<delayTime; i++) {
       TCNT0 = 0;             // Reset counter;
       while (TCNT0<250) {};  // wait 1ms, we could do this more accuratly, but im not bothered
    }
    TCCR0B = 0; // turn off
}

// Because we turned off interrupts delay() doesnt work! This delays in 100us intervals, ie: delaytime=10 would be 1ms
void smalldelay100us(unsigned long delayTime) {
    // Use timer 0 to count the correct number of us
    TCCR0A = 0;         // Simple counter
    TCCR0B =bit(CS01); // Prescaler of divide by 8.  So if F_CPU=16000000, 250 clock ticks occur in 1ms second
    for (unsigned long i=0; i<delayTime; i++) {
       TCNT0 = 0;              // Reset counter;
       while (TCNT0<200) {};   // wait 100us, we could do this more accuratly, but im not bothered
    }
    TCCR0B = 0; // turn off
}

// Step the head once.  This seems to be an acceptable speed for the head
// Drive spec says pulse should be at least 3ms, but the time between pulses must be greater than 1us.  16 NOPS is approx 1us, so im just being cautious
void stepDirectionHead(unsigned long delayTime100usIntervals = 35) {
    asm volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"::);
    digitalWrite(PIN_MOTOR_STEP,LOW);
    smalldelay100us(delayTime100usIntervals);    
    if (slowerDiskSeeking) smalldelay(2);
    digitalWrite(PIN_MOTOR_STEP,HIGH);
    asm volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"::);
    if (slowerDiskSeeking) smalldelay(2);
}

// Prepare serial port - We dont want to use the arduino serial library as we want to use faster speeds and no serial interrupts
void prepSerialInterface() { 
#ifdef UART_USE_DOUBLESPEED_MODE
    UBRR0H = (uint8_t)(BAUD_PRESCALLER_DOUBLESPEED_MODE>>8);
    UBRR0L = (uint8_t)(BAUD_PRESCALLER_DOUBLESPEED_MODE);
    UCSR0A |= 1<<U2X0;
#else
    UBRR0H = (uint8_t)(BAUD_PRESCALLER_NORMAL_MODE>>8);
    UBRR0L = (uint8_t)(BAUD_PRESCALLER_NORMAL_MODE);
    UCSR0A &= ~(1<<U2X0);
#endif 

    // UCSROA is a status register only (apart from U2Xn):
    //  • Bit 7 – RXCn: USART Receive Complete 
    //  • Bit 6 – TXCn: USART Transmit Complete
    //  • Bit 5 – UDREn: USART Data Register Empty
    //  • Bit 4 – FEn: Frame Error
    //  • Bit 3 – DORn: Data OverRun/
    //  • Bit 2 – UPEn: USART Parity Error
    //  • Bit 1 – U2Xn: Double the USART Transmission Speed
    //  • Bit 0 – MPCMn: Multi-processor Communication Mode

    UCSR0B  = (0<<RXCIE0)  |   // Disable ReceiveCompleteInteruptEnable
              (0<<TXCIE0)  |   // Disable TransmitCompleteInteruptEnable
              (0<<UDRIE0)  |   // Disable UsartDataRegisterEmptyInteruptEnable
              (1<<RXEN0)   |   // Enable RX
              (1<<TXEN0)   |   // Enable TX
              (0<<UCSZ02)  ;   // Clear the 9-bit character mode bit

    UCSR0C =  (0<<UMSEL01) | (0<<UMSEL00) |   // UsartModeSelect - Asynchronous (00=Async, 01=Sync, 10=Reserved, 11=Master SPI)
              (0<<UPM01)   | (0<<UPM00)   |   // UsartParatyMode - Disabled  (00=Off, 01=Reserved, 10=Even, 11=Odd)
              (0<<USBS0)   |   // UsartStopBitSelect (0=1 Stop bit, 1 = 2Stop Bits)
              (1<<UCSZ01)  | (1<<UCSZ00);    // UsartCharacterSiZe  - 8-bit (00=5Bit, 01=6Bit, 10=7Bit, 11=8Bit, must be 11 for 9-bit)
}

// Directly read a byte from the UART0 (serial)
inline byte readByteFromUART() {
    while (!( UCSR0A & ( 1 << RXC0 ))) {};    // Wait for data to be available
    return UDR0;                                 // Read it
}

// Directly write a byte to the UART0
inline void writeByteToUART(const char value) {
    while(!(UCSR0A & (1<<UDRE0))) {};                // Wait until the last byte has been sent
    UDR0 = value;                                 // And send another
}

// Main arduino setup 
void setup() {
    // Do these right away to prevent the disk being written to
    pinMode(PIN_MOTOR_DIR, OUTPUT);
    pinMode(PIN_MOTOR_STEP,OUTPUT);
    pinMode(PIN_ACTIVITY_LED,OUTPUT);
    pinMode(PIN_DRIVE_ENABLE_MOTOR, OUTPUT);
    pinMode(PIN_HEAD_SELECT, OUTPUT);
    digitalWrite(PIN_SELECT_DRIVE,HIGH);
    digitalWrite(PIN_WRITE_DATA, HIGH);
    digitalWrite(PIN_WRITE_GATE, HIGH);
    digitalWrite(PIN_SELECT_DRIVE,LOW);
    digitalWrite(PIN_DRIVE_ENABLE_MOTOR,HIGH);
    digitalWrite(PIN_HEAD_SELECT,LOW);    
    digitalWrite(PIN_MOTOR_STEP, HIGH);
    digitalWrite(PIN_ACTIVITY_LED,LOW);
    digitalWrite(PIN_SELECT_DRIVE,HIGH);

    pinMode(PIN_HD, INPUT_PULLUP);
    pinMode(PIN_WRITE_GATE,OUTPUT);
    pinMode(PIN_WRITE_DATA,OUTPUT);
    pinMode(PIN_CTS,OUTPUT);
    pinMode(PIN_SELECT_DRIVE,OUTPUT);

    pinMode(PIN_WRITE_PROTECTED, INPUT_PULLUP);
    pinMode(PIN_DETECT_TRACK_0, INPUT_PULLUP);
    pinMode(PIN_READ_DATA,INPUT_PULLUP);
    pinMode(PIN_DETECT_ADVANCED_MODE,INPUT_PULLUP);

    pinMode(PIN_INDEX_DETECTED,INPUT_PULLUP);
    pinMode(PIN_DISK_CHANGE,INPUT_PULLUP);    

    refreshEEPROMSettings();

    // Disable all interrupts - we dont want them!
    cli();
    TIMSK0=0;
    TIMSK1=0;
    TIMSK2=0;
    PCICR = 0;
    PCIFR = 0;
    PCMSK0 = 0;
    PCMSK2 = 0;
    PCMSK1 = 0;
    EIMSK&=~B00000011;  // disable INT0/1 interrupt mask
    
    // Setup the USART
    prepSerialInterface();
}

// Refresh bools after eeprom has been written to
void refreshEEPROMSettings() {
    // Has this hardware been updated? - If you can't connect pin12 to GND because you want to use the ISP headers, then see https://amiga.robsmithdev.co.uk/isp
    advancedControllerMode = digitalRead(PIN_DETECT_ADVANCED_MODE) == LOW;
    
    if (!advancedControllerMode) {
       unsigned char b1,b2,b3,b4;
       EEPROM.get(0, b1);
       EEPROM.get(1, b2);
       EEPROM.get(2, b3);
       EEPROM.get(3, b4);

       // If you can't connect pin12 to GND because you want to use the ISP headers, then see https://amiga.robsmithdev.co.uk/isp
       if ((b1 == 0x52) && (b2 == 0x6F) && (b3 == 0x62) && (b4 == 0x53)) advancedControllerMode = true;
    } 

    unsigned char b1,b2;
    EEPROM.get(4, b1);
    EEPROM.get(5, b2);
    drawbridgePlusMode = ((b1 == 0x2B) && (b2 == 0xB2));

    EEPROM.get(6, b1);
    EEPROM.get(7, b2);
    disableDensityDetection = (((b1 == 0x44) && (b2 == 0x53)));

    EEPROM.get(8, b1);
    EEPROM.get(9, b2);
    slowerDiskSeeking = (((b1 == 0x53) && (b2 == 0x77)));   

    EEPROM.get(10, b1);
    EEPROM.get(11, b2);
    alwaysIndexAlignWrites = (((b1 == 0x69) && (b2 == 0x61)));   
     
}

// EEPROM access
void readEpromValue() {
  writeByteToUART('1');
  unsigned char index = readByteFromUART();
  unsigned char value;
  EEPROM.get(index, value);
  writeByteToUART(value);
}

void writeEpromValue() {
  writeByteToUART('1');
  unsigned char index = readByteFromUART();
  unsigned char value = readByteFromUART();
  EEPROM.put(index, value);
  writeByteToUART('1');

  // Force settings reload
  refreshEEPROMSettings();
}

// Run a diagnostics test command - Updated for Drawbridge Plus
void runDiagnostic() {
    // See what test to run
    byte test = readByteFromUART();

    switch (test) {
        case '1':  // Turn off CTS
            PIN_CTS_PORT &= (~PIN_CTS_MASK);   
            writeByteToUART('1');
            readByteFromUART();
            writeByteToUART('1');
            break;

        case '2':  // Turn on CTS
            PIN_CTS_PORT|=PIN_CTS_MASK;   
            writeByteToUART('1');
            readByteFromUART();
            writeByteToUART('1');
            break;

        case '3':  // Index pulse test (with timeout)
            {
               bool state1 = false;
               bool state2 = false;

               // At the 300 RPM (5 turns per second) this runs at, this loop needs to run a few times to check for index pulses.  This runs for approx 1 second
               for (unsigned int b=0; b<20; b++) {
                 for (unsigned int a=0; a<60000; a++) {
                     if (PIN_INDEX_PORT & PIN_INDEX_MASK) state1=true; else state2=true;
                     if (state1&&state2) break;
                 }
                 if (state1&&state2) break;
               }

               if (state1&&state2) {
                   writeByteToUART('1');
               } else {
                   writeByteToUART('0');
               }
            }
            break;   

        case '4':  // Data pulse test (with timeout)
            {
               bool state1 = false;
               bool state2 = false;

              if (drawbridgePlusMode) {
                for (unsigned int b=0; b<20; b++) {
                   for (unsigned int a=0; a<60000; a++) {
                        if (PIN_READ_DATA_PLUSMODE_PORT & PIN_READ_DATA_PLUSMODE_MASK) state1=true; else state2=true;
                        if (state1&&state2) break;
                    }
                    if (state1&&state2) break;
                }
              } else {
                for (unsigned int b=0; b<20; b++) {
                   for (unsigned int a=0; a<60000; a++) {
                        if (PIN_READ_DATA_PORT & PIN_READ_DATA_MASK) state1=true; else state2=true;
                        if (state1&&state2) break;
                    }
  
                    if (state1&&state2) break;
                }
              }
              

              if (state1&&state2) {
                  writeByteToUART('1');
              } else {
                  writeByteToUART('0');
              }
            }
            break;  

        case '5':  // speed test
          {
            writeByteToUART('1');
            smalldelay(200);
            for (unsigned char loops = 0; loops <= 10; loops++)
              for (unsigned int value = 0; value<=255; value++) {
                writeByteToUART(value);
              }        
          }
          break;

       case '6':  // help identify drawbridge PLUS
          {
              unsigned int stdCounter = 0;
              unsigned int plusCounter = 0;
              bool stdLastState = false;
              bool plusLastState = false;
              
              for (unsigned int b=0; b<5; b++) {
                 for (unsigned int a=0; a<60000; a++) {
                      bool state = (PIN_READ_DATA_PLUSMODE_PORT & PIN_READ_DATA_PLUSMODE_MASK);
                      if (state != plusLastState) {
                        plusLastState = state;
                        if (plusCounter<0xFFFF) plusCounter++;
                      }
                      
                      state = (PIN_READ_DATA_PORT & PIN_READ_DATA_MASK);
                      if (state != stdLastState) {
                        stdLastState = state;
                        if (stdCounter<0xFFFF) stdCounter++;
                      }
                  }
              }

              if (stdCounter<plusCounter) {
                  writeByteToUART('1');
              } else {
                  writeByteToUART('0');
              }
          }
          break;
              
      default:
        writeByteToUART('0');
        break;
    }
}

// This is used for better compatability with the non-modded hardware
void startDriveForOperation() {
  if (advancedControllerMode) return;
  if (driveEnabled) return;
  digitalWrite(PIN_DRIVE_ENABLE_MOTOR,LOW);
  smalldelay(10);
}
void stopDriveForOperation() {
  if (advancedControllerMode) return;
  if (driveEnabled) return;
  digitalWrite(PIN_DRIVE_ENABLE_MOTOR,HIGH);
  smalldelay(1);
}

// Rewinds the head back to Track 0 - Updated for Drawbridge Plus
bool goToTrack0() {
    startDriveForOperation();
    digitalWrite(PIN_MOTOR_DIR,MOTOR_TRACK_DECREASE);   // Set the direction to go backwards
    int counter=0;
    unsigned char portPin = drawbridgePlusMode ? PIN_READ_DATA : PIN_DETECT_TRACK_0;
    while (digitalRead(portPin) != LOW) {
       stepDirectionHead(45);   // Keep moving the head until we see the TRACK 0 detection pin
       counter++;
       // If this happens we;ve steps twice as many as needed and still havent found track 0
       if (counter>170) {
          stopDriveForOperation();
          return false;
       }
    }

    stopDriveForOperation();
    
    currentTrack = 0;    // Reset the track number
    return true;
}

// Handle a no-click seeking operation
void handleNoClickSeek() {
  if (currentTrack != 0) {
     // Not allowed.
     writeByteToUART('0');
     return;
  }

  startDriveForOperation();

  digitalWrite(PIN_MOTOR_DIR,MOTOR_TRACK_DECREASE);   // Move OUT
  stepDirectionHead(45);

  writeByteToUART('1');

   // Now see if there is a disk in the drive.  Returning '#' means no disk in drive
   if (advancedControllerMode) {     
       if (digitalRead(PIN_DISK_CHANGE) == HIGH) writeByteToUART('1'); else writeByteToUART('#');  
   } else {
       // Don't detect disk
       writeByteToUART('x');      
   }
   if (digitalRead(PIN_WRITE_PROTECTED) == LOW) writeByteToUART('1'); else writeByteToUART('#');  

   stopDriveForOperation();
}
        
// Goto to a specific track.  During testing it was easier for the track number to be supplied as two ASCII characters, so I left it like this
bool gotoTrackX(bool reportDiskChange) {
    // Read the bytes
    byte track1 = readByteFromUART();
    byte track2 = readByteFromUART();
    byte flags = 1;  // default to normal speed

    if (reportDiskChange) {
      flags = readByteFromUART()-'0';      
    }
    
    // Handle speed of seek
    unsigned long delayTime;
    switch (flags & 3) {
      case 0: delayTime = 30; break;   // very fast
      case 1: delayTime = 32; break;   // fast
      case 2: delayTime = 35; break;   // normal
      case 3: delayTime = 45; break;   // slow
    }

    // Validate
    if ((track1<'0') || (track1>'9')) return false;
    if ((track2<'0') || (track2>'9')) return false;

    // Calculate target track and validate 
    int track = ((track1-'0')*10) + (track2-'0');
    if (track<0) return false;
    if (track>83) return false; // yes amiga could read track 83!

    // Exit if its already been reached
    if (track == currentTrack) {
      if (reportDiskChange) writeByteToUART('2');
      return true;
    }

    // If current track is unknown go to track 0 first
    if (currentTrack == -1) goToTrack0();

    if (reportDiskChange) writeByteToUART('1');

    startDriveForOperation();

    // And step the head until we reach this track number
    if (currentTrack < track) {
        digitalWrite(PIN_MOTOR_DIR,MOTOR_TRACK_INCREASE);   // Move IN
        while (currentTrack < track) {
            stepDirectionHead(delayTime);
            currentTrack++;         
        }
    } else {
        digitalWrite(PIN_MOTOR_DIR,MOTOR_TRACK_DECREASE);   // Move OUT
        while (currentTrack > track) {
            stepDirectionHead(delayTime);
            currentTrack--;
        }
    }

    if (reportDiskChange) {
        // Now see if there is a disk in the drive.  Returning '#' means no disk in drive
       if (advancedControllerMode) {     
         if (digitalRead(PIN_DISK_CHANGE) == HIGH) writeByteToUART('1'); else writeByteToUART('#');  
       } else {
         if (flags & 4) {
            // We've been told to check for disk presence regardless
            if (nonModCheckForDisk()) writeByteToUART('1'); else writeByteToUART('#');  
         } else {
            // Don't detect disk
            writeByteToUART('x');      
         }
       }
       // The second byte is '1' for write protected and '#' for not write protected
       if (digitalRead(PIN_WRITE_PROTECTED) == LOW) writeByteToUART('1'); else writeByteToUART('#');  
    }

    stopDriveForOperation();

    return true;
}

// Checks manually to see if theres a disk on un-modded hardware. - Updated for Drawbridge Plus
bool nonModCheckForDisk() {
    register unsigned char lastState = PIN_READ_DATA_PORT & PIN_READ_DATA_MASK;
    const unsigned char indexPinStatus = PIN_INDEX_PORT & PIN_INDEX_MASK;

    // Configure timer 2 just as a counter in NORMAL mode, we need rto measure approx 200ms (a full rotation) before giving up
    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20) | bit(CS21) | bit(CS22);       // Precale = 1024, ie: divide the clock timer by 1024, meaning each 'count' is approx 0.064ms
    OCR2A = 0x00;
    OCR2B = 0x00;

    // So if the disk wasnt spinning, we allow longer
    const unsigned char totalLoops = 18 + (!driveEnabled ? 62 : 0);
    unsigned char counter = 0;

    // We could do this with timer 1, but hey.  
    for (unsigned int loops=0; loops<totalLoops; loops++) {
      TCNT2 = 0;

      // Allow this inner loop to run for approx 12ms
      while (TCNT2<=188) {
        register unsigned char currentState = drawbridgePlusMode ? (PIN_READ_DATA_PLUSMODE_PORT & PIN_READ_DATA_PLUSMODE_MASK) : (PIN_READ_DATA_PORT & PIN_READ_DATA_MASK);

        // See if the index pulse was detected.  
        if ((PIN_INDEX_PORT & PIN_INDEX_MASK) != indexPinStatus) {
          counter=10;  
          currentState = ~lastState;
        }

        // Check if data was discovered
        if (currentState != lastState) {
          lastState = currentState;
          if (++counter>=3) {
            TCCR2A = 0;              // disable and reset everything
            TCCR2B = 0;              // Stop timer 2     
            return true;
          }
        }
      }
    }

    TCCR2A = 0;              // disable and reset everything
    TCCR2B = 0;              // Stop timer 2     

    return false;
}

// Test if theres a disk in the drive, a '1' if yes, a '#' if not
bool testForDisk(bool sendOutput) {
  if (advancedControllerMode) {
    bool isDisk = digitalRead(PIN_DISK_CHANGE) == HIGH;

    if (!isDisk) {
      if (currentTrack < 40) {
        digitalWrite(PIN_MOTOR_DIR,MOTOR_TRACK_INCREASE);
        stepDirectionHead();
        digitalWrite(PIN_MOTOR_DIR,MOTOR_TRACK_DECREASE);
        stepDirectionHead();
      } else {
        digitalWrite(PIN_MOTOR_DIR,MOTOR_TRACK_DECREASE);
        stepDirectionHead();
        digitalWrite(PIN_MOTOR_DIR,MOTOR_TRACK_INCREASE);
        stepDirectionHead();    
      }
      isDisk = digitalRead(PIN_DISK_CHANGE) == HIGH;
    }
    if (sendOutput) {
      if (isDisk) writeByteToUART('1'); else writeByteToUART('#');
      if (digitalRead(PIN_WRITE_PROTECTED) == LOW) writeByteToUART('1'); else writeByteToUART('#');  
    }
    return isDisk;    
  } else {
    // This is much harder
    startDriveForOperation();

    bool diskFound = nonModCheckForDisk();


    if (sendOutput) {      
      if (diskFound) writeByteToUART('1'); else writeByteToUART('#');
      if (digitalRead(PIN_WRITE_PROTECTED) == LOW) writeByteToUART('1'); else writeByteToUART('#');  
    } 
    
    stopDriveForOperation();
    return diskFound;
  }
}

// Check if the disk is write protected.  Sends '#' if its write protected, or '1' if its not.  If theres no disk in the drive this number is meaningless
void checkWriteProtectStatus() {
  startDriveForOperation();
  if (digitalRead(PIN_WRITE_PROTECTED) == LOW) {
        // Drive is write protected
        writeByteToUART('1');
  } else {
        // Drive can be written to
        writeByteToUART('#');
  }                
  stopDriveForOperation();
}


// This runs the PWM for writing a single pulse
#define RUN_PULSE_DD()                                                                                 \
        while (!(TIFR2 &  bit(TOV2))) {                                                                \
          if (UCSR0A & bit(RXC0)) {                                                                    \
             PIN_CTS_PORT|=PIN_CTS_MASK;                                                               \
             SERIAL_BUFFER[serialWritePos++] = UDR0;                                                   \
             serialBytesInUse++;                                                                       \
          }                                                                                            \
        };                                                                                             \
        OCR2A = counter;    /* reset to zero when we get to counter */                                 \
        OCR2B = pulseStart;                                                                            \
        TIFR2 |= bit(TOV2);        

// Write a track to disk from the UART - This works like the precomp version as the old method isn't fast enough.  This version is 100% jitter free
void writePrecompTrack() {
    // Check if its write protected.  You can only do this after the write gate has been pulled low
    if (digitalRead(PIN_WRITE_PROTECTED) == LOW) {
        writeByteToUART('N'); 
        return;
    } else
    writeByteToUART('Y');

    unsigned char highByte = readByteFromUART();
    unsigned char lowByte = readByteFromUART();
    unsigned char waitForIndex = readByteFromUART();
    unsigned short numBytes = (((unsigned short)highByte)<<8) | lowByte;
    
    unsigned char serialReadPos = 0;
    unsigned char serialWritePos = SERIAL_BUFFER_START;
    unsigned char serialBytesInUse = SERIAL_BUFFER_START;
    
    writeByteToUART('!');     
    digitalWrite(PIN_ACTIVITY_LED,HIGH);
    register unsigned char currentByte = readByteFromUART();

    // Fill our buffer to give us a head start
    for (int a=0; a<SERIAL_BUFFER_START; a++) {
        // Wait for it
        while (!( UCSR0A & ( 1 << RXC0 ))){}; 
        // Save the byte
        SERIAL_BUFFER[a] = UDR0;
    }  

    numBytes--;

    PIN_CTS_PORT|=PIN_CTS_MASK;                // stop any more data coming in!

    // Reset the counter, ready for writing - abuse the Fast PWM to produce the wayforms we need.  All we do is change the value it resets at!
    TCCR2A = bit(COM2B1) | bit(WGM20) | bit(WGM21);  // (COM2B0|COM2B1) Clear OC2B. on compare match, set OC2B at BOTTOM.  WGM20|WGM21 is Fast PWM. 
    TCCR2B = bit(WGM22)| bit(CS20);         // WGM22 enables waveform generation.  CS20 starts the counter runing at maximum speed

    // Enable writing
    PIN_WRITE_DATA_PORT|=PIN_WRITE_DATA_MASK;
    
    // While the INDEX pin is high wait.  Might as well write from the start of the track
    if (waitForIndex || alwaysIndexAlignWrites) 
        while (PIN_INDEX_PORT & PIN_INDEX_MASK)  {};

    PIN_WRITE_GATE_PORT&=~PIN_WRITE_GATE_MASK;

    // Get it ready    
    OCR2A = 64;      // Just setup something to get this going        
    OCR2B = 62;         
    TCNT2 = 0;      
    TIFR2 |= bit(TOV2) | bit(OCF2B);

    // Loop through all bytes of data required.  Each byte contains two sequences to write
    do {
        // Group 1
        register unsigned char counter = 63 + ((currentByte&0x03) * 32);
        register unsigned char pulseStart = counter - 5;
        if (currentByte & 0x04) pulseStart-=2;    // Pulse should be early, so just move the pulse start back
        if (currentByte & 0x08) pulseStart+=2;    // Pulse should be late, so move the pulse start forward
        
        RUN_PULSE_DD();
        if (serialBytesInUse<SERIAL_BUFFER_START) PIN_CTS_PORT &= (~PIN_CTS_MASK); else PIN_CTS_PORT|=PIN_CTS_MASK;    
              
        numBytes--;
        // Check for overflow errors
        if (UCSR0A & (bit(FE0)|bit(DOR0))) break;

        // Group 2
        counter = 63 + ((currentByte&0x30) * 2);
        pulseStart = counter - 5;
        if (currentByte & 0x40) pulseStart-=2;    // Pulse should be early, so just move the pulse start back
        if (currentByte & 0x80) pulseStart+=2;    // Pulse should be late, so move the pulse start forward
        RUN_PULSE_DD();

        if (!serialBytesInUse) break;
        
        currentByte = SERIAL_BUFFER[serialReadPos++]; 
        serialBytesInUse--;
    } while (numBytes);    

    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;
    // Turn off the write head
    PIN_WRITE_DATA_PORT|=PIN_WRITE_DATA_MASK;

    TCCR2A = 0;              // disable and reset everything
    TCCR2B = 0;              // Stop timer 2     

    // Data wouldnt have been read quick enough
    if (numBytes) {
        if (UCSR0A & bit(FE0)) {
           writeByteToUART('Y');   // Serial Framing Error
        } else
        if (UCSR0A & bit(DOR0)) {
            writeByteToUART('Z');   // Serial IO overrun
        } else {
            writeByteToUART('X');   // Serial data not received quickly enough
        }
    } else {
      // Done!
      writeByteToUART('1');      
    }

    digitalWrite(PIN_ACTIVITY_LED,LOW);
    PIN_CTS_PORT &= (~PIN_CTS_MASK);     
}

#define FLUX_OFFSET        (44-1)     // The -1 is important as the counters count up to and including the number
#define FLUX_MULTIPLIER    2
#define FLUX_NOFLUX_OFFSET 5
#define FLUX_SPECIAL_CODE_BLANK 30    // This special code causes DB to skip 3125ns of time without a flux transition.
#define FLUX_SPECIAL_CODE_END   31    // This special code causes the writing to finish

#define prepareFlux(v)                                                        \
  if (EIFR&bit(INTF0)) {                                                      \
    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;                                 \
    EIFR = bit(INTF0);                                                        \
    indexTerminated = true;                                                   \
  }                                                                           \
  timeValue = v;                                                              \
  if (timeValue == FLUX_SPECIAL_CODE_END) {                                   \ 
    while (!(TIFR2 &  bit(TOV2))) {};    /* Wait until the pulse finishes */  \ 
    completedOK = true;                  /* Set success */                    \ 
    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK; /* stop writing */              \
  }                                                                           \
                                                                              \
  if (timeValue == FLUX_SPECIAL_CODE_BLANK) {                                 \ 
    timeValue = (FLUX_NOFLUX_OFFSET * FLUX_MULTIPLIER) + FLUX_OFFSET;         \
    osr2b =  0xFF;                                                            \ 
  } else {                                                                    \
    timeValue = (timeValue * FLUX_MULTIPLIER) + FLUX_OFFSET;                  \ 
    osr2b = timeValue - 2;                                                    \
  }                                                                           \
                                                                              \
  while (!(TIFR2 &  bit(TOV2))) { /* Wait for the previous timer */           \ 
    if (UCSR0A & bit(RXC0)) {     /* Incoming Data */                         \
      SERIAL_BUFFER[serialWritePos++] = UDR0;                                 \
      serialBytesInUse++;                                                     \
      if (serialBytesInUse>=SERIAL_BUFFER_START) PIN_CTS_PORT|=PIN_CTS_MASK;  \ 
    }                                                                         \
  };                                                                          \
  OCR2A = timeValue,                                                          \
  OCR2B = osr2b,                                                              \
  TIFR2 |= bit(TOV2);

#define prepareFluxFirstTime(v)                                               \
  if (v == FLUX_SPECIAL_CODE_BLANK) {                                         \ 
    timeValue = (FLUX_NOFLUX_OFFSET * FLUX_MULTIPLIER) + FLUX_OFFSET;         \
    osr2b = 0xFF;                /* never reached, so no pulse generated */   \ 
  } else {                                                                    \
    timeValue = (v * FLUX_MULTIPLIER) + FLUX_OFFSET;                          \ 
    osr2b = timeValue - 2;                                                    \
  }                                                                           \
  OCR2A = timeValue;                                                          \
  OCR2B = osr2b;                                                              \
  TIFR2 |= bit(TOV2);                                                         \
  TCNT2 = 0;                                              

#define GetSerialDataWithWait(variable)                                           \
     if (serialBytesInUse<1) {                                                    \
        PIN_CTS_PORT&=~PIN_CTS_MASK;                                              \
        while (!(UCSR0A & bit(RXC0))) {};                                         \
        PIN_CTS_PORT|=PIN_CTS_MASK;                                               \
        while (UCSR0A & bit(RXC0)) {                                             \
          SERIAL_BUFFER[serialWritePos++] = UDR0;                                 \
          serialBytesInUse++;                                                     \
        }                                                                         \
      }                                                                           \
      serialBytesInUse--;                                                         \
      variable = SERIAL_BUFFER[serialReadPos++];                                  \

#define breakIfZero(c)                                                            \
    if ((c) == FLUX_SPECIAL_CODE_END) {                                           \
      completedOK = true;                                                         \
      break;                                                                      \
    }

// Attempt to write a track in 'flux' mode
void writeFluxTrack() {
    // Check if its write protected.  
    if (digitalRead(PIN_WRITE_PROTECTED) == LOW) {
        writeByteToUART('N'); 
        return;
    } else
    writeByteToUART('Y');

    // read what the delay is, in arduino clock ticks
    unsigned long initialDelay = 0;
    {
      unsigned long b1 = readByteFromUART();
      unsigned long b2 = readByteFromUART();
      unsigned long b3 = readByteFromUART();
      initialDelay = b1 | (b2 << 8UL) | (b3 << 16UL);
    }

    // Find out any flags that were specified
    unsigned char flags = readByteFromUART();
    bool terminateAtIndex = (flags&1) != 0;
    bool indexTerminated = false;
    
    unsigned char firstFlux = readByteFromUART();
    unsigned char nextFlux;  
    unsigned char timeValue; 
    unsigned char osr2b;
    bool completedOK = false;

    // This is *always* written from INDEX
    // Fill our boots!
    for (int a=0; a<SERIAL_BUFFER_START; a++) {
        // Wait for it
        while (!( UCSR0A & ( 1 << RXC0 ))){}; 
        // Save the byte
        SERIAL_BUFFER[a] = UDR0;
    }

    // Stop more bytes coming in, although we expect one more
    PIN_CTS_PORT|=PIN_CTS_MASK; 

    unsigned char serialReadPos = 0;
    unsigned char serialWritePos = SERIAL_BUFFER_START;
    unsigned char serialBytesInUse = SERIAL_BUFFER_START;

    // Reset the counter, ready for writing - abuse the Fast PWM to produce the wayforms we need.  All we do is change the value it resets at!
    TCCR2A = bit(COM2B1) | bit(WGM20) | bit(WGM21);  // (COM2B0|COM2B1) Clear OC2B. on compare match, set OC2B at BOTTOM.  WGM20|WGM21 is Fast PWM. 
    TCCR2B = 0;

    // Enable writing.  This will start the erase head running
    PIN_WRITE_GATE_PORT&=~PIN_WRITE_GATE_MASK;

    EICRA = bit(ISC01);       // falling edge of INT0 generates an interrupt, they are turned off, but its an easy way for us to detect a falling edge rather than monitoring a pin

    // Setup the first flux, rather than decoding, its not packed in any way
    digitalWrite(PIN_ACTIVITY_LED,HIGH);
    prepareFluxFirstTime(firstFlux);

    if (initialDelay>0) {
         // Program timer 0 to use as a delay
        TCCR0A = bit(WGM00) | bit(WGM01);   // Fast PWM again, but with no output
        TCCR0B = bit(CS00); // Prescaler of divide by 1.  So max speed
        OCR0A = 255;
        OCR0B = 128;  // dont care about this

        // Wait for the INDEX pulse
        EIFR=bit(INTF0);  // clear the register saying it detected an index pulse
        while (!(EIFR&bit(INTF0))) {};
        TCNT0=0;   // Reset count
        TIFR0 |= bit(TOV0); // Clear overflow register
        EIFR=bit(INTF0);  // clear the register saying it detected an index pulse
        
        // We now need to pause for intialDelay clock ticks before continuing.  This is a bit rough but should be ok
        while (initialDelay>255) {
            // Wait for overflow
            while (!(TIFR0 &  bit(TOV0))) {};
            // Reset overflow
            TIFR0 |= bit(TOV0);
            initialDelay-=256;
        }
        if (initialDelay>16) {
          // Should now be less than 256
          OCR0A = initialDelay;
          while (!(TIFR0 &  bit(TOV0))) {};
        }
        // Start the counter
        TCCR2B = bit(WGM22)| bit(CS20);                  // WGM22 enables waveform generation.  CS20 starts the counter running at maximum speed
        // stop this one
        TCCR0A=0;
        TCCR0B=0;        
    } else {
        EIFR=bit(INTF0);  // clear the register saying it detected an index pulse
        
        // Wait for the INDEX pulse
        while (!(EIFR&bit(INTF0))) {};
        // Start the counter
        TCCR2B = bit(WGM22)| bit(CS20);                  // WGM22 enables waveform generation.  CS20 starts the counter running at maximum speed
        // clear the register saying it detected an index pulse
        EIFR=bit(INTF0);  
    }
    if (!terminateAtIndex) EICRA = 0;   // disable if we're not using terminate at index
    
    /*
     * Data is in the following layout, this means to read any 5-bits we only need to perform at most, a bit-test, a SWAP, AND and an OR:
     *     Bit:  7   6   5   4   3   2   1   0  variable
     * Byte 1 : D4  C4  B4  A4  A3  A2  A1  A0  first
     * Byte 2 : C3  C2  C1  C0  B3  B2  B1  B0  next
     * Byte 3 : E3  E2  E1  E0  D3  D2  D1  D0  next
     * Byte 4 : E4  H4  G4  F4  F3  F2  F1  F0  first
     * Byte 5 : H3  H2  H1  H0  G3  G2  G1  G0  next
     */

    
    // and write it
    do {

       // Read in byte 1 of 5
       if (!serialBytesInUse) break; 
       firstFlux = SERIAL_BUFFER[serialReadPos++], serialBytesInUse--;

       // Calculate 
       prepareFlux(firstFlux & B00011111);                                          // A

       // Read in byte 2 of 5
       if (!serialBytesInUse) break; 
       nextFlux = SERIAL_BUFFER[serialReadPos++], serialBytesInUse--;

       // Second pulse
       prepareFlux(((firstFlux & B00100000)?B00010000:0) | (nextFlux & 0x0F));      // B

       // Handle overflow errors
       if (UCSR0A & (bit(FE0)|bit(DOR0))) break;
       // Re-enable reading
       if (serialBytesInUse<SERIAL_BUFFER_START) PIN_CTS_PORT&=~PIN_CTS_MASK; 

       // Third Pulse
       prepareFlux(((firstFlux & B01000000)?B00010000:0) | (nextFlux >> 4));        // C - the >>4 will be a SWAP + AND operation

        // Read in byte 3 of 5
       if (!serialBytesInUse) break; 
       nextFlux = SERIAL_BUFFER[serialReadPos++], serialBytesInUse--;

       // Forth pulse
       prepareFlux(((firstFlux & B10000000)?B00010000:0) | (nextFlux & 0x0F));      // D     

       // Read in byte 4 of 5
       if (!serialBytesInUse) break; 
       firstFlux = SERIAL_BUFFER[serialReadPos++], serialBytesInUse--;

       // Fifth Pulse
       prepareFlux(((firstFlux & B10000000)?B00010000:0) | (nextFlux >> 4));        // E - the >>4 will be a SWAP + AND operation

       // Re-enable reading
       if (serialBytesInUse<SERIAL_BUFFER_START) PIN_CTS_PORT&=~PIN_CTS_MASK; 

       // Sixth Pulse
       prepareFlux(firstFlux & B00011111);                                          // F

       // Read in byte 5 of 5
       if (!serialBytesInUse) break; 
       nextFlux = SERIAL_BUFFER[serialReadPos++], serialBytesInUse--;       

       // Seventh Pulse
       prepareFlux(((firstFlux & B00100000)?B00010000:0) | (nextFlux & 0x0F));         // G     

       // Re-enable reading
       if (serialBytesInUse<SERIAL_BUFFER_START) PIN_CTS_PORT&=~PIN_CTS_MASK; 

       // Eighth  Pulse
       prepareFlux(((firstFlux & B01000000)?B00010000:0) | (nextFlux >> 4));           // H
    } while (!completedOK);
    
    // Disable write gate!
    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;
    // Turn off the write head
    PIN_WRITE_DATA_PORT|=PIN_WRITE_DATA_MASK;    
    TCCR2A = 0;              // disable and reset everything
    TCCR2B = 0;              // Stop timer 2 

    // Check what happened
    if (!completedOK) {
        if (UCSR0A & bit(FE0)) {
           writeByteToUART('Y');   // Serial Framing Error
        } else
        if (UCSR0A & bit(DOR0)) {
            writeByteToUART('Z');   // Serial IO overrun
        } else {
            writeByteToUART('X');   // Serial data not received quickly enough
        }
    } else {
      // Done!
      if (indexTerminated) writeByteToUART('I'); else writeByteToUART('1');      
    }

    EICRA = 0;
    digitalWrite(PIN_ACTIVITY_LED,LOW);
    PIN_CTS_PORT &= (~PIN_CTS_MASK);
}


// Lets the disk rotate while no flux transitions are written.  Creates a totally unformatted disk
void fluxWipe() {    
    // Check if its write protected.  
    if (digitalRead(PIN_WRITE_PROTECTED) == LOW) {
        writeByteToUART('N'); 
        return;
    } else writeByteToUART('Y');
    
    digitalWrite(PIN_ACTIVITY_LED,HIGH);
   
    // Enable writing
    PIN_WRITE_GATE_PORT&=~PIN_WRITE_GATE_MASK;

    // To write the 01010101 sequence we're going to ask the Arduino to generate this from its PWM output.
    TCCR2A = bit(WGM20) | bit(WGM21);       // WGM20|WGM21 is Fast PWM. 
    TCCR2B = bit(WGM22)| bit(CS20);         // WGM22 enables waveform generation.  CS20 starts the counter runing at maximum speed
    OCR2A = 63;                         
    OCR2B = 47;                         
    TCNT2=0;  

    // Now just count how many times this happens.  Approx 200ms is a revolution, so we'll go 200ms + 5% to be on the safe side (210ms)
    TIFR2 |= bit(TOV2);

    // 52500 is 210 / 0.004
    for (unsigned int counter=0; counter<60000; counter++) {
      // Every time this loop completes, 4us have passed.
      while (!(TIFR2 & bit(TOV2))) {};
      TIFR2 |= bit(TOV2);        
    };

    // Turn off the write head to stop writing instantly
    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;

    TCCR2A = 0;
    TCCR2B = 0;   // No Clock (turn off)    
  
    // Done!
    writeByteToUART('1');
    digitalWrite(PIN_ACTIVITY_LED,LOW);
}


#define CHECK_SERIAL_PART1()    if (UCSR0A & ( 1 << RXC0 )) {                      \
                                    SERIAL_BUFFER[serialWritePos++] = UDR0;        \
                                    serialBytesInUse++;                            \
                                }                                                  

#define CHECK_SERIAL_PART2()    if (serialBytesInUse<SERIAL_BUFFER_START)          \
                                    PIN_CTS_PORT &= (~PIN_CTS_MASK);               \
                                else PIN_CTS_PORT|=PIN_CTS_MASK;                        

#define CHECK_SERIAL()        CHECK_SERIAL_PART1(); CHECK_SERIAL_PART2();                                                
 
// Small Macro to write a '1' pulse to the drive if a bit is set based on the supplied bitmask
#define WRITE_BIT(value,bitmask) if (currentByte & bitmask) {                            \
                                     while (TCNT2<value) {};                             \
                                     PIN_WRITE_DATA_PORT&=~PIN_WRITE_DATA_MASK;          \
                                 } else {                                                \
                                     while (TCNT2<value) {};                             \
                                     PIN_WRITE_DATA_PORT|=PIN_WRITE_DATA_MASK;           \
                                 }                                                       
 
// Write a track to disk from the UART - the data should be pre-MFM encoded raw track data where '1's are the pulses/phase reversals to trigger - this is old, should use the PRECOMP version now for better quality
void writeTrackFromUART() {

    // Check if its write protected. 
    if (digitalRead(PIN_WRITE_PROTECTED) == LOW) {
        writeByteToUART('N'); 
        return;
    } else
    writeByteToUART('Y');

    // Configure timer 2 just as a counter in NORMAL mode
    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20);       // Precale = 1  
    OCR2A = 0x00;
    OCR2B = 0x00;
    
    // Find out how many bytes they want to send
    unsigned char highByte = readByteFromUART();
    unsigned char lowByte = readByteFromUART();
    unsigned char waitForIndex = readByteFromUART();
    PIN_CTS_PORT|=PIN_CTS_MASK;                // stop any more data coming in!
    
    unsigned short numBytes = (((unsigned short)highByte)<<8) | lowByte;

    writeByteToUART('!');
    
    register unsigned char currentByte;

    // Signal we're ready for another byte to come
    PIN_CTS_PORT &= (~PIN_CTS_MASK);   

    // Fill our buffer to give us a head start
    for (int a=0; a<SERIAL_BUFFER_START; a++) {
        // Wait for it
        while (!( UCSR0A & ( 1 << RXC0 ))){}; 
        // Save the byte
        SERIAL_BUFFER[a] = UDR0;
    }

    // Stop more bytes coming in, although we expect one more
    PIN_CTS_PORT|=PIN_CTS_MASK; 

    // Setup buffer parameters
    unsigned char serialReadPos = 0;
    unsigned char serialWritePos = SERIAL_BUFFER_START;
    unsigned char serialBytesInUse = SERIAL_BUFFER_START;
    digitalWrite(PIN_ACTIVITY_LED,HIGH);
    // Enable writing
    PIN_WRITE_DATA_PORT|=PIN_WRITE_DATA_MASK;

    // While the INDEX pin is high wait.  Might as well write from the start of the track
    if (waitForIndex || alwaysIndexAlignWrites) 
        while (PIN_INDEX_PORT & PIN_INDEX_MASK)  {};
    PIN_WRITE_GATE_PORT&=~PIN_WRITE_GATE_MASK;

    // Reset the counter, ready for writing
    TCNT2=0;  
    
    // Loop them bytes - ideally I wanted to use an ISR here, but theres just too much overhead even with naked ISRs to do this (with preserving registers etc)
    for (register unsigned int a=0; a<numBytes; a++) {
        // Should never happen, but we'll wait here if theres no data 
        if (serialBytesInUse<1) {
            // This can't happen and causes a write failure
            digitalWrite(PIN_ACTIVITY_LED,LOW);
            writeByteToUART('X');   // Thus means buffer underflow. PC wasn't sending us data fast enough
            PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;
            // Need to allow data to come in again
            PIN_CTS_PORT &= (~PIN_CTS_MASK);              
            TCCR2B = 0;   // No Clock (turn off)                
            return;
        }

        // Read a byte from the buffer
        currentByte = SERIAL_BUFFER[serialReadPos++]; 
        serialBytesInUse--;

        // Theres a small possibility, looking at the decompiled ASM (and less likely even with these few extra instructions) we actually might get back here before the TCNT2 overflows back to zero causing this to write early
        while (TCNT2>=240) {}        
    
        // Now we write the data.  Hopefully by the time we get back to the top everything is ready again. 
        WRITE_BIT(0x10,B10000000);
        CHECK_SERIAL();
        WRITE_BIT(0x30,B01000000);
        CHECK_SERIAL();
        WRITE_BIT(0x50,B00100000);
        CHECK_SERIAL();
        WRITE_BIT(0x70,B00010000);
        // Extra check for some of the other errors that can occur
        if (UCSR0A & (bit(FE0)|bit(DOR0))) {
            // This can't happen and causes a write failure
            digitalWrite(PIN_ACTIVITY_LED,LOW);
            writeByteToUART((UCSR0A & bit(FE0)) ? 'Y' : 'Z');   // Thus means buffer underflow. PC wasn't sending us data fast enough
            PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;
            // Need to allow data to come in again
            PIN_CTS_PORT &= (~PIN_CTS_MASK);              
            TCCR2B = 0;   // No Clock (turn off)                
            return;
        }
        WRITE_BIT(0x90,B00001000);
        CHECK_SERIAL();
        WRITE_BIT(0xB0,B00000100);
        CHECK_SERIAL();
        WRITE_BIT(0xD0,B00000010);
        CHECK_SERIAL();
        WRITE_BIT(0xF0,B00000001); 
        PIN_CTS_PORT|=PIN_CTS_MASK;   // Stop data coming in while we're not monitoring it
    }  
  
    // Turn off the write head
    PIN_WRITE_DATA_PORT|=PIN_WRITE_DATA_MASK;
    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;
    // Done!
    writeByteToUART('1');
    digitalWrite(PIN_ACTIVITY_LED,LOW);

    // Need to allow data to come in again
    PIN_CTS_PORT &= (~PIN_CTS_MASK);  
    
    TCCR2B = 0;   // No Clock (turn off)    
}

// This runs the PWM for writing a single pulse
#define RUN_PULSE()                                                                                  \
        while (!(TIFR2 &  bit(TOV2))) {                                                                \
          if (UCSR0A & bit(RXC0)) {                                                                    \
             PIN_CTS_PORT|=PIN_CTS_MASK;                                                               \
             SERIAL_BUFFER[serialWritePos++] = UDR0;                                                   \
             serialBytesInUse++;                                                                       \
          }                                                                                            \
        };                                                                                            \
        OCR2A = counter;    /* reset to zero when we get to counter */                               \
        OCR2B = counter-2;                                                                           \
        TIFR2 |= bit(TOV2);        

// Write a track to disk from the UART - the data should be pre-MFM encoded raw track data where '1's are the pulses/phase reversals to trigger  - This works like the precomp version as the old method isn;t fast enough
// This is 100% jitter free!
void writeTrackFromUART_HD() {
    // Check if its write protected.  
    if (digitalRead(PIN_WRITE_PROTECTED) == LOW) {
        writeByteToUART('N'); 
        return;
    } else
    writeByteToUART('Y');

    unsigned char waitForIndex = readByteFromUART();
    
    digitalWrite(PIN_ACTIVITY_LED,HIGH);
    
    writeByteToUART('!');     

    register unsigned char currentByte = readByteFromUART();
    
    // Fill our buffer to give us a head start
    for (int a=0; a<SERIAL_BUFFER_START; a++) {
        // Wait for it
        while (!( UCSR0A & ( 1 << RXC0 ))){}; 
        // Save the byte
        SERIAL_BUFFER[a] = UDR0;
    }  

    unsigned char serialReadPos = 0;
    unsigned char serialWritePos = SERIAL_BUFFER_START;
    unsigned char serialBytesInUse = SERIAL_BUFFER_START;

    PIN_CTS_PORT|=PIN_CTS_MASK;                // stop any more data coming in!

    // Reset the counter, ready for writing - abuse the Fast PWM to produce the wayforms we need.  All we do is change the value it resets at!
    TCCR2A = bit(COM2B1) | bit(WGM20) | bit(WGM21);  // (COM2B0|COM2B1) Clear OC2B. on compare match, set OC2B at BOTTOM.  WGM20|WGM21 is Fast PWM. 
    TCCR2B = bit(WGM22)| bit(CS20);         // WGM22 enables waveform generation.  CS20 starts the counter runing at maximum speed

    // Enable writing
    PIN_WRITE_DATA_PORT|=PIN_WRITE_DATA_MASK;
    
    // While the INDEX pin is high wait.  Might as well write from the start of the track
    if (waitForIndex || alwaysIndexAlignWrites) 
        while (PIN_INDEX_PORT & PIN_INDEX_MASK)  {};

    PIN_WRITE_GATE_PORT&=~PIN_WRITE_GATE_MASK;

    // Get it ready    
    OCR2A = 32;      // Just setup something to get this going        
    OCR2B = 30;         
    TCNT2 = 0;      
    TIFR2 |= bit(TOV2);

    // Loop through all bytes of data required.  Each byte contains two sequences to write
    do {
        // Group 1
        register unsigned char counter = 15 + (currentByte&B00110000);
        RUN_PULSE();
                
        // Group 2
        counter = 15 + ((currentByte&B00001100)*4);
        RUN_PULSE();
        if (serialBytesInUse<SERIAL_BUFFER_START) PIN_CTS_PORT &= (~PIN_CTS_MASK); else PIN_CTS_PORT|=PIN_CTS_MASK;    

        // Group 3
        counter = 15 + ((currentByte&B00000011) * 16);
        RUN_PULSE();

        // Check for overflows
        if (UCSR0A & (bit(FE0)|bit(DOR0))) break;

        // Group 4
        counter = 15 + ((currentByte&B11000000)/4);
        RUN_PULSE();

        if (!serialBytesInUse) break;        
        currentByte = SERIAL_BUFFER[serialReadPos++]; 
        serialBytesInUse--;
        
    } while (currentByte);    

    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;
    // Turn off the write head
    PIN_WRITE_DATA_PORT|=PIN_WRITE_DATA_MASK;

    TCCR2A = 0;              // disable and reset everything
    TCCR2B = 0;              // Stop timer 2     

    // Data wouldnt have been read quick enough
    if (currentByte) {
        if (UCSR0A & bit(FE0)) {
           writeByteToUART('Y');   // Serial Framing Error
        } else
        if (UCSR0A & bit(DOR0)) {
            writeByteToUART('Z');   // Serial IO overrun
        } else {
            writeByteToUART('X');   // Serial data not received quickly enough
        }
    } else {
      // Done!
      writeByteToUART('1');      
    }

    digitalWrite(PIN_ACTIVITY_LED,LOW);
    PIN_CTS_PORT &= (~PIN_CTS_MASK);     
}

// Write blank data to a disk so that no MFM track could be detected 
void eraseTrack() {    
    // Check if its write protected.  
    if (digitalRead(PIN_WRITE_PROTECTED) == LOW) {
        writeByteToUART('N'); 
        return;
    } else writeByteToUART('Y');
    
    digitalWrite(PIN_ACTIVITY_LED,HIGH);
   
    // Enable writing
    PIN_WRITE_GATE_PORT&=~PIN_WRITE_GATE_MASK;

    // To write the 01010101 sequence we're going to ask the Arduino to generate this from its PWM output.
    TCCR2A = bit(COM2B1) | bit(WGM20) | bit(WGM21);  // (COM2B0|COM2B1) Clear OC2B. on compare match, set OC2B at BOTTOM.  WGM20|WGM21 is Fast PWM. 
    TCCR2B = bit(WGM22)| bit(CS20);         // WGM22 enables waveform generation.  CS20 starts the counter runing at maximum speed
    // This generates a square wave, 3usec high, and 1usec low, 4uSec in total
    OCR2A = 63;                         
    OCR2B = 47;                         
    TCNT2=0;  

    // Now just count how many times this happens.  Approx 200ms is a revolution, so we'll go 200ms + 5% to be on the safe side (210ms)
    TIFR2 |= bit(TOV2);

    // 52500 is 210 / 0.004
    for (unsigned int counter=0; counter<65000; counter++) {
      // Every time this loop completes, 4us have passed.
      while (!(TIFR2 & bit(TOV2))) {};
      TIFR2 |= bit(TOV2);        
    };

    // Turn off the write head to stop writing instantly
    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;

    TCCR2A = 0;
    TCCR2B = 0;   // No Clock (turn off)    
  
    // Done!
    writeByteToUART('1');
    digitalWrite(PIN_ACTIVITY_LED,LOW);
}

// Write blank data to a disk so that no MFM track could be detected
void eraseTrack_HD() {
    // Check if its write protected.  
    if (digitalRead(PIN_WRITE_PROTECTED) == LOW) {
        writeByteToUART('N'); 
        return;
    } else writeByteToUART('Y');
    
    digitalWrite(PIN_ACTIVITY_LED,HIGH);
   
    // Enable writing
    PIN_WRITE_GATE_PORT&=~PIN_WRITE_GATE_MASK;

    // To write the 01010101 sequence we're going to ask the Arduino to generate this from its PWM output.
    TCCR2A = bit(COM2B1) | bit(WGM20) | bit(WGM21);  // (COM2B0|COM2B1) Clear OC2B. on compare match, set OC2B at BOTTOM.  WGM20|WGM21 is Fast PWM. 
    TCCR2B = bit(WGM22)| bit(CS20);         // WGM22 enables waveform generation.  CS20 starts the counter runing at maximum speed
    // This generates a square wave, 1.5usec high, and 0.5usec low, 2uSec in total
    OCR2A = 31;                         
    OCR2B = 23;         
    TCNT2=0;  

    // Now just count how many times this happens.  Approx 200ms is a revolution, so we'll go 200ms + 5% to be on the safe side (210ms)
    TIFR2 |= bit(TOV2);

    // This si the same as the DD version, except we do it twice as the HD version is twice as fast
    for (unsigned char loops=0; loops<2; loops++) 
      for (unsigned int counter=0; counter<65000; counter++) {
        // Every time this loop completes, 2us have passed.
        while (!(TIFR2 & bit(TOV2))) {};
        TIFR2 |= bit(TOV2);        
      };

    // Turn off the write head to stop writing instantly
    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;

    TCCR2A = 0;
    TCCR2B = 0;   // No Clock (turn off)    
  
    // Done!
    writeByteToUART('1');
    digitalWrite(PIN_ACTIVITY_LED,LOW);  
}

// Nasty string sending function
void sendString(char* str) {
  while (*str!='\0') {
    writeByteToUART(*str);
    str++;
  }
}

// Nasty int sending function
void sendInt(unsigned int i) {
  char buffer[6];
  buffer[0] = '0' + (i/10000)%10;
  buffer[1] = '0' + (i/1000)%10;
  buffer[2] = '0' + (i/100)%10;
  buffer[3] = '0' + (i/10)%10;
  buffer[4] = '0' + (i/1)%10;
  buffer[5] = '\0';
  for (int a=0; a<4; a++)
    if (buffer[a]=='0') buffer[a] = ' '; else break;
   sendString(buffer);
}

// Nasty send 'ticks' as uSec
void sendTickAsuSec(unsigned int i) {
  i*=62.5;
  int h = (i/10000)%10;
  writeByteToUART(h ? '0' + h : ' ');
  writeByteToUART('0' + (i/1000)%10);
  writeByteToUART('.');
  writeByteToUART('0' + (i/100)%10);
  writeByteToUART('0' + (i/10)%10);
  writeByteToUART('0' + (i/1)%10);
}

// Drive disk statistics (used to check #define at the top fo the code) - Updated for Drawbridge Plus
void measureCurrentDisk() {
  sendString("\n\nDrive Counters.  Current Parameters:\n");
  int offsetIgnore = 0;
  
  if (drawbridgePlusMode) {
      offsetIgnore = -TIMING_OVERHEAD; // PLUS mode doesnt use this value
      sendString("DrawBridge Plus Mode\n"); 
  } else {
    sendString("Timing Compensation Overhead in Ticks: ");
    if (TIMING_OVERHEAD<0) {
      writeByteToUART('-');
      sendInt(-TIMING_OVERHEAD);
    } else sendInt(TIMING_OVERHEAD);
  }
  sendString("\nTicks for middle 4, 6 and 8 uS: ");
  sendInt(TIMING_DD_MIDDLE_4us+offsetIgnore); sendString(", "); sendInt(TIMING_DD_MIDDLE_6us+offsetIgnore); sendString(", "); sendInt(TIMING_DD_MIDDLE_8us+offsetIgnore); sendString("\n");
  sendString("Ticks for Upper bound for 4, 6 and 8 uS: ");
  sendInt(TIMING_DD_UPPER_4us+offsetIgnore); sendString(", "); sendInt(TIMING_DD_UPPER_6us+offsetIgnore); sendString(", "); sendInt(TIMING_DD_UPPER_8us+offsetIgnore); sendString("\n");
  sendString("Ticks for HD Upper bound for 2, 3 uS: ");
  sendInt(TIMING_HD_UPPER_2us+offsetIgnore); sendString(", "); sendInt(TIMING_HD_UPPER_3us+offsetIgnore); sendString(", "); sendString("\n");
  sendString("\nBitcell timings for current track/side/disk: Testing...");
  
    unsigned int cc[256];
    for (int a=0; a<256; a++) cc[a] = 0;

    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20);       // Precale = 1 (ie: no prescale)
    EICRA = bit(ISC01);       // falling edge of INT0 generates an interrupt, they are turned off, but its an easy way for us to detect a falling edge rather than monitoring a pin
    // Wait for index pin
    while (!(EIFR&bit(INTF0))) {};
    EIFR=bit(INTF0);  // clear the register saying it detected an index pulse

    // Skip the first bit.  We're probably already half way through timing it
    if (drawbridgePlusMode) {
      TCCR1A = 0;
      TCCR1B = bit(CS10); // Capture from pin 8, falling edge. falling edge input capture, prescaler 1, no output compare
      TCCR1C = 0;

      while (PIN_READ_DATA_PLUSMODE_PORT & PIN_READ_DATA_PLUSMODE_MASK) {};
      while (!(PIN_READ_DATA_PLUSMODE_PORT & PIN_READ_DATA_PLUSMODE_MASK)) {};   
      unsigned char lastCounter = ICR1L;
      TIFR1 = bit(ICF1);
      
      for (int a=0; a<2; a++) {
        EIFR=bit(INTF0);  // clear the register saying it detected an index pulse
        
        while (!(EIFR&bit(INTF0))) {

           // Wait for capture or overflow
           while ((!(TIFR1 & bit(ICF1)))&&(!(TIFR2 & bit(OCF2B)))) {};

           // Was it a capture?
           if ((TIFR1 & bit(ICF1))) {
             // Grab the value, caluclate the change, and store the new value
             const unsigned char tmp = ICR1L;
             unsigned char output = ICR1L-lastCounter; 
             TIFR1 = bit(ICF1);   // Reset the cvapture flag
             lastCounter = tmp;
             
             if (cc[output]<65535) cc[output]++;
             TCNT2 = 0;   // Reset the overflow counter
            
           } else TIFR2 = bit(TOV2);
           
        }
      } 
      TCCR1B = 0;
    } else {
      while (PIN_READ_DATA_PORT & PIN_READ_DATA_MASK) {};
      TCNT2 = 0;  // reset
      while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) {};   

      for (int a=0; a<2; a++) {
        EIFR=bit(INTF0);  // clear the register saying it detected an index pulse
        
        while (!(EIFR&bit(INTF0))) {
           register unsigned char counter;
           while (PIN_READ_DATA_PORT & PIN_READ_DATA_MASK) { };                                                                                    
           counter = TCNT2, TCNT2 = 0;
           if (cc[counter]<65535) cc[counter]++;
           while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) { };                
        }
      } 
    }
    TCCR2B=0;

    sendString(".Completed 2 revolutions.\n\n");
    sendString("Ticks,   uSec, Count, Identified as\n"); 

    
    for (int a=1; a<=255; a++) {      
      if (cc[a]>1) {
        sendInt(a);
        sendString(", ");
        sendTickAsuSec(a);
        sendString(", ");
        sendInt(cc[a]);
        sendString(", ");

        if (a<TIMING_HD_UPPER_2us+offsetIgnore) {
              sendString("2us (HD)");
        } else                                                                                
        if (a<TIMING_HD_UPPER_3us+offsetIgnore) {                                                    
              sendString("3us (HD)");
        } else                                                                                
        if (a<TIMING_DD_UPPER_4us+offsetIgnore) {
              sendString("4us (DD/HD)");
        } else                                                                                
        if (a<TIMING_DD_UPPER_6us+offsetIgnore) {                                                    
              sendString("6us (DD)");
        } else                                                                                
        if (a<TIMING_DD_UPPER_8us+offsetIgnore) {                                                    
              sendString("8us (DD)");
        } else {
          sendString("10us illegal mfm");
        }
        
        writeByteToUART('\n');
      }
    }

    EICRA = 0;
}

// Read the track using a timings to calculate which MFM sequence has been triggered - Updated for Drawbridge Plus
void readTrackDataFast() {
    // Configure timer 2 just as a counter in NORMAL mode
    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20);       // Precale = 1  
    OCR2A = 0x00;
    OCR2B = 0x00;
    // First wait for the serial port to be available
    while(!(UCSR0A & (1<<UDRE0)));   
   
    // Signal we're active
    digitalWrite(PIN_ACTIVITY_LED,HIGH);

    // Force data to be stored in a register
    register unsigned char DataOutputByte = 0;

    // While the INDEX pin is high wait if the other end requires us to
    if (readByteFromUART())
        while (PIN_INDEX_PORT & PIN_INDEX_MASK)  {};

    TCNT2=0;       // Reset the counter

    register unsigned char counter;
    long totalBits=0;
    long target = ((long)RAW_TRACKDATA_LENGTH)*(long)8;

    if (drawbridgePlusMode) {
      // Drawbridge Plus mode

      TCCR1A = 0;
      TCCR1B = bit(CS10); // Capture from pin 8, falling edge. falling edge input capture, prescaler 1, no output compare
      TCCR1C = 0;
      unsigned char lastCounter = ICR1L;
      TIFR1 |= bit(ICF1);
      
      while (totalBits<target) {
            for (register unsigned char bits=0; bits<4; bits++) {
                // Wait while pin is high
                
                while (!(TIFR1 & bit(ICF1))) {};
                const unsigned char tmp = ICR1L;
                counter = tmp - lastCounter, lastCounter = tmp;
                TIFR1 |= bit(ICF1);
               
                DataOutputByte<<=2;   
    
                // DO NOT USE BRACES HERE, use the "," or the optomiser messes it up.  Numbers changed as these are best centered around where the bitcels actually are
                if (counter<TIMING_DD_UPPER_4us - TIMING_OVERHEAD) DataOutputByte|=B00000001,totalBits+=2; else    // this accounts for just a '1' or a '01' as two '1' arent allowed in a row
                if (counter<TIMING_DD_UPPER_6us - TIMING_OVERHEAD) DataOutputByte|=B00000010,totalBits+=3; else            
                if (counter<TIMING_DD_UPPER_8us - TIMING_OVERHEAD) DataOutputByte|=B00000011,totalBits+=4; else      
                                 totalBits+=5;   // this is treated as an 00001, which isnt allowed, but does work
            }
            if (!DataOutputByte) {
              // sending 0 here is wrong and will cause the PC side to get confused.  so we set one ot be a 8us rather than the fake 10
              DataOutputByte = B00000011; 
              totalBits--;  // account for one less bit
            }
            UDR0 = DataOutputByte;
      }
      TCCR1B = 0;
    } else {
        
        while (totalBits<target) {
            for (register unsigned char bits=0; bits<4; bits++) {
                // Wait while pin is high
                
                while (PIN_READ_DATA_PORT & PIN_READ_DATA_MASK) {
                };
                counter = TCNT2, TCNT2 = 0;  // reset - must be done with a COMMA
               
                DataOutputByte<<=2;   
    
                // DO NOT USE BRACES HERE, use the "," or the optomiser messes it up.  Numbers changed as these are best centered around where the bitcels actually are
                if (counter<TIMING_DD_UPPER_4us) DataOutputByte|=B00000001,totalBits+=2; else    // this accounts for just a '1' or a '01' as two '1' arent allowed in a row
                if (counter<TIMING_DD_UPPER_6us) DataOutputByte|=B00000010,totalBits+=3; else            
                if (counter<TIMING_DD_UPPER_8us) DataOutputByte|=B00000011,totalBits+=4; else      
                                 totalBits+=5;   // this is treated as an 00001, which isnt allowed, but does work
                
                // Wait until pin is high again
                while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) {};
            }
            if (!DataOutputByte) {
              // sending 0 here is wrong and will cause the PC side to get confused.  so we set one ot be a 8us rather than the fake 10
              DataOutputByte = B00000011; 
              totalBits--;  // account for one less bit
            }
            UDR0 = DataOutputByte;
        }
    }
    // Because of the above rules the actual valid two-bit sequences output are 01, 10 and 11, so we use 00 to say "END OF DATA"
    writeByteToUART(0);

    // turn off the status LED
    digitalWrite(PIN_ACTIVITY_LED,LOW);

    // Disable the counter
    TCCR2B = 0;      // No Clock (turn off)    
}

#define HALF_TIMING_OVERHEAD (TIMING_OVERHEAD >> 1)

// This is a nasty way to unroll the FOR loop.  Would be nice if there was a directive to do this - this over time will be jitter free, but not for the individual fluxs
#define READ_UNROLLED_LOOP(p4uSec, p6uSec, p8uSec)                                          \
      TIFR0 |= bit(OCF0B);                                                                  \
      while ((!(PCIFR & bit(PCIF2)))&&(!(TIFR0 & bit(OCF0B)))) {};                          \
      counter = TCNT0, TCNT0=0;                                                             \
      while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) {};                                \
      /* See what MFM 'window' this fits in. Its either a 2uSec, 4uSec or 6uSec,       */   \
      /* or 8+ which isnt technically allowed.   Numbers changed as these are best     */   \
      /* centered around where the bitcels actually are                                */   \
      if (PCIFR & bit(PCIF2)) {                                                              \       
        PCIFR |= bit(PCIF2);                                                                  \
        if (counter<TIMING_DD_UPPER_4us - HALF_TIMING_OVERHEAD) {                             \
          DataOutputByte|=p4uSec;                                                             \
          if (counter >= TIMING_DD_UPPER_2us - HALF_TIMING_OVERHEAD)                          \
              counter -= TIMING_DD_UPPER_2us - HALF_TIMING_OVERHEAD; else counter=0;          \
        } else                                                                                \
        if (counter<TIMING_DD_UPPER_6us - HALF_TIMING_OVERHEAD) {                             \
          DataOutputByte|=p6uSec;                                                             \
          counter -= TIMING_DD_UPPER_4us - HALF_TIMING_OVERHEAD;                              \
        } else                                                                                \
        if (counter<TIMING_DD_UPPER_8us - HALF_TIMING_OVERHEAD) {                             \
          DataOutputByte|=p8uSec;                                                             \
          counter -= TIMING_DD_UPPER_6us - HALF_TIMING_OVERHEAD;                              \
        } else {                                                                              \
          counter = 1;                                                                        \
        }                                                                                     \
      } else {                                                                                \
        counter = 1;                                                                          \
      }                                                                                       \
                                                                                   

// This is a nasty way to unroll the FOR loop.  Would be nice if there was a directive to do this
#define READ_UNROLLED_LOOP_DRAWBRIDGE_PLUS(p4uSec, p6uSec, p8uSec)                          \
    TIFR0 |= bit(OCF0B);                                                                    \
    while ((!(TIFR1 & bit(ICF1)))&&(!(TIFR0 & bit(OCF0B)))) {};                             \
    TCNT0 = 0;  /* otherwise an overflow will be detected when we dont want it */           \
    if (TIFR1 & bit(ICF1))  {                                                               \
        counter = ICR1L - lastCounter, lastCounter+= counter;                               \
        TIFR1 |= bit(ICF1);                                                                 \
        if (counter<TIMING_DD_UPPER_4us - TIMING_OVERHEAD) {                                \
          DataOutputByte|=p4uSec;                                                           \
          if (counter >= TIMING_DD_UPPER_2us - TIMING_OVERHEAD)                             \
              counter -= TIMING_DD_UPPER_2us - TIMING_OVERHEAD; else counter=0;             \
        } else                                                                              \
        if (counter<TIMING_DD_UPPER_6us - TIMING_OVERHEAD) {                                \
          DataOutputByte|=p6uSec;                                                           \
          counter -= TIMING_DD_UPPER_4us - TIMING_OVERHEAD;                                 \
        } else                                                                              \
        if (counter<TIMING_DD_UPPER_8us - TIMING_OVERHEAD) {                                \
          DataOutputByte|=p8uSec;                                                           \
          counter -= TIMING_DD_UPPER_6us - TIMING_OVERHEAD;                                 \
        } else {                                                                            \
          /* >=TIMING_DD_UPPER_8us is an error but some disks do this  */                   \
          counter = 1;                                                                      \
        }                                                                                   \
    } else {                                                                                \
        counter = 1;                                                                        \
        lastCounter += TIMING_DD_UPPER_8us - TIMING_OVERHEAD;                               \
    }                                                                                       \

// Read the track using a timings to calculate which MFM sequence has been triggered, however, this keeps running until a byte is received from the serial port telling it to stop - Updated for Drawbridge Plus
void readContinuousStream(bool highPrecision) {
    // Configure timer 2 just as a counter in NORMAL mode
    TCCR0A = 0 ;              // No physical output port pins and normal operation
    TCCR0B = bit(CS00);       // Precale = 1 (ie: no prescale)
    EICRA = bit(ISC01);       // falling edge of INT0 generates an interrupt, they are turned off, but its an easy way for us to detect a falling edge rather than monitoring a pin
    OCR0A = 0x00;
    OCR0B = 0x00;
    // First wait for the serial port to be available to receive
    while(!(UCSR0A & (1<<UDRE0)));  

    // Signal we're active
    digitalWrite(PIN_ACTIVITY_LED,HIGH);
  
    // Index alignment won't work if we're already at the index when this starts.  The index pulse is actually quite long and spans several bytes of data, could be several ms in length
    while (!(PIN_INDEX_PORT & PIN_INDEX_MASK)) {};

    TCNT0 = 0;  // reset
    // Skip the first bit.  We're probably already half way through timing it
    while (PIN_READ_DATA_PORT & PIN_READ_DATA_MASK) {
        if (TCNT0>250) break;                             // this is to stop the inteface freezing if theres no disk in the drive
    };
    TCNT0 = 0;  // reset
    if (drawbridgePlusMode) {
      while (!(PIN_READ_DATA_PLUSMODE_PORT & PIN_READ_DATA_PLUSMODE_MASK)) {};      
    } else {
      while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) {};
    }
      
    EIFR |=bit(INTF0);                 // clear the register saying it detected an index pulse

    if (drawbridgePlusMode) {
      OCR0B = TIMING_DD_UPPER_8us-2;   // This is set to the upper bound.  If we exceed this we must have received a load of non-flux data, this allows us to write '0000' on the PC and loop back round
      TCCR1A = 0;
      TCCR1B = bit(CS10); // Capture from pin 8, falling edge.
      TCCR1C = 0;
      
      unsigned char lastCounter = ICR1L;
      TIFR1 |= bit(ICF1);
      
      if (highPrecision) {  
          register unsigned char toTransmit = 0xC3;
          do {
                // A variable to store the data we collect
                register unsigned char DataOutputByte = 0;
                register unsigned char counter, average;
        
                READ_UNROLLED_LOOP_DRAWBRIDGE_PLUS(B01000000, B10000000, B11000000);
                average = counter;
                if ((EIFR&bit(INTF0))) {
                  EIFR|=bit(INTF0);
                  average|= 0x80;
                };   
    
                READ_UNROLLED_LOOP_DRAWBRIDGE_PLUS(B00010000, B00100000, B00110000);
                average += counter;
                UDR0 = toTransmit;
                
                READ_UNROLLED_LOOP_DRAWBRIDGE_PLUS(B00000100, B00001000, B00001100);
                average += counter;
                
                READ_UNROLLED_LOOP_DRAWBRIDGE_PLUS(B00000001, B00000010, B00000011);
                average += counter;
                toTransmit = average; 

                UDR0 = DataOutputByte;
                
            } while (!(UCSR0A & ( 1 << RXC0 )));
      } else {
          do {          
                  // A variable to store the data we collect
                  register unsigned char DataOutputByte = 0;
                  register unsigned char counter, average;
          
                  // format is INDEX B1 B2 Spd
          
                  READ_UNROLLED_LOOP(B00100000, B01000000, B01100000);
                  average = counter;
                  if ((EIFR&bit(INTF0))) {
                    EIFR|=bit(INTF0);
                    DataOutputByte|= 0x80;
                  }  
  
                  READ_UNROLLED_LOOP(B00001000, B00010000, B00011000);
                  average += counter;
                  average >>= 3;
                  UDR0 = average | DataOutputByte;
          
           } while (!(UCSR0A & ( 1 << RXC0 )));
       }

       TCCR1B = 0;
    } else {   
        OCR0B = TIMING_DD_UPPER_8us;   // This is set to the upper bound.  If we exceed this we must have received a load of non-flux data, this allows us to write '0000' on the PC and loop back round      
        // This sets up what would be an interrupt for when the READ PIN is signalled (unfortunatly we can't choose the direction. Its just set when it changes)
        // But this allows us to make sure we dont miss a bit, although the timing might be off slightly.  This is mainly used when disks have very long areas of
        // no flux transitions, typically used for copy protection
        PCMSK0 = 0;
        PCMSK1 = 0;
        PCMSK2 = bit(PCINT20);
        register char total = 0;
        register unsigned char toTransmit = 0xC3;
        unsigned char lastCounter = 0;
        TCNT0=0;

        if (highPrecision) {
          do {
                // A variable to store the data we collect
                register unsigned char DataOutputByte = 0;
                register unsigned char counter, average;
        
                READ_UNROLLED_LOOP(B01000000, B10000000, B11000000);
                average = counter;
                if ((EIFR&bit(INTF0))) {
                  EIFR|=bit(INTF0);
                  average|= 0x80;
                };   
    
                READ_UNROLLED_LOOP(B00010000, B00100000, B00110000);
                average += counter;
                UDR0 = toTransmit;
                
                READ_UNROLLED_LOOP(B00000100, B00001000, B00001100);
                average += counter;
                
                READ_UNROLLED_LOOP(B00000001, B00000010, B00000011);
                average += counter;
                
                toTransmit = average;
                UDR0 = DataOutputByte;
                
            } while (!(UCSR0A & ( 1 << RXC0 )));
        } else {
            do {
        
                // A variable to store the data we collect
                register unsigned char DataOutputByte = 0;
                register unsigned char counter, average;
        
                // format is INDEX B1 B2 Spd        
                READ_UNROLLED_LOOP(B00100000, B01000000, B01100000);
                average = counter;
                if ((EIFR&bit(INTF0))) {
                  EIFR|=bit(INTF0);
                  DataOutputByte|= 0x80;
                };      

                READ_UNROLLED_LOOP(B00001000, B00010000, B00011000);
                average += counter;                
                average >>= 3;
                UDR0 = average | DataOutputByte;
        
            } while (!(UCSR0A & ( 1 << RXC0 )));
        }

        PCMSK2 = 0;
    }

    // Read the byte that was sent to stop us, should be a 0, although we don't care
    unsigned char response = UDR0;

    // We want to make sure the PC knows we've quit, and whilst this isnt fool proof its a start.  
    // The chance of this exact sequence coming from MFM data from the drive is slim I guess
    // A little hacky, bit without woriding another pin to something we dont have any other options
    writeByteToUART('X');
    writeByteToUART('Y');
    writeByteToUART('Z');
    writeByteToUART(response);
    writeByteToUART('1');
    
    // turn off the status LED
    digitalWrite(PIN_ACTIVITY_LED,LOW);

    // Disable the counter
    TCCR0B = 0;      // No Clock (turn off)    
    EICRA = 0;      // disable monitoring   
    OCR0A = 0;
    OCR0B = 0;
}



#define HANDLE_INDEX_DETECTION(name)                                                            \
            "sbis  %14, %15            \n"           /* skip if ((EIFR&bit(INTF0))) */          \
            "rjmp NOTDETECTED_" name " \n"           /* bit not set, index not detected */      \
            "sbi %14, %15              \n"           /* Reset flag */                           \
            "ori r24, 0x80             \n"           /* Write the flag into the output */       \
            "NOTDETECTED_" name ":      \n"          /* End of check */                         \

#define READ_UNROLLED_LOOP_PLL(p4uSec, p6uSec, p8uSec, name)                                                                               \  
            "mov r27, r21                   \n"    /* Take a copy of the flux value */                                                     \
            "sub r27, r28                   \n"    /* Remove the PLL middle point, so we get a density value */                            \
            "subi r21, 16                   \n"    /* Remove the middle point, so we get a density value */                                \
            "cpi r27, %6                    \n"    /* if counter < TIMING_DD_MIDDLE_4us */                                                 \
            "brlo SAVEFLUX_LESS4US_" name " \n"    /* Goto the <4us code */                                                                \
            "cpi r27, %7                    \n"    /* if counter < TIMING_DD_MIDDLE_4us */                                                 \
            "brlo SAVEFLUX_LESS6US_" name " \n"    /* Goto the <6us code */                                                                \
            "cpi r27, %8                    \n"    /* if counter < TIMING_DD_MIDDLE_4us */                                                 \
            "brlo SAVEFLUX_LESS8US_" name " \n"    /* Goto the <8us code */                                                                \
                                                                                                                                           \
            "ldi r21, 16                    \n"    /* counter = 16 */                                                                      \
            "ldi r27, 16                    \n"    /* counter = 16 */                                                                      \
            "rjmp ENDOF_PLLFLUX_" name "    \n"    /* done! */                                                                             \  
                                                                                                                                           \
            "SAVEFLUX_LESS8US_" name ":     \n"    /* <6US bit */                                                                          \
            "subi r27, %7                   \n"    /* Subtract */                                                                          \
            "ori r23, " p8uSec "            \n"    /* DataOutputByte|=p8uSec; */                                                           \
            "cpi r21, %7                    \n"    /* if counter >= TIMING_DD_MIDDLE_6us */                                                \
            "brlo SAFEFLUX_LT_6US_" name "  \n"    /* less than, goto SAFEFLUX_LT_6US */                                                   \
            "subi r21, %7                   \n"    /* counter -= TIMING_DD_MIDDLE_62us */                                                  \
            "rjmp ENDOF_PLLFLUX_" name "    \n"    /* goto end */                                                                          \
            "SAFEFLUX_LT_6US_" name ":      \n"    /* less than 2us */                                                                     \
            "ldi r21, 0                     \n"    /* counter=0 */                                                                         \
            "rjmp ENDOF_PLLFLUX_" name "    \n"    /* done! */                                                                             \  
                                                                                                                                           \
            "SAVEFLUX_LESS6US_" name ":     \n"    /* <6US bit */                                                                          \
            "subi r27, %6                   \n"    /* Subtract */                                                                          \
            "ori r23, " p6uSec "            \n"    /* DataOutputByte|=p6uSec; */                                                           \
            "cpi r21, %6                    \n"    /* if counter >= TIMING_DD_MIDDLE_4us */                                                \
            "brlo SAFEFLUX_LT_4US_" name "  \n"    /* less than, goto SAFEFLUX_LT_4US */                                                   \
            "subi r21, %6                   \n"    /* counter -= TIMING_DD_MIDDLE_42us */                                                  \
            "rjmp ENDOF_PLLFLUX_" name "    \n"    /* goto end */                                                                          \
            "SAFEFLUX_LT_4US_" name ":      \n"    /* less than 2us */                                                                     \
            "ldi r21, 0                     \n"    /* counter=0 */                                                                         \
            "rjmp ENDOF_PLLFLUX_" name "    \n"    /* done! */                                                                             \  
                                                                                                                                           \
            "SAVEFLUX_LESS4US_" name ":     \n"    /* <4US bit */                                                                          \
            "cpi r27, %5                    \n"    /* if counter >= TIMING_DD_MIDDLE_2us */                                                \
            "brlo SAFEFLUX_LT_22US_" name " \n"    /* less than, goto SAFEFLUX_LT_2US */                                                   \
            "subi r27, %5                   \n"    /* counter -= TIMING_DD_MIDDLE_2us */                                                   \
            "rjmp NEXT_" name "    \n"    /* goto end */                                                                                   \
            "SAFEFLUX_LT_22US_" name ":     \n"    /* less than 2us */                                                                     \
            "ldi r27, 0                     \n"    /* counter=0 */                                                                         \
            "NEXT_" name ":                 \n"    /* end */                                                                               \
            "ori r23, " p4uSec "            \n"    /* DataOutputByte|=p4uSec; */                                                           \
            "cpi r21, %5                    \n"    /* if counter >= TIMING_DD_MIDDLE_2us */                                                \
            "brlo SAFEFLUX_LT_2US_" name "  \n"    /* less than, goto SAFEFLUX_LT_2US */                                                   \
            "subi r21, %5                   \n"    /* counter -= TIMING_DD_MIDDLE_2us */                                                   \
            "rjmp ENDOF_PLLFLUX_" name "    \n"    /* goto end */                                                                          \
            "SAFEFLUX_LT_2US_" name ":      \n"    /* less than 2us */                                                                     \
            "ldi r21, 0                     \n"    /* counter=0 */                                                                         \
                                                                                                                                           \
            "ENDOF_PLLFLUX_" name ":        \n"                                                                                            \
            "cpi r21, 32                    \n"   /* Check density value against 32 */                                                     \
            "brlo LESSTHAN_32_" name "      \n"   /* Skip if less than 32 */                                                               \ 
            "ldi r21, 31                    \n"   /* limit to 31 max */                                                                    \
            "LESSTHAN_32_" name ":          \n"   /* end */                                                                                \
            "NEXTFLUX_" name ":             \n"                                                                                            \


#define WAIT_FOR_FLUX_PLUS(name)                                                                                                           \
            "sbi %0, %12                     \n"   /* TIFR0 = OCF0B */                                                                     \
            "PLLFLUX_" name ":               \n"   /* Wait start */                                                                        \
            "sbic %1, %2                     \n"   /* fetch TIFR1 & bit(ICF1) and skip the next line if its clear */                       \
            "rjmp SAVEFLUX_" name "          \n"   /* skip other test */                                                                   \
            "sbis %0, %12                    \n"   /* fetch (TIFR0 & bit(OCF0B) and skip the next line if its set */                       \
            "rjmp PLLFLUX_" name "           \n"   /* Wait for flux to start */                                                            \
                                                                                                                                           \
            /* No flux detected for too long */                                                                                            \
            "out %3, r17                      \n"   /* TCNT0=0 */                                                                          \ 
            "add r22, r20                     \n"   /* Skip forward 8uS */                                                                 \
            "ldi r21, 16                      \n"  /* counter = 16 */                                                                      \
            "ldi r27, 16                      \n"  /* counter = 16 */                                                                      \
            "rjmp NEXTFLUX_" name "           \n"   /* and we're done */                                                                   \
                                                                                                                                           \
            /* Flux detected */                                                                                                            \
            "SAVEFLUX_" name ":             \n"                                                                                            \
            "out %3, r17                    \n"    /* TCNT0=0 */                                                                           \ 
            "lds r21, %13                   \n"    /* Copy ICR1L into r21 */                                                               \
            "sub r21, r22                   \n"    /* subtract last counter value (r22) */                                                 \
            "add r22, r21                   \n"    /* advance last counter value */                                                        \
            "sbi %1, %2                     \n"    /* Reset the input caputure flag TIFR1|=bit(ICF1) */                                    \

/*
 * The Classic mode cheats in several ways
 * We simulate the Input Capture using an interrupt.  But to keep the interrupt short and reduce its overhead we take advantage of:
 *     The pin we use for interrupt only detects a change, so you can't set it up to capture only for rising or falling edge
 *     So if we allow the falling edge to trigger the interrupt, and then cheat.  We use RET rather than RETI to exit the interrupt
 *     They're basically the same, *EXCEPT* RET doesnt re-enable the interrupts, meaning as the pin goes back to HIGH we dont get another
 *     interrupt triggered.  However, the flag (PCIFT bit PCIF2) will still get set.  So we can monitor for that to know when the interrupt
 *     occured rather than setting a flag
 */
#define WAIT_FLOR_FLUX_CLASSIC(name)                                                                                                       \  
            "sbi %0, %11                     \n"   /* TIFR2 = OCF2B */                                                                     \
            "PLLFLUX_" name ":               \n"   /* Wait start */                                                                        \
            "sbic %1, %2                     \n"   /* fetch PCIFR & bit(PCIF2) and skip the next line if its clear */                      \
            "rjmp SAVEFLUX_" name "          \n"   /* skip other test */                                                                   \
            "sbis %0, %11                    \n"   /* fetch (TIFR2 & bit(OCF2B) and skip the next line if its set */                       \
            "rjmp PLLFLUX_" name "           \n"   /* Wait for flux to start */                                                            \
                                                                                                                                           \
            /* No flux detected for too long */                                                                                            \
            "sts %16, r17                    \n"   /* TCNT2=0 */                                                                           \
            "add r22, r20                    \n"   /* Skip forward 8uS */                                                                  \
            "ldi r21, 16                     \n"   /* counter = 16 */                                                                      \
            "ldi r27, 16                     \n"   /* counter = 16 */                                                                      \
            "rjmp ENDOF_PLLFLUX_" name "     \n"   /* and we're done */                                                                    \
                                                                                                                                           \
            /* Flux detected */                                                                                                            \
            "SAVEFLUX_" name ":             \n"                                                                                            \
            "mov r21, r19                   \n"    /* grab the value captured, this is the time since last time */                         \
            "sub r21, r22                   \n"    /* subtract last counter value (r22) */                                                 \
            "add r22, r21                   \n"    /* advance last counter value */                                                        \
            "sbis  %12, %13                 \n"    /* Check the READ DATA pin */                                                           \
            "rjmp  .-4                      \n"    /* 2 Jump back if its still within a pulse */                                           \
            "sbi %1, %2                     \n"    /* 2 Clear the flag again incase it was singalled by the wrong change */                \
            "sei                            \n"    /* re-enable interrupts */         


ISR (PCINT2_vect,ISR_NAKED) {         // 4 to start an interrupt, 4 ish to jump to interrupt
  __asm__ __volatile__(
      "in r19, %0         \n"         // 1 Take a copy of TCNT0  
      "sts %2, r18        \n"         // Reset, taking into account the delay here    
      "sbi %3, %4         \n"         // TIFR2 = OCF2B (reset the 8us timer incase it happened during the interrupt)
      "ret                \n"         // !!!!! Return from the ISR, but WITHOUT enabling interrupts again, this is a little naughty, I should call reti, but I dont want this interrupt serviced again until im ready
                                       // Originally I disabled the interrupt by doing "sts %1, (value 0), but this way I save 2 clock cycles
      ::"i" _SFR_IO_ADDR(TCNT0), "m"(PCICR), "m"(TCNT2), "i"_SFR_IO_ADDR(TIFR2), "i" (OCF2B) :);       
}


// This is a special version of the above, converted to assembly language for speed, which includes a partial pll which is enough for weak/flaky bits to work
void readContinuousStreamPLL_ASM_CLASSIC() {
    TCCR0A = 0 ;              // No physical output port pins and normal operation
    TCCR0B = bit(CS00);       // Precale = 1 (ie: no prescale)
    EICRA = bit(ISC01);       // falling edge of INT0 generates an interrupt, they are turned off, but its an easy way for us to detect a falling edge rather than monitoring a pin
    OCR0A = 0x00;
    OCR0B = 0x00;

    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20);       // Precale = 1 (ie: no prescale)
    OCR2A = 0x00;
    OCR2B = 0x00;     
    // First wait for the serial port to be available to receive
    while(!(UCSR0A & (1<<UDRE0)));  

    // Signal we're active
    digitalWrite(PIN_ACTIVITY_LED,HIGH);
  
    // Index alignment won't work if we're already at the index when this starts.  The index pulse is actually quite long and spans several bytes of data, could be several ms in length
    while (!(PIN_INDEX_PORT & PIN_INDEX_MASK)) {};

    TCNT0 = 0;  // reset
    // Skip the first bit.  We're probably already half way through timing it
    while (PIN_READ_DATA_PORT & PIN_READ_DATA_MASK) {
        if (TCNT0>250) break;                             // this is to stop the interface freezing if theres no disk in the drive
    };
    TCNT0 = 0;  // reset
    while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) {};
      
    // This sets up what would be an interrupt for when the READ PIN is signalled (unfortunatly we can't choose the direction. Its just set when it changes)
    // This uses the interrupt for the experimental fluxStreamer() command below...
    PCMSK0 = 0;
    PCMSK1 = 0;
    PCMSK2 = bit(PCINT20);
    PCIFR |= bit(PCIF2);  // Clear that its been triggered
    PCICR = bit(PCIF2);   // enable an interrupt
    
    OCR2B = TIMING_DD_MIDDLE_8us + 7;

    unsigned char lastCounter = 0;
    EIFR |=bit(INTF0);     

    __asm__ volatile(
        "push r17                 \n"            // 0
        "push r18                 \n"            // 0 for the interrupt
        "push r19                 \n"            // To capture the value from the inter in the interrupt
        "push r20                 \n"            // Store r20  8uS exactly
        "push r21                 \n"            // Store r21  counter
        "push r22                 \n"            // Store r22  last counter
        "push r23                 \n"            // Store r23  DataOutputByte
        "push r24                 \n"            // Store r24  average
        "push r25                 \n"            // Store r25  toTransmit
        "push r26                 \n"            // Store r26  value from UCSRA0
        "push r27                 \n"            // To use as a copy of the flux value
        "push r28                 \n"            // PLL Value
        "push r29                 \n"            // PLL version of r24        
        "push r30                 \n"            // Copy of the above, when its ready to apply it        
        "ldi r28, 16              \n"            // Set to 16, the default
        "ldi r25, 0xC3            \n"            // toTransmit = 0xC3
        "ldi r20, %8              \n"            // Set the offset when no flux is detected 
        "in r22, %3               \n"            // grab the value captured, this is the time since last time
        "ldi r17, 0               \n"            // r17 = 0        
        "ldi r18, 8               \n"            // Value to reset TCNT2 to in the interrupt, allowing for some overhead        
        "sbi %1, %2               \n"            // Reset the input caputure flag TIFR1|=bit(ICF1)
        "sei                      \n"            // Start Interrupts
        
        "PLL_CLASSIC_LOOP_START:  \n"            
        "ldi r23, 0               \n"            // DataOutputByte = 0;
        
        WAIT_FLOR_FLUX_CLASSIC("C1")
        READ_UNROLLED_LOOP_PLL("0x40", "0x80", "0xC0","C1") 
        "mov r24, r21             \n"           // average = counter
        "mov r29, r27             \n"           // average = counter
        "lds r26, %10             \n"           // Read UCSR0A into r26

        HANDLE_INDEX_DETECTION("CLASSIC")

        WAIT_FLOR_FLUX_CLASSIC("C2")
        READ_UNROLLED_LOOP_PLL("0x10", "0x20", "0x30","C2")   
        "add r24, r21             \n"           // average += counter
        "add r29, r27             \n"           // average += counter
        "sts %4, r25              \n"           // UDR0 = toTransmit

        "cpi r30, 36               \n"           // Has it gone too fast?
        "brge ABOVE_LOWEST2       \n"           // Skip
        "dec r28                  \n"           // Decrease it
        "ABOVE_LOWEST2:           \n"

        WAIT_FLOR_FLUX_CLASSIC("C3")
        READ_UNROLLED_LOOP_PLL("0x04", "0x08", "0x0C","C3")
        "add r24, r21             \n"           // average += counter
        "add r29, r27             \n"           // average += counter

        "cpi r30, 88              \n"           // Has it gone too fast?
        "brlo ABOVE_HIGHEST2      \n"           // Skip
        "inc r28                  \n"           // Decrease it
        "ABOVE_HIGHEST2:          \n"           //
        
      
        WAIT_FLOR_FLUX_CLASSIC("C4")
        READ_UNROLLED_LOOP_PLL("0x01", "0x02", "0x03","C4")
        "add r24, r21             \n"           // average += counter
        "add r29, r27             \n"           // average += counter
        "mov r25, r24             \n"           // toTransmit = average
        "mov r30, r29             \n"           // Take a copy of the PLL counter        
        "sts %4, r23             \n"            // UDR0 = DataOutputByte;

        "sbrs r26, %9            \n"            // Skip the following line if the "data received" bit is set
        "rjmp PLL_CLASSIC_LOOP_START \n"        // Start again!

        "cli                     \n"            // Store r31
        "pop r30                 \n"            // Store r30
        "pop r29                 \n"            // Store r29
        "pop r28                 \n"            // Store r28
        "pop r27                 \n"            // Store r27
        "pop r26                 \n"            // Store r26
        "pop r25                 \n"            // Store r25
        "pop r24                 \n"            // Store r24
        "pop r23                 \n"            // Store r23
        "pop r22                 \n"            // Store r22
        "pop r21                 \n"            // Store r21
        "pop r20                 \n"            // Store r20
        "pop r19                 \n"            // Store r19
        "pop r18                 \n"            // Store r18
        "pop r17                 \n"            // Store r17
      :: /*0*/ "i"_SFR_IO_ADDR(TIFR2), /*1*/ "i"_SFR_IO_ADDR(PCIFR), /*2*/ "i"(PCIF2), /*3*/ "i"_SFR_IO_ADDR(TCNT0), /*4*/ "m"(UDR0),  
    /*5*/ "i"(TIMING_DD_MIDDLE_2us), /*6*/ "i"(TIMING_DD_MIDDLE_4us), /*7*/ "i"(TIMING_DD_MIDDLE_6us), /*8*/ "i"(TIMING_DD_MIDDLE_8us), 
    /*9*/ "i"(RXC0), /*10*/ "m" (UCSR0A), /*11*/ "i" (OCF2B), /*12*/ "i"_SFR_IO_ADDR(PIN_READ_DATA_PORT), /*13*/ "i"(PIN_READ_DATA), /*14*/"i" _SFR_IO_ADDR(EIFR), /*15*/"i" (INTF0),
    /*16*/ "m"(TCNT2));
 
    TCCR0B = 0;      // No Clock (turn off)    
    TCCR2B = 0;

    PCMSK2 = 0;
    PCICR = 0;
    
    // Read the byte that was sent to stop us, should be a 0, although we don't care
    unsigned char response = UDR0;

    // We want to make sure the PC knows we've quit, and whilst this isnt fool proof its a start.  
    // The chance of this exact sequence coming from MFM data from the drive is slim I guess
    // A little hacky, bit without woriding another pin to something we dont have any other options
    writeByteToUART('X');
    writeByteToUART('Y');
    writeByteToUART('Z');
    writeByteToUART(response);
    writeByteToUART('1');
    
    // turn off the status LED
    digitalWrite(PIN_ACTIVITY_LED,LOW);

    // Disable the counter  
    EICRA = 0;      // disable monitoring   
    OCR0A = 0;
    OCR0B = 0;
}



// This is a special version of the above, converted to assembly language for speed, which includes a partial pll which is enough for weak/flaky bits to work
void readContinuousStreamPLL_ASM_PLUS() {
    TCCR0A = 0 ;              // No physical output port pins and normal operation
    TCCR0B = bit(CS00);       // Precale = 1 (ie: no prescale)
    EICRA = bit(ISC01);       // falling edge of INT0 generates an interrupt, they are turned off, but its an easy way for us to detect a falling edge rather than monitoring a pin
    OCR0A = 0x00;
    OCR0B = 0x00;
    // First wait for the serial port to be available to receive
    while(!(UCSR0A & (1<<UDRE0)));  

    // Signal we're active
    digitalWrite(PIN_ACTIVITY_LED,HIGH);
  
    // Index alignment won't work if we're already at the index when this starts.  The index pulse is actually quite long and spans several bytes of data, could be several ms in length
    while (!(PIN_INDEX_PORT & PIN_INDEX_MASK)) {};

    TCNT0 = 0;  // reset
    // Skip the first bit.  We're probably already half way through timing it
    while (PIN_READ_DATA_PORT & PIN_READ_DATA_MASK) {
        if (TCNT0>250) break;                             // this is to stop the interface freezing if theres no disk in the drive
    };
    TCNT0 = 0;  // reset
    while (!(PIN_READ_DATA_PLUSMODE_PORT & PIN_READ_DATA_PLUSMODE_MASK)) {};      

    OCR0B = TIMING_DD_MIDDLE_8us+5;   // This is set to the upper bound.  If we exceed this we must have received a load of non-flux data, this allows us to write '000' on the PC and loop back round
    TCCR1A = 0;
    TCCR1B = bit(CS10);             // Capture from pin 8, falling edge.
    TCCR1C = 0;
  
    EIFR |=bit(INTF0);

  __asm__ volatile(
        "push r17                 \n"            // Store r17  0
        "push r20                 \n"            // To store max flux allowed
        "push r21                 \n"            // Store r21  counter
        "push r22                 \n"            // Store r22  last counter
        "push r23                 \n"            // Store r23  DataOutputByte
        "push r24                 \n"            // Store r24  average
        "push r25                 \n"            // Store r25  toTransmit
        "push r26                 \n"            // Store r26  value from UCSRA0
        "push r27                 \n"            // To use as a copy of the flux value
        "push r28                 \n"            // PLL Value
        "push r29                 \n"            // PLL version of r24
        "push r30                 \n"            // Copy of the above, when its ready to apply it
        "ldi r28, 16              \n"            // Set to 16, the default
        "ldi r17, 0               \n"            // We need a zero
        "lds r22, %16             \n"            // lastcounter = TCNT1L
        "ldi r25, 0xC3            \n"            // toTransmit = 0xC3
        "ldi r20, %4              \n"            // 
        "sbi %1, %2               \n"            // Reset the input caputure flag TIFR1|=bit(ICF1)
        
        "PLL_PLUS_LOOP_START:     \n"            
        "ldi r23, 0               \n"            // DataOutputByte = 0;

        WAIT_FOR_FLUX_PLUS("P1")
        READ_UNROLLED_LOOP_PLL("0x40", "0x80", "0xC0","P1")          
        "mov r24, r21             \n"           // average = counter
        "mov r29, r27             \n"           // average = counter
        "lds r26, %10              \n"           // Read UCSR0A into r26
        
        HANDLE_INDEX_DETECTION("PLUS")

        WAIT_FOR_FLUX_PLUS("P2")
        READ_UNROLLED_LOOP_PLL("0x10", "0x20", "0x30","P2")
        "add r24, r21             \n"           // average += counter
        "add r29, r27             \n"           // average += counter
        "sts %11, r25              \n"          // UDR0 = toTransmit

        "cpi r30, 36              \n"           // Has it gone too fast?
        "brge ABOVE_LOWEST        \n"           // Skip
        "dec r28                  \n"           // Decrease it
        "ABOVE_LOWEST:            \n"           // OK
        
        WAIT_FOR_FLUX_PLUS("P3")
        READ_UNROLLED_LOOP_PLL("0x04", "0x08", "0x0C","P3")
        "add r24, r21             \n"           // average += counter
        "add r29, r27             \n"           // average += counter

        "cpi r30, 88              \n"           // Has it gone too fast?
        "brlo ABOVE_HIGHEST       \n"           // Skip
        "inc r28                  \n"           // Decrease it
        "ABOVE_HIGHEST:           \n"           // End
                
      
        WAIT_FOR_FLUX_PLUS("P4")
        READ_UNROLLED_LOOP_PLL("0x01", "0x02", "0x03","P4")
        "add r24, r21             \n"           // average += counter
        "add r29, r27             \n"           // average += counter
        "mov r25, r24             \n"           // toTransmit = average
        "mov r30, r29             \n"           // Take a copy of the PLL counter
        "sts %11, r23             \n"           // UDR0 = DataOutputByte;

        "sbrs r26, %9            \n"            // Skip the following line if the "data received" bit is set
        "rjmp PLL_PLUS_LOOP_START \n"           // Start again!

        "pop r30                 \n"            // Store r30
        "pop r29                 \n"            // Store r29
        "pop r28                 \n"            // Store r28
        "pop r27                 \n"            // Store r27
        "pop r26                 \n"            // Store r26
        "pop r25                 \n"            // Store r25
        "pop r24                 \n"            // Store r24
        "pop r23                 \n"            // Store r23
        "pop r22                 \n"            // Store r22
        "pop r21                 \n"            // Store r21
        "pop r20                 \n"            // Store r20
        "pop r17                 \n"            // Store r17
      :: /*0*/ "i"_SFR_IO_ADDR(TIFR0), /*1*/ "i" _SFR_IO_ADDR(TIFR1), /*2*/ "i" (ICF1), /*3*/ "i"_SFR_IO_ADDR(TCNT0), /*4*/ "i"(TIMING_DD_MIDDLE_8us),  
    /*5*/ "i"(TIMING_DD_MIDDLE_2us), /*6*/ "i"(TIMING_DD_MIDDLE_4us), /*7*/ "i"(TIMING_DD_MIDDLE_6us), /*8*/ "i"(TIMING_DD_MIDDLE_8us), 
    /*9*/ "i"(RXC0), /*10*/ "m" (UCSR0A), /*11*/ "m" (UDR0), /*12*/ "i"(OCF0B), /*13*/ "m"(ICR1L), /*14*/"i" _SFR_IO_ADDR(EIFR), /*15*/"i" (INTF0), /* 16 */ "m"(TCNT1L));
        

    TCCR1B = 0;

    // Read the byte that was sent to stop us, should be a 0, although we don't care
    unsigned char response = UDR0;

    // We want to make sure the PC knows we've quit, and whilst this isnt fool proof its a start.  
    // The chance of this exact sequence coming from MFM data from the drive is slim I guess
    // A little hacky, bit without woriding another pin to something we dont have any other options
    writeByteToUART('X');
    writeByteToUART('Y');
    writeByteToUART('Z');
    writeByteToUART(response);
    writeByteToUART('1');
    
    // turn off the status LED
    digitalWrite(PIN_ACTIVITY_LED,LOW);

    // Disable the counter
    TCCR0B = 0;      // No Clock (turn off)    
    EICRA = 0;      // disable monitoring   
    OCR0A = 0;
    OCR0B = 0;
}


// Yeah those functions we're quite large on their own
void readContinuousStreamPLL_ASM() {
  if (drawbridgePlusMode) {
      readContinuousStreamPLL_ASM_PLUS();
  } else {
    readContinuousStreamPLL_ASM_CLASSIC();
  }
}



/***************************************************************************************************************************************************
 * This is an experimental flux read at 125ns resolution.  It works ok as long as no flux transitions repeatdly occur below around 3600ns, which   *
 * for normal disks is ok, but things like long tracks wont get read properly.                                                                     *
 * It's a work i progress.  I hope to come up with a better solution at some point                                                                 *
 * *************************************************************************************************************************************************
 */


#define MAX_FLUX_SIGNAL           31

// RAW Counter Values
#define MIN_FLUX_ALLOWED          48                      // in 62.5 time 
#define MAX_FLUX_ALLOWED          (MIN_FLUX_ALLOWED+61)   // in 62.5 time - comes out as '30'
#define MAX_FLUX_REPEAT           (MAX_FLUX_ALLOWED-6)    // in 62.5 time - comes out as '27'
#define FLUX_REPEAT_OFFSET        (MAX_FLUX_REPEAT-MIN_FLUX_ALLOWED)    // The amount MAX_FLUX_SIGNAL represents in clock ticks


// We optimise this for the exact value of 31 we're storing here
#define ENCODE_FLUXREPEAT_ASM(id, loopName, finishedName)                                         \      
        "cpi r23, 1                           \n"  /* check the mode */                           \
        "brlo MODE_0_FIXED_" id "             \n"  /* mode=0 */                                   \
        "breq MODE_1_FIXED_" id "             \n"  /* mode=1 */                                   \
        /* Mode 2                            */                                                   \
        "ori r25, 31                          \n"  /* OutputByte2 |= amount */                    \
        "sbis  %0, %1                         \n"  /* skip next line if (!EIFR&bit(INTF0) */      \
        "rjmp SKIP_INDEX_FIXED_" id "         \n"  /* skip the following block  */                \
        "sbi %0, %1                           \n"  /* Reset the register  */                      \
        "ori r25, 0x80                        \n"  /* MSB means "index detected"  */              \
        "SKIP_INDEX_FIXED_" id ":             \n"  /* skipping index marker  */                   \
        "ldi r23, 0                           \n"  /* set mode=0 */                               \        
        "WAIT_UART3_" id ":                    \n" /* retry point */                              \
        "lds r26, %4                           \n" /* r26 = UCSR0A */                             \
        "sbrs r26, %11                         \n" /* See If UDRE0 is set in UCSR0A */            \
        "rjmp WAIT_UART3_" id "                \n"                                                \
        "sts %3, r25                          \n"  /* UDR0=OutputByte2 */                         \
        "rjmp " loopName "                    \n"                                                 \
        /* Mode 0                            */                                                   \
        "MODE_0_FIXED_" id ":                  \n"                                                \
        "ldi r24, 31                           \n"  /* outputByte1 = amount; */                   \
        "ldi r23, 1                            \n"  /* mode=1 */                                  \
        "lds r20, %4                           \n"  /* r20 = UCSR0A */                            \
        "sbrc r20, %5                          \n"  /* See If RXCO is set in UCSR0A */            \
        "rjmp " finishedName "                 \n"  /* Quit!  */                                  \
        "rjmp " loopName "                     \n"                                                \
        /* Mode 1                             */                                                  \
        "MODE_1_FIXED_" id ":                  \n"                                                \
        "ori r24, 0xE0                         \n"  /* store it */                                \
        "WAIT_UART4_" id ":                    \n" /* retry point */                              \
        "lds r26, %4                           \n" /* r26 = UCSR0A */                             \
        "sbrs r26, %11                         \n" /* See If UDRE0 is set in UCSR0A */            \
        "rjmp WAIT_UART4_" id "                \n"                                                \
        "sts %3, r24                           \n"  /* UDR0=OutputByte1 */                        \
        "ldi r25, 0x60                         \n"  /* OutputByte2=amount */                      \
        "ldi r23, 2                            \n"  /* mode=2 */                                  \
        "rjmp " loopName "                     \n"                                                \

#define ENCODE_FLUX_ASM(id, loopName, finishedName)                                               \
        "cpi r23, 1                           \n"  /* check the mode */                           \
        "brlo MODE_0_" id "                   \n"  /* mode=0 */                                   \
        "breq MODE_1_" id "                   \n"  /* mode=1 */                                   \
        /* Mode 2 */                                                                              \
        "or r25, r20                          \n"  /* OutputByte2 |= amount */                    \
        "sbis  %0, %1                         \n"  /* skip next line if (!EIFR&bit(INTF0)  */     \
        "rjmp SKIP_INDEX_" id "               \n"  /* skip the following block  */                \
        "sbi %0, %1                           \n"  /* Reset the register  */                      \
        "ori r25, 0x80                        \n"  /* MSB means "index detected"  */              \
        "SKIP_INDEX_" id ":                   \n"  /* skipping index marker */                    \
        "ldi r23, 0                           \n"  /* set mode=0 */                               \
        "WAIT_UART1_" id ":                    \n" /* retry point */                              \
        "lds r26, %4                           \n" /* r26 = UCSR0A */                             \
        "sbrs r26, %11                         \n"  /* See If UDRE0 is set in UCSR0A */           \
        "rjmp WAIT_UART1_" id "                \n"                                                \        
        "sts %3, r25                          \n"  /* UDR0=OutputByte2 */                         \
        "rjmp " loopName "                    \n"                                                 \
        /* Mode 0 */                                                                              \
        "MODE_0_" id ":                        \n"                                                \
        "mov r24, r20                          \n" /* outputByte1 = amount; */                    \
        "ldi r23, 1                            \n" /* mode=1 */                                   \
        "lds r20, %4                           \n"  /* r20 = UCSR0A */                            \
        "sbrc r20, %5                          \n"  /* See If RXCO is set in UCSR0A */            \
        "rjmp " finishedName "                 \n"  /* Quit!  */                                  \
        "rjmp " loopName "                     \n"                                                \
        /* Mode 1 */                                                                              \
        "MODE_1_" id ":                        \n"                                                \
        "mov r26, r20                          \n" /* make a copy of amount */                    \
        "swap r26                              \n" /* swap nybbles */                             \
        "lsl r26                               \n" /* shift left 1 */                             \
        "andi r26, 0xE0                        \n" /* and off the bits we dont want */            \
        "or r24, r26                           \n" /* store it */                                 \
        "WAIT_UART2_" id ":                    \n" /* retry point */                              \
        "lds r26, %4                           \n"  /* r26 = UCSR0A */                            \
        "sbrs r26, %11                         \n"  /* See If UDRE0 is set in UCSR0A */           \
        "rjmp WAIT_UART2_" id "                \n"                                                \
        "sts %3, r24                           \n"  /* UDR0=OutputByte1 */                        \
        "mov r25, r20                          \n"  /* OutputByte2=amount */                      \ 
        "andi r25, 0x18                        \n" /* AND with B00011000 */                       \
        "lsl r25                               \n"  /* << 1 */                                    \
        "lsl r25                               \n"  /* << 1 */                                    \
        "ldi r23, 2                            \n"  /* mode=2 */                                  \
        "rjmp " loopName "                     \n"  

// Checks the value of flux in r20, and handles its difference between it and r22 (last flux value)
#define CHECK_RECEIVED_FLUX(version)                                                                \
        "cpi r20, %10                     \n"  /* Compare timer to MIN_FLUX_ALLOWED */              \
        "brsh CHECK_UPPER_" version "     \n"  /* Skip if its at least MIN_FLUX_ALLOWED */          \
        "ldi r20, 0                       \n"  /* Make it 0 */                                      \
        "rjmp FLUX_OK_" version "         \n"  /* jump to flux encoding! */                         \
                                                                                                    \
        "CHECK_UPPER_" version ":         \n"                                                       \
        "subi r20, %10                    \n"  /* Subtract MIN_FLUX_ALLOWED */                      \
        "lsr r20                          \n"  /* Divide by 2  */                                   \
        "cpi r20, %8                      \n"  /* Compare timer to MAX_FLUX_SIGNAL-1 (30) */        \
        "brlo FLUX_OK_" version "         \n"  /* If <MAX_FLUX_SIGNAL */                            \
        "ldi r20, %9                      \n"  /* else, change it to MAX_FLUX_SIGNAL-2 (29) */      \
                                                                                                    \
        "FLUX_OK_" version ":             \n"  /* Later on...  */                                   \


ISR (TIMER1_CAPT_vect, ISR_NAKED) {
   __asm__ __volatile__(
      "lds r27, %0        \n"         // 1 Take a copy of ICR1L
      "in r29, __SREG__   \n"
      "subi r27, 1        \n"         // Compensate for the the extra clock cycle lds takes compared to in
      "sub r27, r22       \n"
      "add r22, r27       \n"
      "out __SREG__, r29  \n"
      "ldi r28, 0xFF      \n"         // 1 Signal new value is available
      "reti               \n"         // 4 Return from ISR      
      ::"m"(ICR1L) :);    
}

// Attempt to stream flux over two bytes in the following format:
// f2 f2 f2 f14 f13 f12 f11 f10
// II f2 f2 f3 f3 f3 f3 f3
void fluxStreamer() {
    EICRA = bit(ISC01);       // falling edge of INT0 generates an interrupt, they are turned off, but its an easy way for us to detect a falling edge rather than monitoring a pin
    // First wait for the serial port to be available to receive
    while(!(UCSR0A & (1<<UDRE0)));   
   
    // Signal we're active
    digitalWrite(PIN_ACTIVITY_LED,HIGH);
  
    // Index alignment won't work if we're already at the index when this starts.  The index pulse is actually quite long and spans several bytes of data, could be several ms in length
    while (!(PIN_INDEX_PORT & PIN_INDEX_MASK)) {};

    if (drawbridgePlusMode) {      
      TCCR1A = 0;
      TCCR1B = bit(CS10); // Capture from pin 8, falling edge. falling edge input capture, prescaler 1, no output compare
      TCCR1C = 0;

      TIFR1 = bit(ICF1);
      EIFR |=bit(INTF0);  // clear the register saying it detected an index pulse
      TIMSK1 |= bit(ICIE1);
      
        __asm__ __volatile__(
            "push r20                       \n"   // To hold the counter
            "push r21                       \n"   // To hold FLUX_REPEAT_OFFSET
            "push r22                       \n"   // To hold the last counter value
            "push r23                       \n"   // r23 will be the MODE
            "push r24                       \n"   // OutputByte1
            "push r25                       \n"   // OutputByte2
            "push r26                       \n"   // tmp
            "push r27                       \n"   // New counter value
            "push r28                       \n"   // New counter received
            "push r29                       \n"   // tmp 
            "ldi r23, 0                     \n"   // Which defaults to 0
            "ldi r28, 0                     \n"
            "ldi r21, %7                    \n"   // Set r21 to FLUX_REPEAT_OFFSET
            "mov r22, r28                   \n"   // Set "fluxSoFar" to 0
            "sts %6, r22                    \n"   // TCNT1L = 0 
            "sei                            \n"

            "LOOPSTARTPLUS:                 \n"   // Start of the loop               
            "WAITFORFLUX_PLUS:               \n"  // Sit and wait for flux
            
            "lds r20, %6                     \n"  // r20 = TCNT1L
            "sbrc r28, 0                     \n"  // Check if input capture occured
            "rjmp SEND_FLUX_PLUS             \n"  // Go and send the flux
                        
            // bit not detected
            "sub r20, r22                    \n"  // r20-=r22 (fluxSoFar)
            "cpi r20, %2                     \n"  // compare to MAX_FLUX_REPEAT
            "brlo WAITFORFLUX_PLUS           \n"  // If <MAX_FLUX_REPEAT then go back to the start
            
            "add r22, r21                    \n"  // Else, Add: fluxSoFar+=FLUX_REPEAT_OFFSET      
            ENCODE_FLUXREPEAT_ASM("plus", "LOOPSTARTPLUS", "FINISHEDPLUS")            

            // This is the FLUX section
            "SEND_FLUX_PLUS:                 \n"     
            "mov r20, r27                    \n"  // r27 = last captured flux time
            "ldi r28, 0                      \n"  // Reset the flag that says there was something                     
            CHECK_RECEIVED_FLUX("plus")
            ENCODE_FLUX_ASM("plus", "LOOPSTARTPLUS", "FINISHEDPLUS")       // easier

            "FINISHEDPLUS:                    \n"
            "cli                              \n"

            "pop r29                          \n"    
            "pop r28                          \n"    
            "pop r27                          \n"
            "pop r26                          \n"
            "pop r25                          \n"
            "pop r24                          \n"
            "pop r23                          \n"
            "pop r22                          \n"
            "pop r21                          \n"
            "pop r20                          \n"
          ::"i"_SFR_IO_ADDR(EIFR), "i"(INTF0), "i"(MAX_FLUX_REPEAT), "m"(UDR0) , "m"(UCSR0A), "i"(RXC0), "m"(TCNT1L), "i"(FLUX_REPEAT_OFFSET), \
              "i"(MAX_FLUX_SIGNAL), "i"(MAX_FLUX_SIGNAL-1), "i"(MIN_FLUX_ALLOWED), "i"(UDRE0), "m"(ICR1L)); 

        TIMSK1 = 0;
        TCCR1B = 0;
    } else {
        PCMSK0 = 0;
        PCMSK1 = 0;
        PCMSK2 = bit(PCINT20);
        PCIFR |= bit(PCIF2);  // Clear that its been triggered
        PCICR = bit(PCIF2);   // enable an interrupt
        
        TCCR0A = 0 ;              // No physical output port pins and normal operation
        TCCR0B = bit(CS00);       // Precale = 1 (ie: no prescale)
        OCR0A = 0x00;
        OCR0B = 0x00;
        EIFR |=bit(INTF0);  // clear the register saying it detected an index pulse

        // We have to use volatile because the code can't understand whats modifying r27 and r28 (interrupt)
        __asm__ __volatile__(
            "push r18                       \n"   // Last value read from the Interrupt
            "push r19                       \n"   // Value to write to TCNT2
            "push r20                       \n"   // To hold the counter
            "push r21                       \n"   // To hold FLUX_REPEAT_OFFSET
            "push r22                       \n"   // To hold the last counter value
            "push r23                       \n"   // r23 will be the MODE
            "push r24                       \n"   // OutputByte1
            "push r25                       \n"   // OutputByte2
            "push r26                       \n"   // tmp
            "push r29                       \n"   // tmp
            "ldi r23, 0                     \n"   // Which defaults to 0
            "ldi r21, %7                    \n"   // Set r21 to FLUX_REPEAT_OFFSET
            "ldi r28, 0                     \n"   // Reset r28
            "mov r22, r23                   \n"   // Set "fluxSoFar" to 0
            "out %6, r22                    \n"   // TCNT0 = 0 
            "sei                            \n"   // Enable interrupts

            "LOOPSTART:                     \n"   // Start of the loop   

            "WAITFORFLUX:                    \n"  // Sit and wait for flux
            "in r20, %6                      \n"  // r20 = TCNT0
            "sbic %14, %15                   \n"  // Check if our interrupt has received a new counter value (the first bit will have been cleared)
            "rjmp SEND_FLUX                  \n"  // Go and send the flux
                        
            // bit not detected
            "sub r20, r22                    \n"  // r20-=r22 (fluxSoFar)
            "cpi r20, %2                     \n"  // compare to MAX_FLUX_REPEAT
            "brlo WAITFORFLUX                \n"  // If <MAX_FLUX_REPEAT then go back to the start
            
            "add r22, r21                    \n"  // Else, Add: fluxSoFar+=FLUX_REPEAT_OFFSET      
            ENCODE_FLUXREPEAT_ASM("standard", "LOOPSTART", "FINISHED")            

            // This is the FLUX section
            "SEND_FLUX:                      \n"   
            "mov r20, r19                    \n"  // r20 = last captured flux time
            "sub r20, r22                    \n"
            "add r22, r20                    \n"
            CHECK_RECEIVED_FLUX("standard")
            "sbis  %12, %13                 \n"    // Check the READ DATA pin
            "rjmp  .-4                      \n"    // 2 Jump back if its still within a pulse                                  
            "sbi %14, %15                   \n"    // 2 Clear the flag again incase it was singalled by the wrong change   
            "sei                            \n"    // Re-enable interrupts
            ENCODE_FLUX_ASM("standard", "LOOPSTART", "FINISHED")       // easier

            "FINISHED:                        \n"
            
            "cli                              \n"  // Disable interrupts
            "pop r29                          \n"
            "pop r26                          \n"
            "pop r25                          \n"
            "pop r24                          \n"
            "pop r23                          \n"
            "pop r22                          \n"
            "pop r21                          \n"
            "pop r20                          \n"
          ::"i"_SFR_IO_ADDR(EIFR),"i"(INTF0), "i"(MAX_FLUX_REPEAT), "m"(UDR0) , "m"(UCSR0A), "i"(RXC0),"i" _SFR_IO_ADDR(TCNT0), "i"(FLUX_REPEAT_OFFSET), \
              "i"(MAX_FLUX_SIGNAL), "i"(MAX_FLUX_SIGNAL-1), "i"(MIN_FLUX_ALLOWED), "i"(UDRE0),
             /*12*/ "i"_SFR_IO_ADDR(PIN_READ_DATA_PORT), /*13*/ "i"(PIN_READ_DATA), /*14*/ "i"_SFR_IO_ADDR(PCIFR), /*15*/ "i"(PCIF2));
        
        TCCR0B = 0;      // No Clock (turn off)    
        PCMSK2 = 0;
        PCICR = 0;
    }

    // Read the byte that was sent to stop us, should be a 0, although we don't care
    unsigned char response = UDR0;
    // We want to make sure the PC knows we've quit, and whilst this isnt fool proof its a start.  
    // The chance of this exact sequence coming from MFM data from the drive is slim I guess
    // A little hacky, bit without woriding another pin to something we dont have any other options
    writeByteToUART('X');
    writeByteToUART('Y');
    writeByteToUART('Z');
    writeByteToUART(response);
    writeByteToUART('1');
    
    // turn off the status LED
    digitalWrite(PIN_ACTIVITY_LED,LOW);

    // Disable the counter
    EICRA = 0;      // disable monitoring   
}























// I originally coded this in ASM as I wasn't sure it was going to be fast enough, it works both ways but is slightly faster with ASM, so i'll leave it enabled
#define USE_ASM
// Enables the HD PLL.  This is always on while this is defined.  It helps improve poor disks with lots of flutter
#define ENABLE_PLL_DRAWBRIDGE_PLUS

// This is a nasty way to unroll the FOR loop.  This has an error of upto 3 clock ticks. For DD this isn't a problem, for HD it can be. Which is why Drawbridge Plus exists
#define READ_UNROLLED_LOOP_HD_BITMODE()                                                     \
            while (!(PCIFR & bit(PCIF2))) {};                                               \
            counter = TCNT0,TCNT0 = TIMING_OVERHEAD + DB_CLASSIC_HD_MIDDLE;                \
            while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) {};                          \
            PCIFR |= bit(PCIF2);                                                            \
            counter /= 16;                                                                  \
            if (counter < 2) {                                                              \
              if ((EIFR&bit(INTF0))) {                                                      \
                EIFR|=bit(INTF0);                                                           \
                DataOutputByte |= 3;                                                        \
              }                                                                             \
            } else                                                                          \
            if (counter == 2) DataOutputByte |= 1; else DataOutputByte |= 2;                \ 
            asm("wdr");               /* reset watchdog timer */                            \
            WDTCSR |= bit(WDIE);      /* Reset its flags */                                 \


#define READ_UNROLLED_LOOP_HD_ASM_BITMODE(id)                                                                                                     \
            asm("sbis 0x1b, 2                    \n"   /* fetch PCIFR & bit(PCIF2) and skip the next line if its set */                         \
                "rjmp .-4                        \n"   /* go back and try again */                                                              \
                "in  r20, 0x26                   \n"   /* 1 Tick,   Read Timer0 value */                                                        \
                "out 0x26, r22                   \n"   /* 1 Tick,   Reset Timer0 value */                                                       \
                                                                                                                                                  \
                "sbis  0x09, 4                   \n"   /* Check the READ DATA pin */                                                            \
                "rjmp  .-4                       \n"   /* Jump back if its still within a pulse */                                              \
                "sbi 0x1b, 2                     \n"   /* Reset the flag that started all of this */                                            \
                                                                                                                                                  \
                "swap r20                        \n"   /* 1 Tick, swap the nybbles */                                                           \
                "andi r20, 15                    \n"   /* keep the last 4 bits */                                                               \
                "cpi r20, 2                      \n"   /* compare with 2 */                                                                     \
                "brlo handle2us_" id "           \n"   /* If less than 2, then branch to the 2us handler */                                     \
                                                                                                                                                  \
                "breq handle3us_" id "           \n"   /* If equal to 2, branch to the 3us handler */                                           \
                "ori r24, 2                      \n"   /* Store the code for 4us */                                                             \
                "rjmp endcode_" id "             \n"   /* goto the end */                                                                       \
                                                                                                                                                  \
                "handle3us_" id ":               \n"   /* 3us Handler */                                                                        \                                                                                                                                                  
                "ori r24, 1                      \n"   /* Store the code for 3us */                                                             \
                "rjmp endcode_" id "             \n"   /* goto the end */                                                                       \
                                                                                                                                                  \
                "handle2us_" id ":               \n"   /* 2us Handler */                                                                        \
                "sbis  0x1c, 0                   \n"   /* 1/3 if (!(EIFR&bit(INTF0))) */                                                        \
                "rjmp endcode_" id "             \n"   /* Goto the end if the INDEX pulse is not detected */                                    \
                "sbi 0x1c, 0                     \n"   /* 2 Reset the register */                                                               \
                "ori r24, 3                      \n"   /* Save the special code code for what we discovered */                                  \
                                                                                                                                                  \
                "endcode_" id ":                 \n"   /* the end */                                                                            \
                "wdr                             \n"   /* Reset the watchdog timer */                                                           \  
                "lds  r20, 0x0060                \n"   /* read WDTCSR  */                                                                       \              
                "ori  r20, 0x40                  \n"   /* bit(WDIE)  */                                                                         \              
                "sts  0x0060, r20                \n"   /* write WDTCSR  */                                                                      \    
              );


// This is a nasty way to unroll the FOR loop.  
#define READ_UNROLLED_LOOP_HD_BITMODE_DRAWBRIDGE_PLUS()                                    \
            while ((!(TIFR1 & bit(ICF1)))&&(!(TIFR0 & bit(OCF0B)))) {}                      \
            tmp = ICR1L; counter = tmp - lastCounter, lastCounter = tmp;                    \
            TIFR1 |= bit(ICF1);                                                             \
            counter += pllValue;                                                            \  
            total += 7-(counter & 0x0F);                                                    \          
            counter = (counter / 16) & 15;                                                  \
            if (counter < 2) {                                                              \
              if ((EIFR&bit(INTF0))) {                                                      \
                EIFR|=bit(INTF0);                                                           \
                DataOutputByte |= 3;                                                        \
              }                                                                             \
            } else {                                                                        \
              if (counter == 2) DataOutputByte |= 1; else DataOutputByte |= 2;              \
            }                                                                               \
            TCNT0 = 0;                                                                      \
            TIFR0 |= bit(OCF0B);                                                            \


#define READ_UNROLLED_LOOP_HD_ASM_BITMODE_DRAWBRIDGE_PLUS(id)                                                                                     \
            asm("sbic  %0, %1                     \n"   /* fetch TIFR1 & bit(ICF1) and skip the next line if its clear */                       \
                "rjmp  capturedata_" id "         \n"   /* skip other test */                                                                   \
                "sbis  %2, %3                     \n"   /* fetch (TIFR0 & bit(OCF0B) and skip the next line if its set */                       \
                "rjmp  .-8                        \n"   /* start checking again */                                                              \
                                                                                                                                                  \
                "capturedata_" id ":              \n"   /* start of capture code */                                                             \
                                                                                                                                                  \
                "lds r21, %4                      \n"   /* Copy ICR1L into r21 (0x0086) */                                                      \ 
                "mov r20, r21                     \n"   /* take a copy of this value */                                                         \
                "sub r20, r25                     \n"   /* subtract previous timer value */                                                     \
                "mov r25, r21                     \n"   /* store new timer value */                                                             \
                                                                                                                                                  \                
                "sbi %0, %1                       \n"   /* Clear capture flag TIFR1 |= bit(ICF1); */                                            \
                                                                                                                                                  \                
                /* At this point, r20=time difference since last bit-cell, r25 is the time of this bit-cell (and 21 is a copy) */                 \
                                                                                                                                                  \
                "add r20, r22                    \n"   /* -8 (by default) */                                                                    \
                                                                                                                                                  \
                "mov r21, r20                    \n"   /* Take a copy of the value that has the +/- pll */                                      \
                "andi r21, 0x0F                  \n"   /* the lower nybble.  This is the offset within the 16 ticks */                          \
                "subi r28, -7                    \n"   /* Add 7 to r28 (total) */                                                               \
                "sub r28, r21                    \n"   /* Subtract from r28, r21 the offset */                                                  \
                                                                                                                                                  \
                "swap r20                        \n"   /* 1 Tick,   swap the nybbles */                                                         \
                "andi r20, 15                    \n"   /* keep the last 4 bits */                                                               \
                "cpi r20, 2                      \n"   /* compare with 2 */                                                                     \
                "brlo handle2us_" id "           \n"   /* If less than 2, then bramch to the 2us handler */                                     \
                                                                                                                                                  \
                "breq handle3us_" id "           \n"   /* If equal to 2, branch to the 3us handler */                                           \
                "ori r24, 2                      \n"   /* Store the code for 4us */                                                             \
                "rjmp endcode_" id "             \n"   /* goto the end */                                                                       \
                                                                                                                                                  \
                "handle3us_" id ":               \n"   /* 3us Handler */                                                                        \                                                                                                                                                  
                "ori r24, 1                      \n"   /* Store the code for 3us */                                                             \
                "rjmp endcode_" id "             \n"   /* goto the end */                                                                       \                
                                                                                                                                                  \
                "handle2us_" id ":               \n"   /* 2us Handler */                                                                        \
                "sbis  %5, %6                    \n"   /* 1/3 if (!(EIFR&bit(INTF0))) 0x1c */                                                   \
                "rjmp endcode_" id "             \n"   /* Goto the end if the INDEX pulse is not detected */                                    \
                "sbi %5, %6                      \n"   /* 2 Reset the register */                                                               \
                "ori r24, 3                      \n"   /* Save the special code code for what we discovered */                                  \
                                                                                                                                                  \
                "endcode_" id ":                 \n"   /* the end */                                                                            \
                "sbi %2, %3                      \n"   /* Reset overflow flag */                                                                \
                "out %7, r26                     \n"   /* 1 Tick,   Reset Timer0 value */                                                       \
              :: "i" _SFR_IO_ADDR(TIFR1), "i" (ICF1), "i" _SFR_IO_ADDR(TIFR0), "i" (OCF0B), "m" (ICR1L), "i" _SFR_IO_ADDR(EIFR), "i" (INTF0), "i" _SFR_IO_ADDR(TCNT0)  : );

#ifdef USE_ASM

// This doesnt affect any of the special flag registers so dont need to push or pop anything
ISR(WDT_vect, ISR_NAKED) {
    asm("ldi r25, 0xFF          \n"       // Set timed out register
         "sbi %0, %2            \n"       // OUTPUT Mode
         "cbi %1, %2            \n"       // LOW
         "cbi %0, %2            \n"       // INPUT Mode
         "sbi %1, %2            \n"       // Pullup
         "reti                  \n"       // And return
         :: "i"_SFR_IO_ADDR(PIN_READ_DATA_IO_DIR), "i"_SFR_IO_ADDR(PIN_READ_DATA_PORT_WRITE), "i"(PIN_READ_DATA): );  // PIN_READ_DATA isnt strictly correct
}
   
#else
volatile bool timedOut = false;
// Used to detect lockups because of no data or disk removed in DrawBridge Classic mode HD reading
ISR(WDT_vect) {
   // When the timer is triggered
   timedOut = true; 
   PIN_READ_DATA_IO_DIR |= PIN_READ_DATA_MASK;  // OUTPUT Mode
   PIN_READ_DATA_PORT_WRITE &= ~PIN_READ_DATA_MASK;  // LOW
   PIN_READ_DATA_IO_DIR &= ~PIN_READ_DATA_MASK;  // INPUT Mode
   PIN_READ_DATA_PORT_WRITE |= PIN_READ_DATA_MASK;  // Pullup
}
#endif

// Read the track using a timings to calculate which MFM sequence has been triggered, hwoever, this keeps running until a byte is received from the serial port telling it to stop
void readContinuousStreamHD() {
    // Configure timer 2 just as a counter in NORMAL mode
    TCCR0A = 0 ;              // No physical output port pins and normal operation
    TCCR0B = bit(CS20);       // Precale = 1 (ie: no prescale)
    EICRA = bit(ISC01);       // falling edge of INT0 generates an interrupt, they are turned off, but its an easy way for us to detect a falling edge rather than monitoring a pin
    OCR0A = 0x00;
    OCR0B = 0x00;
    // First wait for the serial port to be available to receive
    while(!(UCSR0A & (1<<UDRE0)));   
   
    // Signal we're active
    digitalWrite(PIN_ACTIVITY_LED,HIGH);
  
    // Index alignment won't work if we're already at the index when this starts.  The index pulse is actually quite long and spans several bytes of data, could be several ms in length
    while (!(PIN_INDEX_PORT & PIN_INDEX_MASK)) {};

    EIFR |=bit(INTF0);  // clear the register saying it detected an index pulse
    TIFR0 |= bit(OCF2B);  // clear the overflow register   
    TCNT0 = 0;  // reset

    if (drawbridgePlusMode) {
      TCCR1A = 0;
      TCCR1B = bit(CS10); // Capture from pin 8, falling edge. falling edge input capture, prescaler 1, no output compare
      TCCR1C = 0;

#ifdef USE_ASM
      // Start, and protect
      asm("push r20             \n"  // counter value after manipulations
          "push r21             \n"  // Temp variable
          "push r22             \n"  // PLL Offset Value
          "push r23             \n"  // UART status
          "push r24             \n"  // DataOutputByte
          "push r25             \n"  // lastCounter
          "push r26             \n"  // Zero
          "push r28             \n"  // Total
          "ldi r22, %0          \n"  // Load the PLL default of -8 which shoudl be bang in the middle
          "lds r25, %1          \n"  // Grab something like the current value from ICR1L (0x0086)
          "sbi %2, %3            \n" // And clear the capture flag (ICF1 in TIFR1)
          "ldi r26, 0           \n"  // Clear r26
          "ldi r28, 0           \n"  // Clear r28 (total)
          "loophere2:           \n"
        :: "i" (PLL_HD_START_VALUE), "m" (ICR1L), "i" _SFR_IO_ADDR(TIFR1), "i" (ICF1) :);
  
      // 5 including loop
      asm("add r24, r24                    \n"   /* SHL 1 */ 
          "add r24, r24                    \n"); /* SHL 1 */       

#ifdef ENABLE_PLL_DRAWBRIDGE_PLUS
      asm("cpi r28, %0                     \n"   // Compare total to PLL_HD_THRESHOLD
          "brlt skip_plus_PLL1             \n"   // skip the following code
          "ldi r28, 0                      \n"   // total = 0
          "subi r22, 0xFF                  \n"   // pll++
          "skip_plus_PLL1:                 \n" 
          :: "i" (PLL_HD_THRESHOLD+1) : );
#endif
           
      READ_UNROLLED_LOOP_HD_ASM_BITMODE_DRAWBRIDGE_PLUS("plusl1");       
  
      // 5-7 Ticks
      asm(
          "sts %0, r24      \n"  // 2 Ticks  UDR0 = DataOutputByte (0x00C6)
          "ldi r24, 0           \n"    // 1 Tick   DataOutputByte=0;
          :: "m" (UDR0) : );  // Next byte time
      
      READ_UNROLLED_LOOP_HD_ASM_BITMODE_DRAWBRIDGE_PLUS("plusl2");
  
      // 2 ticks
      asm("add r24, r24                    \n"   /* SHL 1 */ 
          "add r24, r24                    \n"   /* SHL 1 */    
          ); 

#ifdef ENABLE_PLL_DRAWBRIDGE_PLUS
      asm("cpi r28, %0                     \n"   // Compare total to -PLL_HD_THRESHOLD
          "brge skip_plus_PLL2             \n"   // skip the following code
          "ldi r28, 0                      \n"   // total = 0
          "subi r22, 1                     \n"    // pll--
          "skip_plus_PLL2:                 \n"
          :: "i" (-PLL_HD_THRESHOLD) : );          
#endif

      READ_UNROLLED_LOOP_HD_ASM_BITMODE_DRAWBRIDGE_PLUS("plusl3");
      asm(
          "lds r23, %0                     \n"  // 2 ticks - r23 = UCSR0A
          "add r24, r24                    \n"   /* SHL 1 */ 
          "add r24, r24                    \n"   /* SHL 1 */
          :: "m" (UCSR0A) :);
      
      READ_UNROLLED_LOOP_HD_ASM_BITMODE_DRAWBRIDGE_PLUS("plusl4");
      
      asm("sbrs r23, %0          \n"   // 1 tick, skip if set (RXC0 in UART status)
          "rjmp loophere2       \n"    // 2 ticks
          "pop r28              \n\r"
          "pop r26              \n\r"
          "pop r25              \n\r"
          "pop r24              \n\r"
          "pop r23              \n\r"
          "pop r22              \n\r"
          "pop r21              \n\r"
          "pop r20              \n\r"
          :: "i" (RXC0) : );
#else
      register unsigned char DataOutputByte = 0;
      register unsigned char counter;
      register unsigned char tmp;
      register char pllValue = PLL_HD_START_VALUE;
      register char total = 0;

      unsigned char lastCounter = ICR1L;
      TIFR1 |= bit(ICF1);

      // This implements a basic PLL to help with really bad disks
      for (;;) {      
        DataOutputByte<<=2; 

#ifdef ENABLE_PLL_DRAWBRIDGE_PLUS        
        if (total>PLL_HD_THRESHOLD) {
          total = 0;
          pllValue++;
        }
#endif        
        READ_UNROLLED_LOOP_HD_BITMODE_DRAWBRIDGE_PLUS();   
        
        UDR0 = DataOutputByte, DataOutputByte=0; 
        READ_UNROLLED_LOOP_HD_BITMODE_DRAWBRIDGE_PLUS();
        
        DataOutputByte<<=2;
#ifdef ENABLE_PLL_DRAWBRIDGE_PLUS        
        if (total<-PLL_HD_THRESHOLD) {
          total = 0;
          pllValue--;
        }
#endif        

        READ_UNROLLED_LOOP_HD_BITMODE_DRAWBRIDGE_PLUS();
        
        if ((UCSR0A & ( 1 << RXC0 ))) break;
        DataOutputByte<<=2;      
        READ_UNROLLED_LOOP_HD_BITMODE_DRAWBRIDGE_PLUS();
      }
#endif      

      TCCR1B = 0;
    } else {
      // In non DB+ mode, we can't add a timeout for reading as it makes the jitter worse.
      // But we can use the Watchdog Timer, which will trigger an Interrupt when the timer expires!
            
      while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) {};

      // This sets up what would be an interrupt for when the READ PIN is signalled (unfortunatly we can't choose the direction. Its just set when it changes)
      PCMSK0 = 0;
      PCMSK1 = 0;
      PCMSK2 = bit(PCINT20);
      PCIFR |= bit(PCIF2); 
      WDTCSR |= bit(WDCE) | bit(WDE);
      WDTCSR = bit(WDIE) | bit(WDP1);   // enable watchdog timer interrupt, for 64ms.  There shouldnt be a gap that long

  #ifdef USE_ASM
  
      // Start, and protect
      asm("push r20             \n"  // counter value
          "push r21             \n"  // temp variable
          "push r22             \n"  // Counter reset value (pll)
          "push r23             \n"  // UART status
          "push r24             \n"  // DataOutputByte
          "push r25             \n"  // Timed out
          "push r28             \n"  // Total
          "ldi r22, %0          \n"  
          "ldi r28, %0          \n"  
          "ldi r25, 0           \n"  // Reset timed out
          "sei                  \n"  // Start interrupts
          "loophere:            \n"
        :: "i" (TIMING_OVERHEAD + DB_CLASSIC_HD_MIDDLE) :);
  
      // 5 including loop
      asm("add r24, r24                    \n"   /* SHL 1 */ 
          "add r24, r24                    \n"); /* SHL 1 */       
  
      READ_UNROLLED_LOOP_HD_ASM_BITMODE("l1");       
  
      // 5-7 Ticks
      asm(
         "sts %0, r24           \n"  // 2 Ticks  UDR0 = DataOutputByte
         "ldi r24, 0            \n"    // 1 Tick   DataOutputByte=0;
         :: "m" (UDR0) : );  // Next byte time
      
      READ_UNROLLED_LOOP_HD_ASM_BITMODE("l2");
  
      // 2 ticks
      asm("add r24, r24                    \n"   /* SHL 1 */ 
          "add r24, r24                    \n"   /* SHL 1 */    
          ); 
                   
      READ_UNROLLED_LOOP_HD_ASM_BITMODE("l3");
      asm(
          "lds r23, %0                    \n"  // 2 ticks r23=UCSR0A
          "add r24, r24                    \n"   /* SHL 1 */ 
          "add r24, r24                    \n"   /* SHL 1 */
          :: "m" (UCSR0A) : );
      
      READ_UNROLLED_LOOP_HD_ASM_BITMODE("l4");

      // Check for timeout
      asm("sbrc r25, 0          \n"    // Skip the line if the bit is clear.  We chage it to 255 if its timed out
          "rjmp timedOut        \n");  // 2 ticks
      
      asm("sbrs r23, %0         \n"    // 1 tick, skip if set (RXC0 in UART status)
          "rjmp loophere        \n"    // 2 ticks
          "timedOut:            \n"    // oh dear
          "cli                  \n"    // turn off interrupts
          "pop r28              \n\r"
          "pop r25              \n\r"
          "pop r24              \n\r"
          "pop r23              \n\r"
          "pop r22              \n\r"
          "pop r21              \n\r"
          "pop r20              \n\r"
          :: "i" (RXC0) : );
  
  #else
      register unsigned char DataOutputByte = 0;
      register unsigned char counter;
      register unsigned char tmp;
      register char total = 0;

      bool timedOut = false;
      sei();
    
      for (;;) {   
        DataOutputByte<<=2;

        READ_UNROLLED_LOOP_HD_BITMODE();  
        
        UDR0 = DataOutputByte, DataOutputByte=0;
        
        READ_UNROLLED_LOOP_HD_BITMODE();
        DataOutputByte<<=2;

        READ_UNROLLED_LOOP_HD_BITMODE();
        
        if ((UCSR0A & ( 1 << RXC0 ))) break;
        DataOutputByte<<=2;      
        
        READ_UNROLLED_LOOP_HD_BITMODE();
        
        if (timedOut) break;
      }
      cli();
  #endif
      // Turn off WDT 
      WDTCSR |= bit(WDCE) | bit(WDE);
      WDTCSR = 0x00;

      PCMSK2 = 0;
    }

    // Read the byte that was sent to stop us, should be a 0, although we don't care
    unsigned char response = UDR0;
    // We want to make sure the PC knows we've quit, and whilst this isnt fool proof its a start.  
    // The chance of this exact sequence coming from MFM data from the drive is slim I guess
    // A little hacky, bit without woriding another pin to something we dont have any other options
    writeByteToUART('X');
    writeByteToUART('Y');
    writeByteToUART('Z');
    writeByteToUART(response);
    writeByteToUART('1');
    
    // turn off the status LED
    digitalWrite(PIN_ACTIVITY_LED,LOW);

    // Disable the counter
    TCCR0B = 0;      // No Clock (turn off)    
    EICRA = 0;      // disable monitoring   
    OCR0A = 0;
    OCR0B = 0;
}

// Attempt to calculate the RPM of the drive
void handleMeasureRPM() {
    // Configure timer 2 just as a counter in NORMAL mode
    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20);       // Precale = 1  
    OCR2A = 0x00;
    OCR2B = 0x00;
   
    // Signal we're active
    digitalWrite(PIN_ACTIVITY_LED,HIGH);

    // Wait for the index pulse
    while (PIN_INDEX_PORT & PIN_INDEX_MASK)  {};
    // And then for it to go away again
    while (!(PIN_INDEX_PORT & PIN_INDEX_MASK))  {};

    TCNT2=0;       // Reset the counter

    char rotations = 0;
    bool isIndexHigh = false;
    unsigned char lastTCNT2 = 0;
    unsigned long overflowCounter = 0;
    const unsigned char TotalRevolutions = 5;
    while (rotations<TotalRevolutions) {      
      if (isIndexHigh) {
        if (!(PIN_INDEX_PORT & PIN_INDEX_MASK)) {
          isIndexHigh = false;
          rotations++;
        }        
      } else {
        if (PIN_INDEX_PORT & PIN_INDEX_MASK) {
          isIndexHigh = true;
        }
      }   

      // Monitor TCNT2.
      unsigned char nextTCNT2 = TCNT2;
      if (nextTCNT2<lastTCNT2) {
        // Overflow occured
        overflowCounter++;
      }  
      lastTCNT2 = nextTCNT2;
    }

    // Calculate the time per revolution in 'ticks'
    unsigned long totalTicks = ((overflowCounter * 256) + lastTCNT2) / TotalRevolutions;
    float rpm = F_CPU / (totalTicks / 60.0f);

    // Now output the float
    char strOutput[10];
    dtostrf(rpm, 4, 2, strOutput);
    sendString(strOutput);
    sendString("\n");
    
    // turn off the status LED
    digitalWrite(PIN_ACTIVITY_LED,LOW);

    // Disable the counter
    TCCR2B = 0;      // No Clock (turn off)   
}

// Perform a manual test to work out the density of the medium inserted.  This also works as a manual 'detect disk' - Updated for Drawbridge Plus
void testDiskDensity() {    
     writeByteToUART('1');
     
    // This is a short-cut used by the slimline drives
    if ((disableDensityDetection) || (digitalRead(PIN_HD) == LOW)) {
        writeByteToUART('D');
        return;
    }
    
    // This is much harder
    startDriveForOperation();

    // Configure timer 2 just as a counter in NORMAL mode
    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20);       // Precale = 1 (ie: no prescale)
    EICRA = bit(ISC01);       // falling edge of INT0 generates an interrupt, they are turned off, but its an easy way for us to detect a falling edge rather than monitoring a pin
    OCR2A = 0x00;
    OCR2B = 0x00;

    TCNT2 = 0;  // reset
    if (drawbridgePlusMode) {
       while (!(PIN_READ_DATA_PLUSMODE_PORT & PIN_READ_DATA_PLUSMODE_MASK)) {};
    } else {
       while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) {};
    }

    EIFR |=bit(INTF0);  // clear the register saying it detected an index pulse
    TIFR2 |= bit(OCF2B);  // clear the overflow register
    OCR2B = TIMING_DD_UPPER_8us; // This is set to the upper bound.  If we exceed this we must have received a load of non-flux data, this allows us to write '0000' on the PC and loop back round

    // This sets up what would be an interrupt for when the READ PIN is signalled (unfortunatly we can't choose the direction. Its just set when it changes)
    // But this allows us to make sure we dont miss a bit, although the timing might be off slightly.  This is mainly used when disks have very long areas of
    // no flux transitions, typilcally used for copy protection
    PCMSK0 = 0;
    PCMSK1 = 0;
    PCMSK2 = bit(PCINT20);

    unsigned int overflows = 0;
    unsigned char lastTCNT2;

    unsigned int HDPulses = 0;
    unsigned int DDPulses = 0;
    unsigned char counter;
    unsigned int errors = 0;

    // Thats a maximum of 1 second of overflows which should be enough for the disk to spin up and start sending data
    const unsigned int MAX_OVERFLOWS = driveEnabled ? ((F_CPU / 256) / 4) : (F_CPU / 256);

    if (!driveEnabled) {
       digitalWrite(PIN_DRIVE_ENABLE_MOTOR,LOW);
       smalldelay(350);
    }

    // Don't need the higher accuracy here, so just a modified version of the loop
    if (drawbridgePlusMode) {
      TCCR1A = 0;
      TCCR1B = bit(CS10); // Capture from pin 8, falling edge. falling edge input capture, prescaler 1, no output compare
      TCCR1C = 0;
      unsigned char lastCounter = ICR1L;
      
      while (overflows < MAX_OVERFLOWS) {
         while ((!(TIFR1 & bit(ICF1)))&&(!(TIFR2 & bit(OCF2B)))) {};
         TCNT2 =  0;
         if (TIFR2 & bit(OCF2B)) {
           TIFR2 |= bit(OCF2B);
           overflows++;
         } else {
           const unsigned char tmp = ICR1L;
           counter = tmp - lastCounter, lastCounter = tmp;
           TIFR1 |= bit(ICF1);
           if ((counter > 8) && (counter <= (TIMING_HD_UPPER_3us - TIMING_OVERHEAD))) HDPulses++; else
           if ((counter >  (TIMING_DD_UPPER_4us - TIMING_OVERHEAD + 8)) && (counter <= (TIMING_DD_UPPER_8us - TIMING_OVERHEAD))) DDPulses++; else
               errors++;         
           if ((HDPulses>6000) || (DDPulses>6000) || (errors > 18000)) break;
         }
      }
      TCCR1B = 0;
      
    } else {
      while (overflows < MAX_OVERFLOWS) {
         while ((!(PCIFR & bit(PCIF2)))&&(!(TIFR2 & bit(OCF2B)))) {};
         counter = TCNT2, TCNT2 =  0;  
         while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) {};
         PCIFR |= bit(PCIF2);
         if (TIFR2 & bit(OCF2B)) {
           TIFR2 |= bit(OCF2B);
           overflows++;
         } else {
          if ((counter > 8) && (counter <= TIMING_HD_UPPER_3us)) HDPulses++; else
          if ((counter >  TIMING_DD_UPPER_4us + 8) && (counter <= TIMING_DD_UPPER_8us)) DDPulses++; else
            errors++;      
          if ((HDPulses>6000) || (DDPulses>6000) || (errors > 18000)) break;
         }
      }
    }

    if (!driveEnabled) {
      digitalWrite(PIN_DRIVE_ENABLE_MOTOR,HIGH);
    }    

    if ((HDPulses < 10) && (DDPulses < 10)) writeByteToUART('x'); else // no disk
    if (HDPulses > DDPulses) writeByteToUART('H'); else writeByteToUART('D');
   
    stopDriveForOperation();

    // Disable the counter
    TCCR2B = 0;      // No Clock (turn off)    
    EICRA = 0;      // disable monitoring   
    PCMSK2 = 0;
    OCR2A = 0;
    OCR2B = 0;
}

// Attempt to do a full reset
void doReset() {
  currentTrack = -1;
  driveEnabled  = 0;
  disktypeHD = 0;

  cli();
  TIMSK0=0;
  TIMSK1=0;
  TIMSK2=0;
  PCICR = 0;
  PCIFR = 0;
  PCMSK0 = 0;
  PCMSK2 = 0;
  PCMSK1 = 0;

  digitalWrite(PIN_SELECT_DRIVE,HIGH);
  digitalWrite(PIN_WRITE_DATA, HIGH);
  digitalWrite(PIN_WRITE_GATE, HIGH);
  digitalWrite(PIN_SELECT_DRIVE,LOW);
  digitalWrite(PIN_DRIVE_ENABLE_MOTOR,HIGH);
  digitalWrite(PIN_HEAD_SELECT,LOW);    
  digitalWrite(PIN_MOTOR_STEP, HIGH);
  digitalWrite(PIN_ACTIVITY_LED,LOW);
  digitalWrite(PIN_SELECT_DRIVE,HIGH);
  refreshEEPROMSettings();
  
  writeByteToUART('1');
}

#define DRAWBRIDGE_BETA_RELEASE        0

#define FLAGS_HIGH_PRECISION_SUPPORT   B00000001     // change to how the DD data is sent
#define FLAGS_DISKCHANGE_SUPPORT       B00000010     // Does the drive support diskchange
#define FLAGS_DRAWBRIDGE_PLUSMODE      B00000100     // Is this a Drawbridge Plus board
#define FLAGS_DENSITYDETECT_ENABLED    B00001000     // Is high denstiy detection disabled? (if so its always reported as double density)
#define FLAGS_SLOWSEEKING_MODE         B00010000     // If slow seeking is enabled
#define FLAGS_INDEX_ALIGN_MODE         B00100000     // Is INDEX ALIGN always enabled?
#define FLAGS_FLUX_READ                B01000000     // Flux-level read mode
#define FLAGS_FIRMWARE_BETA            B10000001     // Is this beta firmware?

// The main command loop
void loop() { 
    PIN_CTS_PORT &= (~PIN_CTS_MASK);            // Allow data incoming
    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;   // always turn writing off    
  
    // Read the command from the PC
    byte command = readByteFromUART();

    digitalWrite(PIN_SELECT_DRIVE, LOW);
    smalldelay(1);

    switch (command) {        
        case 'x': break; // this is ignored.  It's to help 'stop streaming' mode if it gets stuck on startup
        case 'R': doReset(); break;
        case 'M': if (!driveEnabled) sendString("Drive motor not switched on.\n"); else measureCurrentDisk(); break;
        case 'P': if (driveEnabled) { 
                      writeByteToUART('1');
                      handleMeasureRPM(); 
                  } else writeByteToUART('0');
                  break;
        case 'T': testDiskDensity(); break;
        case 't': if (disableDensityDetection) writeByteToUART('D'); 
                      else 
                        if (digitalRead(PIN_HD) == LOW) writeByteToUART('D'); else writeByteToUART('H'); 
                  break;
            
        // Command: "?" Means information about the firmware
        case '?':writeByteToUART('1');  // Success
                 writeByteToUART('V');  // Followed
                 writeByteToUART('1');  // By
                 writeByteToUART(advancedControllerMode ? ',' : '.');  // Advanced controller version
                 writeByteToUART('9');  // Number
                 break;
        case '@': writeByteToUART('1');  // Success
                  writeByteToUART(FLAGS_HIGH_PRECISION_SUPPORT | FLAGS_FLUX_READ |
                                  (DRAWBRIDGE_BETA_RELEASE ? FLAGS_FIRMWARE_BETA : 0) | 
                                  (drawbridgePlusMode ? FLAGS_DRAWBRIDGE_PLUSMODE : 0) | 
                                  (advancedControllerMode ? FLAGS_DISKCHANGE_SUPPORT : 0) | 
                                  (disableDensityDetection ? 0 : FLAGS_DENSITYDETECT_ENABLED) |
                                  (slowerDiskSeeking ? FLAGS_SLOWSEEKING_MODE : 0)  |
                                  (alwaysIndexAlignWrites ? FLAGS_INDEX_ALIGN_MODE : 0)  
                                  );
                  writeByteToUART(0);  // RFU
                  writeByteToUART(25);  // build number
                  break;
  
        // Command "." means go back to track 0
        case '.':if (goToTrack0())    // reset 
                      writeByteToUART('1');
                 else writeByteToUART('0');
                 break;                 
  
        // Command "#" means goto track.  Should be formatted as #00 or #32 etc
        case '#': if (gotoTrackX(false)) {
                      smalldelay(1); // wait for drive
                      writeByteToUART('1');
                  } else writeByteToUART('0'); 
                  break;

        // Command "=" means goto track.  Should be formatted as =00 or =32 etc.  This also reports disk change and write protect status
        case '=': if (gotoTrackX(true)) { 
                               
                  } else writeByteToUART('0'); 
                  break;  

        case 'O': handleNoClickSeek();
                  break;
  
        // Command "[" select LOWER disk side
        case '[': digitalWrite(PIN_HEAD_SELECT,LOW);
                  writeByteToUART('1');
                  break;
  
        // Command "]" select UPPER disk side
        case ']': digitalWrite(PIN_HEAD_SELECT,HIGH);
                  writeByteToUART('1');
                  break;

        // Command "{" Read data continuously from the drive until a byte is sent from the PC
        case '{': if (driveEnabled) {
                    if(disktypeHD) {
                      writeByteToUART('1');   
                      readContinuousStreamHD();
                    } else {
                      writeByteToUART('1');
                      readContinuousStream(false);
                    }
                   } else writeByteToUART('0');
                   break;

        case 'l': if (driveEnabled) {
                    if(disktypeHD) {
                      writeByteToUART('1');   
                      readContinuousStreamHD();
                    } else {
                      writeByteToUART('1');
                      readContinuousStreamPLL_ASM();
                    }
                   } else writeByteToUART('0');
                   break;


        // This reads raw flux from the drive - this is experimental
        case 'L':  if (driveEnabled) {
                    if(disktypeHD) {
                      writeByteToUART('1');   
                      readContinuousStreamHD();
                    } else {
                      writeByteToUART('1');
                      fluxStreamer();
                    }
                   } else writeByteToUART('0');
                   break;

        // Command "W" Write flux data
        case 'Y': if (!driveEnabled) writeByteToUART('0'); else {     
                     writeByteToUART('1');                     
                     writeFluxTrack();
                   }
                   break;    
        // Command "w" is Flux Wipe
        case 'w': if (!driveEnabled) writeByteToUART('0'); else {     
                     writeByteToUART('1');            
                     fluxWipe();
                   }
                   break;

        // Command "F" Read data continuously from the drive until a byte is sent from the PC higher accuracy flux.  
        case 'F':  if (driveEnabled) {
                    if(disktypeHD) {
                      writeByteToUART('0');   
                    } else {
                      writeByteToUART('1');
                      readContinuousStream(true);
                    }
                   } else writeByteToUART('0');
                   break;                   

        // Command "}" Write track to the drive with precomp
        case '}': if (!driveEnabled) writeByteToUART('0'); else {                 
                         if(disktypeHD) 
                           writeByteToUART('0');
                        else {
                            writeByteToUART('1');
                            writePrecompTrack();
                       }
                     }
                   break;                   
  
        // Command "<" Read track from the drive
        case '<': if(!driveEnabled) writeByteToUART('0'); 
                  else {
                     if(disktypeHD)
                         writeByteToUART('0');      // Not supported.  Use read contineous stream
                     else {
                        writeByteToUART('1');
                        readTrackDataFast();
                     }
                   }
                   break;
  
        // Command ">" Write track to the drive 
        case '>': if (!driveEnabled) writeByteToUART('0'); else {                     
                     if(disktypeHD) {
                        writeByteToUART('1');       
                        writeTrackFromUART_HD();               
                     } else {
                        writeByteToUART('1');
                        writeTrackFromUART();
                     }
                   }
                   break;

        // Command "X" Erase current track (writes 0xAA to it)
        case 'X': if (!driveEnabled) writeByteToUART('0'); else
                  {
                     writeByteToUART('1');
                     if (disktypeHD) 
                       eraseTrack_HD();
                     else
                       eraseTrack();
                  }
                  break;                   

        // Command "H" Set HD disk type
        case 'H': disktypeHD = 1;
                  writeByteToUART('1');                  
                  break;
          
        // Command "D" Set DD or SD disk type
        case 'D': disktypeHD = 0;
                  writeByteToUART('1');                  
                  break;
  
        // Turn off the drive motor
        case '-': digitalWrite(PIN_WRITE_GATE,HIGH);
                  digitalWrite(PIN_DRIVE_ENABLE_MOTOR,HIGH);
                  writeByteToUART('1');
                  driveEnabled = 0;
                  break;

       // Turn on the drive motor and setup in READ MODE, this has no delay, the computer must handle this
       case '*':  if (!driveEnabled) {
                     digitalWrite(PIN_DRIVE_ENABLE_MOTOR,LOW);
                     driveEnabled = 1;
                  }
                  writeByteToUART('1');
                  break;
  
       // Turn on the drive motor and setup in READ and WRITE MODE.  They both work the same now
       case '+':  
       case '~':  if (!driveEnabled) {
                     digitalWrite(PIN_DRIVE_ENABLE_MOTOR,LOW);
                     driveEnabled = 1;
                     smalldelay(750); // wait for drive
                  }
                  writeByteToUART('1');
                  break;

        // Check write protect flag
        case '$': checkWriteProtectStatus();
                  break;  

        // Ask if the drive is ready (has a disk in it) and if its write protected or not
        case '^': testForDisk(true);     
                  break;
                         
        case '&': runDiagnostic();
                  break;

        case 'E': readEpromValue(); break;
        case 'e': writeEpromValue(); break;
    
      // We don't recognise the command!
      default:
                 writeByteToUART('!'); // error
                 break;
     }

    if (!driveEnabled) {      
      digitalWrite(PIN_SELECT_DRIVE, HIGH);
      smalldelay(1);
    }
}
