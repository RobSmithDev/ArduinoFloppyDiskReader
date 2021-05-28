/* ArduinoFloppyReader (and writer)
*
* Copyright (C) 2017-2021 Robert Smith (@RobSmithDev)
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

/* Latest History: Last Modified: 22/04/2021
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
*/    

/////////////////////////////////////////////////////////////////////////////////////////////////////
// This sketch manages the interface between the floppy drive and the computer as well as the     //
// low-level disk reading and writing.  For more information and how to connect your Arduino     //
// to a floppy drive and computer visit https://amiga.robsmithdev.co.uk                         //
/////////////////////////////////////////////////////////////////////////////////////////////////
// This code doesnt actually do any decoding, and is mearly reading pulses, so can be used to //
// read data from other disk formats too.                                                    //
//////////////////////////////////////////////////////////////////////////////////////////////

// ** Hardware Modification Changes to get the best support for disk change notifications **
//    Pin 34 on the floppy drive connector (Disk Ready/Change) must be connected to Pin 10 on the Arduino
//    Pin 12 on the floppy drive connector (Select Disk B) must be *disconnected* from pin 16 on the Arduino and connected to Pin 12.  Note you *must* leave the connection between Arduino Pin 5 and Floppy Connector 16 in place
//    On the Arduino, connect Pin 12 to GND (0v) - this enables this advanced mode automatically.
//    If you can't connect pin12 to GND because you want to use the ISP headers, then see https://amiga.robsmithdev.co.uk/isp

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
#define PIN_READ_DATA            4          // Reads RAW floppy data on this pin
#define PIN_READ_DATA_MASK       B00010000  // The mask for the port
#define PIN_READ_DATA_PORT       PIND       // The port the above pin is on

// PIN 5, 6 and 7 - DRIVE, HEAD MOTOR DIRECTION and CONTROL
#define PIN_DRIVE_ENABLE_MOTOR   5        // Turn on and off the motor on the drive
#define PIN_MOTOR_DIR            6        // Stepper motor output to choose the direction the head should move
#define PIN_MOTOR_STEP           7        // Stepper motor step line to move the head position

// PIN 8 - Used to detect track 0 while moving the head
#define PIN_DETECT_TRACK_0       8        // Used to see if the drive is at track 0

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

// Reserved for future use
#define PIN_HD                   A3
#define PIN_RDY                  A4

// PIN 13 - Activity LED
#define PIN_ACTIVITY_LED         13       // Standard LED on Arduinos.  We're just using this as a read/write status flag

//  ** Hardware Modification Changes to get the best support for disk change notifications **  We control these regardless
#define PIN_DISK_CHANGE          10   // This is actually disk exists pin.  Some drives have a resistor that needs to be added/removed to get this operation.  This is usually the default on PC drives.
#define PIN_SELECT_DRIVE         11
#define PIN_DETECT_ADVANCED_MODE 12



// Detect advanced mode
bool advancedControllerMode     = false;   // DO NOT CHANGE THIS, it is automatically detected. If you can't connect pin12 to GND because you want to use the ISP headers, then see https://amiga.robsmithdev.co.uk/isp


// Paula on the Amiga used to find the SYNC WORDS and then read 0x1900 further WORDS.  
// A dos track is 11968 bytes in size, theritical revolution is 12800 bytes. 
/* The ATARI ST could format a track with up to 11 Sectors, so the AMIGA settings are OK. */
#define RAW_TRACKDATA_LENGTH   (0x1900*2+0x440)  // Paula assumed it was 12868 bytes, so we read that, plus the size of a sector, to find overlap

/* For the HD (1.4 MBytes) Disks the amount of data should be about 26688: */
#define RAW_HD_TRACKDATA_LENGTH   (0x1900*2*2+0x440)

// The current track that the head is over. Starts with -1 to identify an unknown head position.
int currentTrack = -1;

// If the drive has been switched on or not
bool driveEnabled  = 0;

/* Where there should be a HD Disk been read (1) or a DD and SD Disk (0).*/
bool disktypeHD = 0;

// The timings here could be changed.  These are based on F_CPU=16Mhz, which leaves the resolution at 1 tick = 0.0625usec, hence 16=1uSec

// There's approx 4 clock ticks on average between noticing the flux transition and the counter value being read/reset
#define TIMING_OVERHEAD               -4

// Calculate the bit-timing windows.  These are the ideal exact centre of the next flux transition since the previous.
#define TIMING_DD_MIDDLE_2us     (2 * 16)
#define TIMING_DD_MIDDLE_4us     (4 * 16)
#define TIMING_DD_MIDDLE_6us     (6 * 16)
#define TIMING_DD_MIDDLE_8us     (8 * 16)

// Work out the upper window of the timing.  Most PLL allow for about 5% drift, but we're not interested in that and just want to recover the data
#define TIMING_DD_UPPER_2us     (TIMING_DD_MIDDLE_2us + 16 + TIMING_OVERHEAD) 
#define TIMING_DD_UPPER_4us     (TIMING_DD_MIDDLE_4us + 16 + TIMING_OVERHEAD) 
#define TIMING_DD_UPPER_6us     (TIMING_DD_MIDDLE_6us + 16 + TIMING_OVERHEAD) 
#define TIMING_DD_UPPER_8us     (TIMING_DD_MIDDLE_8us + 16 + TIMING_OVERHEAD) 

// HD versions
#define TIMING_HD_UPPER_2us     ((TIMING_DD_MIDDLE_4us/2) + 8 + TIMING_OVERHEAD) 
#define TIMING_HD_UPPER_4us     ((TIMING_DD_MIDDLE_6us/2) + 8 + TIMING_OVERHEAD) 
#define TIMING_HD_UPPER_6us     ((TIMING_DD_MIDDLE_8us/2) + 8 + TIMING_OVERHEAD) 

// 256 byte circular buffer - don't change this, we abuse the unsigned char to overflow back to zero!
#define SERIAL_BUFFER_SIZE 256
#define SERIAL_BUFFER_START (SERIAL_BUFFER_SIZE-16)
unsigned char SERIAL_BUFFER[SERIAL_BUFFER_SIZE];




#include <EEPROM.h>

// Because we turned off interrupts delay() doesnt work! This is approx ms
void smalldelay(unsigned long delayTime) {
    delayTime*=(F_CPU/(8*1000L));
  
    for (unsigned long loops=0; loops<delayTime; ++loops) {
        asm volatile("nop\n\t"::);
    }
}

// Step the head once.  This seems to be an acceptable speed for the head
// Drive spec says pulse should be at least 3ms, but the time between pulses must be greater than 1us.  16 NOPS is approx 1us, so im just being cautious
void stepDirectionHead() {
    asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t"::);
    digitalWrite(PIN_MOTOR_STEP,LOW);
    smalldelay(3);    // drive pulse must be at least 3ms long
    digitalWrite(PIN_MOTOR_STEP,HIGH);
    asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t"::);
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
    
    // Setup the USART
    prepSerialInterface();
}

// Run a diagnostics test command
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

              for (unsigned int b=0; b<20; b++) {
                 for (unsigned int a=0; a<60000; a++) {
                      if (PIN_READ_DATA_PORT & PIN_READ_DATA_MASK) state1=true; else state2=true;
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

// Rewinds the head back to Track 0
bool goToTrack0() {
    startDriveForOperation();
    digitalWrite(PIN_MOTOR_DIR,MOTOR_TRACK_DECREASE);   // Set the direction to go backwards
    int counter=0;
    while (digitalRead(PIN_DETECT_TRACK_0) != LOW) {
       stepDirectionHead();   // Keep moving the head until we see the TRACK 0 detection pin
       smalldelay(1);         // slow down a little
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

// Goto to a specific track.  During testing it was easier for the track number to be supplied as two ASCII characters, so I left it like this
bool gotoTrackX(bool reportDiskChange) {
    // Read the bytes
    byte track1 = readByteFromUART();
    byte track2 = readByteFromUART();
    byte flags = 1;  // default to normal speed

    if (reportDiskChange) {
      flags = readByteFromUART()-'0';      
    }
    
    // Work so its compatiable with previous versions
    const unsigned char delayTime = 4 - (flags & 3);

    // Validate
    if ((track1<'0') || (track1>'9')) return false;
    if ((track2<'0') || (track2>'9')) return false;

    // Calculate target track and validate 
    int track = ((track1-'0')*10) + (track2-'0');
    if (track<0) return false;
    if (track>81) return false; // yes amiga could read track 81!

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
            stepDirectionHead();
            if (delayTime) smalldelay(delayTime);
            currentTrack++;         
        }
    } else {
        digitalWrite(PIN_MOTOR_DIR,MOTOR_TRACK_DECREASE);   // Move OUT
        while (currentTrack > track) {
            stepDirectionHead();
            if (delayTime) smalldelay(delayTime);
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

// Checks manually to see if theres a disk on un-modded hardware
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
        register unsigned char currentState = PIN_READ_DATA_PORT & PIN_READ_DATA_MASK;

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


#define CHECKSERIAL_ONLY() if (UCSR0A & bit(RXC0)) {                      \
                            SERIAL_BUFFER[serialWritePos++] = UDR0;     \
                            serialBytesInUse++;                         \
                          }   

// 12 is the minimum number here.  Any less than this and the CHECKSERIAL_ONLY() code will impact the output.  The pulse width doesn't matter as long as its at least 0.125uSec (its the falling edge that triggers a bit write)   
// Because only the falling edge is important we achieve precomp by shifting the pulse starting position back or forward two clock ticks      
// Because it may go back 2 ticks, we increase this number here by 2.  12 ticks is 750 ns, 14 ticks is 875 ns and 16 is 1000ns (1us) 
// By doing this, the bit cell timing remains constant, but the actual write position is shifted +/- 125ns as required
#define PULSE_WIDTH 14
// This is where the above starts from the end of the timer
#define PULSE_WIDTH_VALUE (0xFF - (PULSE_WIDTH-1))
// This is where to start the counter from compensating for code delay of 6 ticks (measured) 
#define PULSE_BREAK (58-PULSE_WIDTH) 

// This makes use of the PWM output to create the wayforms for us as accurate as possible.
void writePrecompTrack() {   
    // Check if its write protected.  You can only do this after the write gate has been pulled low
    if (digitalRead(PIN_WRITE_PROTECTED) == LOW) {
        writeByteToUART('N'); 
        return;
    } else
    writeByteToUART('Y');

    // Find out how many bytes they want to send
    unsigned char highByte = readByteFromUART();
    unsigned char lowByte = readByteFromUART();
    unsigned char waitForIndex = readByteFromUART();
    
    unsigned short numBytes = (((unsigned short)highByte)<<8) | lowByte;

    // Setup buffer parameters
    unsigned char serialReadPos = 0;
    unsigned char serialWritePos = SERIAL_BUFFER_START;
    unsigned char serialBytesInUse = SERIAL_BUFFER_START;
    digitalWrite(PIN_ACTIVITY_LED,HIGH);
    
    writeByteToUART('!');     
    
    PIN_CTS_PORT|=PIN_CTS_MASK;                // stop any more data coming in!
 
    // While the INDEX pin is high wait.  Might as well write from the start of the track
    if (waitForIndex) 
        while (PIN_INDEX_PORT & PIN_INDEX_MASK)  {};

    // Signal we're ready for data
    PIN_CTS_PORT &= (~PIN_CTS_MASK);   

    // Fill our buffer to give us a head start
    for (unsigned char a=0; a<SERIAL_BUFFER_START; a++) {
        // Wait for it
        while (!( UCSR0A & ( bit(RXC0)))) {}; 
        // Save the byte
        SERIAL_BUFFER[a] = UDR0;
    }

    // Block more data
    PIN_CTS_PORT|=PIN_CTS_MASK;  
    
    // Enable writing
    PIN_WRITE_GATE_PORT&=~PIN_WRITE_GATE_MASK;

    // Reset the counter, ready for writing - abuse the Fast PWM to produce the wayforms we need
    TCCR2A = bit(COM2B1) | bit(WGM20) | bit(WGM21)| bit(WGM22);  // (COM2B0|COM2B1) Clear OC2B. on compare match, set OC2B at BOTTOM.  WGM20|WGM21|WGM22 is Fast PWM. 
    TCCR2B = bit(WGM22)| bit(CS20);         // WGM22 enables waveform generation.  CS20 starts the counter runing at maximum speed

    // Get it ready    
    OCR2A = 0;                         // Because we're using the PWM limits backwards this causes it to get stuck when it reaches 0
    OCR2B = PULSE_WIDTH_VALUE;         // pulse width is set to default
    TCNT2 = PULSE_WIDTH_VALUE-96;      // trigger it to start. This will just sent a 0001 to start with
    TIFR2 |= bit(OCF2B);
    TIFR2 |= bit(TOV2);

    // Loop through all bytes of data required.  Each byte contains two sequences to write
    while (--numBytes>0) {
        if (!serialBytesInUse) break;

        // Read a byte from the buffer
        register unsigned char currentByte = SERIAL_BUFFER[serialReadPos++];
        serialBytesInUse--;
        register unsigned char counter = PULSE_WIDTH_VALUE - (PULSE_BREAK + ( (currentByte&0x03)    *32));
        register unsigned char pulseStart = PULSE_WIDTH_VALUE;
        if (currentByte & 0x04) pulseStart=PULSE_WIDTH_VALUE-2;    // Pulse should be early, so just move the pulse start back
        if (currentByte & 0x08) pulseStart=PULSE_WIDTH_VALUE+2;    // Pulse should be late, so move the pulse start forward

        // Hardware error checks (frame error and overrun)
        if (UCSR0A & (bit(FE0)|bit(DOR0))) break;

        // Run until the pulse starts.  The pulse start is also timed so that its width is enough to cover the time to execute CHECKSERIAL_ONLY()
        while (!(TIFR2 &  bit(OCF2B))) {
            CHECKSERIAL_ONLY();
        }; 

        // Wait for overflow (ie: pulse finishes)
        while (!(TIFR2 &  bit(TOV2)));
        // Set the new counter and clear all the overflows
        TCNT2 = counter;
        OCR2B = pulseStart;
        
        // Clear overflow flags
        TIFR2 |= bit(TOV2);
        TIFR2 |= bit(OCF2B);
        
        // Control I/O with the serial port
        if (serialBytesInUse<SERIAL_BUFFER_START) PIN_CTS_PORT &= (~PIN_CTS_MASK); else PIN_CTS_PORT|=PIN_CTS_MASK;  
        
        // Work out the next entry
        counter = PULSE_WIDTH_VALUE - (PULSE_BREAK + ( (currentByte&0x30) * 2));
        pulseStart = PULSE_WIDTH_VALUE;
        if (currentByte & 0x40) pulseStart=PULSE_WIDTH_VALUE-2;    // Pulse should be early, so just move the pulse start back
        if (currentByte & 0x80) pulseStart=PULSE_WIDTH_VALUE+2;    // Pulse should be late, so move the pulse start forward

        // Hardware error checks (frame error and overrun)
        if (UCSR0A & (bit(FE0)|bit(DOR0))) break;
        
        // Run until the pulse starts.  The pulse start is also timed so that its width is enough to cover the time to execute CHECKSERIAL_ONLY()
        while (!(TIFR2 &  bit(OCF2B))) {
            CHECKSERIAL_ONLY();
        }; 
           
        // Wait for overflow (ie: pulse finishes)
        while (!(TIFR2 &  bit(TOV2)));
        // Set the new counter and clear all the overflows
        TCNT2 = counter;
        OCR2B = pulseStart;
        
        // Clear overflow flags
        TIFR2 |= bit(TOV2);
        TIFR2 |= bit(OCF2B);
    }  
    
    // Wait for the pulse to finish
    while (!(TIFR2 &  bit(OCF2B)));
    while (!(TIFR2 &  bit(TOV2)));

    // Blank NOPS
    asm volatile("nop\n\t"::);
    asm volatile("nop\n\t"::);
    asm volatile("nop\n\t"::);
    asm volatile("nop\n\t"::);

    // Turn off the write head
    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;

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


#define CHECK_SERIAL()          if (UCSR0A & ( 1 << RXC0 )) {                      \
                                    SERIAL_BUFFER[serialWritePos++] = UDR0;        \
                                    serialBytesInUse++;                            \
                                }                                                  \   
                                if (serialBytesInUse<SERIAL_BUFFER_START)          \
                                    PIN_CTS_PORT &= (~PIN_CTS_MASK);               \
                                else PIN_CTS_PORT|=PIN_CTS_MASK;                   
                                
 
// Small Macro to write a '1' pulse to the drive if a bit is set based on the supplied bitmask
#define WRITE_BIT(value,bitmask) if (currentByte & bitmask) {                            \
                                     while (TCNT2<value) {};                             \
                                     PIN_WRITE_DATA_PORT&=~PIN_WRITE_DATA_MASK;          \
                                 } else {                                                \
                                     while (TCNT2<value) {};                             \
                                     PIN_WRITE_DATA_PORT|=PIN_WRITE_DATA_MASK;           \
                                 }                                                       
 
// Write a track to disk from the UART - the data should be pre-MFM encoded raw track data where '1's are the pulses/phase reversals to trigger
void writeTrackFromUART() {
    // Configure timer 2 just as a counter in NORMAL mode
    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20);       // Precale = 1  
    OCR2A = 0x00;
    OCR2B = 0x00;
    
    // Check if its write protected.  You can only do this after the write gate has been pulled low
    if (digitalRead(PIN_WRITE_PROTECTED) == LOW) {
        writeByteToUART('N'); 
        return;
    } else
    writeByteToUART('Y');

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
    PIN_WRITE_GATE_PORT&=~PIN_WRITE_GATE_MASK;

    // While the INDEX pin is high wait.  Might as well write from the start of the track
    if (waitForIndex) 
        while (PIN_INDEX_PORT & PIN_INDEX_MASK)  {};

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
    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;

    // Done!
    writeByteToUART('1');
    digitalWrite(PIN_ACTIVITY_LED,LOW);

    // Need to allow data to come in again
    PIN_CTS_PORT &= (~PIN_CTS_MASK);  
    
    TCCR2B = 0;   // No Clock (turn off)    
}

// Write a track to disk from the UART - the data should be pre-MFM encoded raw track data where '1's are the pulses/phase reversals to trigger
// THIS CODE IS UNTESTED
void writeTrackFromUART_HD() {
    // Configure timer 2 just as a counter in NORMAL mode
    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20);       // Precale = 1  
    OCR2A = 0x00;
    OCR2B = 0x00;
    
    // Check if its write protected.  You can only do this after the write gate has been pulled low
    if (digitalRead(PIN_WRITE_PROTECTED) == LOW) {
        writeByteToUART('N'); 
        return;
    } else writeByteToUART('Y');

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
    PIN_WRITE_GATE_PORT&=~PIN_WRITE_GATE_MASK;
    
    // While the INDEX pin is high wait.  Might as well write from the start of the track
    if (waitForIndex) 
        while (PIN_INDEX_PORT & PIN_INDEX_MASK)  {};

    // Reset the counter, ready for writing
    TCNT2=0;  
    
    // Loop them bytes - ideally I wanted to use an ISR here, but theres just too much overhead even with naked ISRs to do this (with preserving registers etc)
    for (register unsigned int a=0; a<numBytes; a++) {
        // Should never happen, but we'll wait here if theres no data 
        if (serialBytesInUse<1) {
            // This can;t happen and causes a write failure
            digitalWrite(PIN_ACTIVITY_LED,LOW);
            writeByteToUART('X');   // Thus means buffer underflow. PC wasn't sending us data fast enough
            PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;
            TCCR2B = 0;   // No Clock (turn off)                
            return;
        }

        // Read a buye from the buffer
        currentByte = SERIAL_BUFFER[serialReadPos++]; 
        serialBytesInUse--;

        
        // Theres a small possibility, looking at the decompiled ASM (and less likely even with these few extra instructions) we actually might get back here before the TCNT2 overflows back to zero causing this to write early
        while (TCNT2>=248) {}
    
        // Now we write the data.  Hopefully by the time we get back to the top everything is ready again
        WRITE_BIT(0x08,B10000000);
        CHECK_SERIAL();
        WRITE_BIT(0x18,B01000000);
        CHECK_SERIAL();
        WRITE_BIT(0x28,B00100000);
        CHECK_SERIAL();
        WRITE_BIT(0x38,B00010000);
        CHECK_SERIAL();
        WRITE_BIT(0x48,B00001000);
        CHECK_SERIAL();
        WRITE_BIT(0x58,B00000100);
        CHECK_SERIAL();
        WRITE_BIT(0x68,B00000010);
        CHECK_SERIAL();
        WRITE_BIT(0x78,B00000001);        
        TCNT2=248;   // a little cheating, but *should* work
        PIN_CTS_PORT|=PIN_CTS_MASK;   // Stop data coming in while we're not monitoring it
    }  
  
    // Turn off the write head
    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;

    // Done!
    writeByteToUART('1');
    digitalWrite(PIN_ACTIVITY_LED,LOW);
    
    TCCR2B = 0;   // No Clock (turn off)    
}


// Write blank data to a disk so that no MFM track could be detected - this is no longer used
void eraseTrack() {    
    // Check if its write protected.  You can only do this after the write gate has been pulled low
    if (digitalRead(PIN_WRITE_PROTECTED) == LOW) {
        writeByteToUART('N'); 
        return;
    } else writeByteToUART('Y');
    
    digitalWrite(PIN_ACTIVITY_LED,HIGH);
   
    // Enable writing
    PIN_WRITE_GATE_PORT&=~PIN_WRITE_GATE_MASK;

    // To write the 01010101 sequence we're going to ask the Arduino to generate this from its PWM output.
    TCCR2A = bit(COM2B1) | bit(WGM20) | bit(WGM21)| bit(WGM22);  // (COM2B0|COM2B1) Clear OC2B. on compare match, set OC2B at BOTTOM.  WGM20|WGM21|WGM22 is Fast PWM. 
    TCCR2B = bit(WGM22)| bit(CS20);         // WGM22 enables waveform generation.  CS20 starts the counter runing at maximum speed
    // This generates a square wave, 3usec high, and 1usec low, 4uSec in total
    OCR2A = 63;                         
    OCR2B = 47;                         
    TCNT2=0;  

    // Now just count how many times this happens.  Approx 200ms is a revolution, so we'll go 200ms + 5% to be on the safe side (210ms)
    TIFR2 |= bit(TOV2);

    // 52500 is 210 / 0.004
    for (unsigned int counter=0; counter<52500; counter++) {
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

// Write blank data to a disk so that no MFM track could be detected - this is no longer used
void eraseTrack_HD() {
    // Check if its write protected.  You can only do this after the write gate has been pulled low
    if (digitalRead(PIN_WRITE_PROTECTED) == LOW) {
        writeByteToUART('N'); 
        return;
    } else writeByteToUART('Y');
    
    digitalWrite(PIN_ACTIVITY_LED,HIGH);
   
    // Enable writing
    PIN_WRITE_GATE_PORT&=~PIN_WRITE_GATE_MASK;

    // To write the 01010101 sequence we're going to ask the Arduino to generate this from its PWM output.
    TCCR2A = bit(COM2B1) | bit(WGM20) | bit(WGM21)| bit(WGM22);  // (COM2B0|COM2B1) Clear OC2B. on compare match, set OC2B at BOTTOM.  WGM20|WGM21|WGM22 is Fast PWM. 
    TCCR2B = bit(WGM22)| bit(CS20);         // WGM22 enables waveform generation.  CS20 starts the counter runing at maximum speed
    // This generates a square wave, 1.5usec high, and 0.1usec low, 4uSec in total
    OCR2A = 31;                         
    OCR2B = 15;                         
    TCNT2=0;  

    // Now just count how many times this happens.  Approx 200ms is a revolution, so we'll go 200ms + 5% to be on the safe side (210ms)
    TIFR2 |= bit(TOV2);

    // 52500 is 210 / 0.004, but we're tqice as quick, so do the loop twice
    for (unsigned char loops=0; loops<2; loops++) 
      for (unsigned int counter=0; counter<52500; counter++) {
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

// Read the track using a timings to calculate which MFM sequence has been triggered
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
    // Because of the above rules the actual valid two-bit sequences output are 01, 10 and 11, so we use 00 to say "END OF DATA"
    writeByteToUART(0);

    // turn off the status LED
    digitalWrite(PIN_ACTIVITY_LED,LOW);

    // Disable the counter
    TCCR2B = 0;      // No Clock (turn off)    
}

// Read the track for a HD disk
void readTrackDataFast_HD() {
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
    long target = ((long)RAW_HD_TRACKDATA_LENGTH)*(long)8;

    while (totalBits<target) {
        for (register unsigned char bits=0; bits<4; bits++) {
            // Wait while pin is high
            
            while (PIN_READ_DATA_PORT & PIN_READ_DATA_MASK) {
            };
            counter = TCNT2, TCNT2 = 0;  // reset - must be done with a COMMA
           
            DataOutputByte<<=2;   
            
            // DO NOT USE BRACES HERE, use the "," or the optomiser messes it up
             if (counter<TIMING_HD_UPPER_2us) DataOutputByte|=B00000001,totalBits+=2; else    // this accounts for just a '1' or a '01' as two '1' arent allowed in a row
            if (counter<TIMING_HD_UPPER_4us) DataOutputByte|=B00000010,totalBits+=3; else            
            if (counter<TIMING_HD_UPPER_6us) DataOutputByte|=B00000011,totalBits+=4; else      
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
    // Because of the above rules the actual valid two-bit sequences output are 01, 10 and 11, so we use 00 to say "END OF DATA"
    writeByteToUART(0);

    // turn off the status LED
    digitalWrite(PIN_ACTIVITY_LED,LOW);

    // Disable the counter
    TCCR2B = 0;      // No Clock (turn off)    
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

// Drive disk statictcs (used to check #define at the top fo the code)
void measureCurrentDisk() {
  sendString("\n\nDrive Counters.  Current Parameters:\n");
  sendString("Timing Compensation Overhead in Ticks: ");
  if (TIMING_OVERHEAD<0) {
    writeByteToUART('-');
    sendInt(-TIMING_OVERHEAD);
  } else sendInt(TIMING_OVERHEAD);
  sendString("\nTicks for middle 4, 6 and 8 uS: ");
  sendInt(TIMING_DD_MIDDLE_4us); sendString(", "); sendInt(TIMING_DD_MIDDLE_6us); sendString(", "); sendInt(TIMING_DD_MIDDLE_8us); sendString("\n");
  sendString("Ticks for Upper bound for 4, 6 and 8 uS: ");
  sendInt(TIMING_DD_UPPER_4us); sendString(", "); sendInt(TIMING_DD_UPPER_6us); sendString(", "); sendInt(TIMING_DD_UPPER_8us); sendString("\n");
  sendString("\nBitcell timings for current track/side/disk: Testing...");
  
    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20);       // Precale = 1 (ie: no prescale)
    EICRA = bit(ISC01);       // falling edge of INT0 generates an interrupt, they are turned off, but its an easy way for us to detect a falling edge rather than monitoring a pin
    unsigned int cc[256];
    for (int a=0; a<256; a++) cc[a] = 0;

    // Wait for index pin
    while (!(EIFR&bit(INTF0))) {};
    EIFR=bit(INTF0);  // clear the register saying it detected an index pulse

    // Skip the first bit.  We're probably already half way through timing it
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
         while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) {};
     }
    }

    sendString(".Completed 2 revolutions.\n\n");
    sendString("Ticks,   uSec, Count, Identified as\n"); 

    int lastWindow = 0;
    
    for (int a=1; a<=255; a++) {
      if ((a>=TIMING_DD_UPPER_4us/2) && (lastWindow==0)) {
        lastWindow = 1;
        sendString("\n");
      }
      if ((a>=TIMING_DD_UPPER_4us) && (lastWindow==1)) {
        lastWindow = 2;
        sendString("\n");
      }
      if ((a>=TIMING_DD_UPPER_6us) && (lastWindow==2)) {
        lastWindow = 3;
        sendString("\n");
      }
      if ((a>=TIMING_DD_UPPER_8us) && (lastWindow==3)) {
        lastWindow = 4;
        sendString("\n");
      }
      if (cc[a]>1) {
        sendInt(a);
        sendString(", ");
        sendTickAsuSec(a);
        sendString(", ");
        sendInt(cc[a]);
        sendString(", ");

        if (a <= TIMING_DD_UPPER_2us) {
              sendString("4us, (assumed) but bad");
        } else
        if (a<TIMING_DD_UPPER_4us) {
              sendString("4us");
        } else                                                                                
        if (a<TIMING_DD_UPPER_6us) {                                                    
              sendString("6us");
        } else                                                                                
        if (a<TIMING_DD_UPPER_8us) {                                                    
              sendString("8us");
        } else {
          sendString("10us illegal mfm");
        }
        
        writeByteToUART('\n');
      }
    }

    EICRA = 0;
    TCCR2B=0;
}


// This is a nasty way to unroll the FOR loop.  Would be nice if there was a directive to do this
#define READ_UNROLLED_LOOP(p4uSec, p6uSec, p8uSec)                                                \
            while ((!(PCIFR & bit(PCIF2)))&&(!(TIFR2 & bit(OCF2B)))) {}                           \
            counter = TCNT2, TCNT2 =  0;  /* reset counter.  the "," treats it as volatile  */    \
            while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) {};                                \
            PCIFR |= bit(PCIF2);                                                                  \
            if (TIFR2 & bit(OCF2B)) TIFR2 |= bit(OCF2B);                                          \      
            /* See what MFM 'window' this fits in. Its either a 2uSec, 4uSec or 6uSec,       */   \
            /* or 8+ which isnt technically allowed.   Numbers changed as these are best     */   \
            /* centered around where the bitcels actually are                                */   \
            if (counter<TIMING_DD_UPPER_4us) {                                                    \
              DataOutputByte|=p4uSec;                                                             \
              if (counter >= TIMING_DD_UPPER_2us) counter -= TIMING_DD_UPPER_2us; else counter=0; \
            } else                                                                                \
            if (counter<TIMING_DD_UPPER_6us) {                                                    \
              DataOutputByte|=p6uSec;                                                             \
              counter -= TIMING_DD_UPPER_4us;                                                     \
            } else                                                                                \
            if (counter<TIMING_DD_UPPER_8us) {                                                    \
              DataOutputByte|=p8uSec;                                                             \
              counter -= TIMING_DD_UPPER_6us;                                                     \
            } else {                                                                              \
              /* >=TIMING_DD_UPPER_8us is an error but some disks do this  */                     \
              counter=16;                                                                         \
            }                                                                                     \


// Read the track using a timings to calculate which MFM sequence has been triggered, hwoever, this keeps running until a byte is received from the serial port telling it to stop
void readContinuousStream() {
    // Configure timer 2 just as a counter in NORMAL mode
    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20);       // Precale = 1 (ie: no prescale)
    EICRA = bit(ISC01);       // falling edge of INT0 generates an interrupt, they are turned off, but its an easy way for us to detect a falling edge rather than monitoring a pin
    OCR2A = 0x00;
    OCR2B = 0x00;
    // First wait for the serial port to be available to receive
    while(!(UCSR0A & (1<<UDRE0)));   
   
    // Signal we're active
    digitalWrite(PIN_ACTIVITY_LED,HIGH);
  
    // Index alignment won't work if we're already at the index when this starts.  The index pulse is actually quite long and spans several bytes of data, could be several ms in length
    while (!(PIN_INDEX_PORT & PIN_INDEX_MASK)) {};

    // Skip the first bit.  We're probably already half way through timing it
    while (PIN_READ_DATA_PORT & PIN_READ_DATA_MASK) {
        if (TCNT2>250) break;                             // this is to stop the inteface freezing if theres no disk in the drive
    };
    TCNT2 = 0;  // reset
    while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) {};

    EIFR |=bit(INTF0);  // clear the register saying it detected an index pulse
    TIFR2 |= bit(OCF2B);  // clear the overflow register
    OCR2B = TIMING_DD_UPPER_8us; // This is set to the upper bound.  If we exceed this we must have received a load of non-flux data, this allows us to write '0000' on the PC and loop back round

    // This sets up what would be an interrupt for when the READ PIN is signalled (unfortunatly we can't choose the direction. Its just set when it changes)
    // But this allows us to make sure we dont miss a bit, although the timing might be off slightly.  This is mainly used when disks have very long areas of
    // no flux transitions, typilcally used for copy protection
    PCMSK0 = 0;
    PCMSK1 = 0;
    PCMSK2 = bit(PCINT20);
    PCICR = bit(PCIE2); // Enable the interrupt for this pin

    // First one will just be 01010101 and is ignored by the reader anyway
    register unsigned char lastDataOutput = B01010101;
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
    TCCR2B = 0;      // No Clock (turn off)    
    EICRA = 0;      // disable monitoring   
    PCMSK2 = 0;
    PCICR = 0;
    OCR2A = 0;
    OCR2B = 0;
}

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
        case 'M': if (!driveEnabled) sendString("Drive motor not switched on.\n"); else measureCurrentDisk(); break;
    
        // Command: "?" Means information about the firmware
        case '?':writeByteToUART('1');  // Success
                 writeByteToUART('V');  // Followed
                 writeByteToUART('1');  // By
                 writeByteToUART(advancedControllerMode ? ',' : '.');  // Advanced controller version
                 writeByteToUART('8');  // Number
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
                    if(disktypeHD)
                      writeByteToUART('0');   // not supported
                    else {
                      writeByteToUART('1');
                      readContinuousStream();
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
                     writeByteToUART('1');
                     if(disktypeHD)
                        readTrackDataFast_HD();
                     else
                        readTrackDataFast();
                   }
                   break;
  
        // Command ">" Write track to the drive 
        case '>': if (!driveEnabled) writeByteToUART('0'); else {                     
                     if(disktypeHD) {
                        writeByteToUART('0');                      
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
