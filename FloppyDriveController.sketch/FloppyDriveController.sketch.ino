/* ArduinoFloppyReader (and writer)
*
* Copyright (C) 2017-2020 Robert Smith (@RobSmithDev)
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

/* Latest History:
	  Firmware V1.4: Merged with Pull Request #6 (Modified the behavior of the current track location on Arduino boot - paulofduarte) which also addresses issues with some drives
	  Firmware V1.5: Merged with Pull Request #9 (Detect and read out HD floppy disks 1.44M by kollokollo)
	  Firmware V1.6: Added experimental unbuffered writing HD disk support
      Firmware V1.7: Added suggestion from GitHub user "prickle" regarding the CHECK_SERIAL function which should reoslve issues with some of the USB to SERIAL converters
*/	  

/////////////////////////////////////////////////////////////////////////////////////////////////////
// This sketch manages the interface between the floppy drive and the computer as well as the     //
// low-level disk reading and writing.  For more information and how to connect your Arduino     //
// to a floppy drive and computer visit https://amiga.robsmithdev.co.uk                         //
/////////////////////////////////////////////////////////////////////////////////////////////////
// This code doesnt actually do any decoding, and is mearly reading pulses, so can be used to //
// Read data from other disk formats too.                                                    //
//////////////////////////////////////////////////////////////////////////////////////////////

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

// PIN 13 - Activity LED
#define PIN_ACTIVITY_LED         13       // Standard LED on Arduinos.  We're just using this as a read/write status flag




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

// If we're in WRITING mode or not
bool inWriteMode = 0;

/* Where there should be a HD Disk been read (1) or a DD and SD Disk (0).*/
bool disktypeHD = 0;




// Because we turned off interrupts delay() doesnt work!
void smalldelay(unsigned long delayTime) {
    delayTime*=(F_CPU/(9*1000L));
  
    for (unsigned long loops=0; loops<delayTime; ++loops) {
        asm volatile("nop\n\t"::);
    }
}


// Step the head once.  This seems to be an acceptable speed for the head
void stepDirectionHead() {
    smalldelay(5);
    digitalWrite(PIN_MOTOR_STEP,LOW);
    smalldelay(5);
    digitalWrite(PIN_MOTOR_STEP,HIGH);
}
// Step the head once.  Do it a bit faster...
void stepDirectionHead_fast() {
    smalldelay(4);
    digitalWrite(PIN_MOTOR_STEP,LOW);
    smalldelay(3);
    digitalWrite(PIN_MOTOR_STEP,HIGH);
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
    while (!( UCSR0A & ( 1 << RXC0 ))){};    // Wait for data to be available
    return UDR0;                                 // Read it
}

// Directly write a byte to the UART0
inline void writeByteToUART(const char value) {
    while(!(UCSR0A & (1<<UDRE0)));                // Wait until the last byte has been sent
    UDR0 = value;                                 // And send another
}

// Main arduino setup 
void setup() {
    // Do these right away to prevent the disk being written to
    digitalWrite(PIN_WRITE_GATE, HIGH);
    digitalWrite(PIN_WRITE_DATA, HIGH);
    pinMode(PIN_WRITE_GATE,OUTPUT);
    pinMode(PIN_WRITE_DATA,OUTPUT);
    pinMode(PIN_CTS,OUTPUT);

    pinMode(PIN_WRITE_PROTECTED, INPUT_PULLUP);
    pinMode(PIN_DETECT_TRACK_0, INPUT_PULLUP);
    pinMode(PIN_READ_DATA,INPUT_PULLUP);

    pinMode(PIN_INDEX_DETECTED,INPUT_PULLUP);

    // Prepre the pin inputs and outputs
    pinMode(PIN_DRIVE_ENABLE_MOTOR, OUTPUT);
    pinMode(PIN_HEAD_SELECT, OUTPUT);
    digitalWrite(PIN_DRIVE_ENABLE_MOTOR,HIGH);
    digitalWrite(PIN_HEAD_SELECT,LOW);    
    
    pinMode(PIN_MOTOR_DIR, OUTPUT);
    pinMode(PIN_MOTOR_STEP,OUTPUT);
    pinMode(PIN_ACTIVITY_LED,OUTPUT);
    
    digitalWrite(PIN_MOTOR_STEP, HIGH);
    digitalWrite(PIN_ACTIVITY_LED,LOW);
   
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
              
      default:
        writeByteToUART('0');
        break;
    }
}

// Rewinds the head back to Track 0
bool goToTrack0() {
    digitalWrite(PIN_MOTOR_DIR,MOTOR_TRACK_DECREASE);   // Set the direction to go backwards
    int counter=0;
    while (digitalRead(PIN_DETECT_TRACK_0) != LOW) {
       stepDirectionHead_fast();   // Keep moving the head until we see the TRACK 0 detection pin
       counter++;
       // If this happens we;ve steps twice as many as needed and still havent found track 0
       if (counter>170) {
          return false;
       }
    }
    
    currentTrack = 0;    // Reset the track number
    return true;
}

// Goto to a specific track.  During testing it was easier for the track number to be supplied as two ASCII characters, so I left it like this
bool gotoTrackX() {
    // Read the bytes
    byte track1 = readByteFromUART();
    byte track2 = readByteFromUART();

    // Validate
    if ((track1<'0') || (track1>'9')) return false;
    if ((track2<'0') || (track2>'9')) return false;

    // Calculate target track and validate 
    int track = ((track1-'0')*10) + (track2-'0');
    if (track<0) return false;
    if (track>81) return false; // yes amiga could read track 81!

    // Exit if its already been reached
    if (track == currentTrack) return true;

    // If current track is unknown go to track 0 first
    if (currentTrack == -1) goToTrack0();

    // And step the head until we reach this track number
    if (currentTrack < track) {
        digitalWrite(PIN_MOTOR_DIR,MOTOR_TRACK_INCREASE);   // Move OUT
        while (currentTrack < track) {
            stepDirectionHead();
            currentTrack++;         
        }
    } else {
        digitalWrite(PIN_MOTOR_DIR,MOTOR_TRACK_DECREASE);   // Move IN
        while (currentTrack > track) {
            stepDirectionHead();
            currentTrack--;
        }
    }

    return true;
}


// 256 byte circular buffer - don't change this, we abuse the unsigned char to overflow back to zero!
#define SERIAL_BUFFER_SIZE 256
#define SERIAL_BUFFER_START (SERIAL_BUFFER_SIZE-16)
unsigned char SERIAL_BUFFER[SERIAL_BUFFER_SIZE];


#define CHECK_SERIAL()          if (UCSR0A & ( 1 << RXC0 )) {                      \
                                    SERIAL_BUFFER[serialWritePos++] = UDR0;        \
                                    serialBytesInUse++;                            \
                                }                                                  \
                                if (serialBytesInUse<SERIAL_BUFFER_START)          \
                                    PIN_CTS_PORT &= (~PIN_CTS_MASK);               \
                                else PIN_CTS_PORT|=PIN_CTS_MASK;                   
                                
                                            


// Small Macro to write a '1' pulse to the drive if a bit is set based on the supplied bitmask
#define WRITE_BIT(value,bitmask) if (currentByte & bitmask)  {                           \
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
    
    // Check if its write protected.  You can only do this after the write gate has been pulled low
    if (digitalRead(PIN_WRITE_PROTECTED) == LOW) {
        writeByteToUART('N'); 
        digitalWrite(PIN_WRITE_GATE,HIGH);
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
    unsigned int serialBytesInUse = SERIAL_BUFFER_START;
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
        while (TCNT2>=240) {}
		
        // Now we write the data.  Hopefully by the time we get back to the top everything is ready again
        WRITE_BIT(0x10,B10000000);
        CHECK_SERIAL();
        WRITE_BIT(0x30,B01000000);
        CHECK_SERIAL();
        WRITE_BIT(0x50,B00100000);
        CHECK_SERIAL();
        WRITE_BIT(0x70,B00010000);
        CHECK_SERIAL();
        WRITE_BIT(0x90,B00001000);
        CHECK_SERIAL();
        WRITE_BIT(0xB0,B00000100);
        CHECK_SERIAL();
        WRITE_BIT(0xD0,B00000010);
        CHECK_SERIAL();
        WRITE_BIT(0xF0,B00000001);
    }  
	
	// Turn off the write head
    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;

    // Done!
    writeByteToUART('1');
    digitalWrite(PIN_ACTIVITY_LED,LOW);
    
    // Disable the 500khz signal
    TCCR2B = 0;   // No Clock (turn off)    
}


// Write a track to disk from the UART - the data should be pre-MFM encoded raw track data where '1's are the pulses/phase reversals to trigger
// THIS CODE IS UNTESTED
void writeTrackFromUART_HD() {
    // Configure timer 2 just as a counter in NORMAL mode
    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20);       // Precale = 1  
    
    // Check if its write protected.  You can only do this after the write gate has been pulled low
    if (digitalRead(PIN_WRITE_PROTECTED) == LOW) {
        writeByteToUART('N'); 
        digitalWrite(PIN_WRITE_GATE,HIGH);
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
    unsigned int serialBytesInUse = SERIAL_BUFFER_START;
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
    }  
	
	  // Turn off the write head
    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;

    // Done!
    writeByteToUART('1');
    digitalWrite(PIN_ACTIVITY_LED,LOW);
    
    // Disable the 500khz signal
    TCCR2B = 0;   // No Clock (turn off)    
}





// Write blank data to a disk so that no MFM track could be detected
void eraseTrack() {
    // Configure timer 2 just as a counter in NORMAL mode
    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20);       // Precale = 1  
    
    // Check if its write protected.  You can only do this after the write gate has been pulled low
    if (digitalRead(PIN_WRITE_PROTECTED) == LOW) {
        writeByteToUART('N'); 
        digitalWrite(PIN_WRITE_GATE,HIGH);
        return;
    } else writeByteToUART('Y');
    
    register unsigned char currentByte;

    digitalWrite(PIN_ACTIVITY_LED,HIGH);
   
    // Enable writing
    PIN_WRITE_GATE_PORT&=~PIN_WRITE_GATE_MASK;

    // Reset the counter, ready for writing
    TCNT2=0;  
    currentByte = 0xAA;

    // Write complete blank track - at 300rpm, 500kbps, a track takes approx 1/5 second to write.  This is roughly 12500 bytes.  Our RAW read is 13888 bytes, so we'll use that just to make sure we get every last bit.
    for (register unsigned int counter=0; counter<RAW_TRACKDATA_LENGTH; counter++) {
        WRITE_BIT(0x10,B10000000);
        WRITE_BIT(0x30,B01000000);
        WRITE_BIT(0x50,B00100000);
        WRITE_BIT(0x70,B00010000);
        WRITE_BIT(0x90,B00001000);
        WRITE_BIT(0xB0,B00000100);
        WRITE_BIT(0xD0,B00000010);
        WRITE_BIT(0xF0,B00000001);
        while (TCNT2>=240) {}
    }  
  
    // Turn off the write head
    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;

    // Done!
    writeByteToUART('1');
    digitalWrite(PIN_ACTIVITY_LED,LOW);
    
    // Disable the 500khz signal
    TCCR2B = 0;   // No Clock (turn off)    
}



// Write blank data to a disk so that no MFM track could be detected
// THIS IS UNTESTED
void eraseTrack_HD() {
    // Configure timer 2 just as a counter in NORMAL mode
    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20);       // Precale = 1  
    
    // Check if its write protected.  You can only do this after the write gate has been pulled low
    if (digitalRead(PIN_WRITE_PROTECTED) == LOW) {
        writeByteToUART('N'); 
        digitalWrite(PIN_WRITE_GATE,HIGH);
        return;
    } else writeByteToUART('Y');
    
    register unsigned char currentByte;

    digitalWrite(PIN_ACTIVITY_LED,HIGH);
   
    // Enable writing
    PIN_WRITE_GATE_PORT&=~PIN_WRITE_GATE_MASK;

    // Reset the counter, ready for writing
    TCNT2=0;  
    currentByte = 0xAA;

    // Write complete blank track - at 300rpm, 500kbps, a track takes approx 1/5 second to write.  This is roughly 12500 bytes.  Our RAW read is 13888 bytes, so we'll use that just to make sure we get every last bit.
    for (register unsigned int counter=0; counter<RAW_TRACKDATA_LENGTH; counter++) {
        WRITE_BIT(0x8,B10000000);
        WRITE_BIT(0x18,B01000000);
        WRITE_BIT(0x28,B00100000);
        WRITE_BIT(0x38,B00010000);
        WRITE_BIT(0x48,B00001000);
        WRITE_BIT(0x58,B00000100);
        WRITE_BIT(0x68,B00000010);
        WRITE_BIT(0x78,B00000001);
        WRITE_BIT(0x89,B10000000);
        WRITE_BIT(0x98,B01000000);
        WRITE_BIT(0xA8,B00100000);
        WRITE_BIT(0xB8,B00010000);
        WRITE_BIT(0xC8,B00001000);
        WRITE_BIT(0xD8,B00000100);
        WRITE_BIT(0xE8,B00000010);
        WRITE_BIT(0xF8,B00000001);
        while (TCNT2>=248) {}
    }  
  
    // Turn off the write head
    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;

    // Done!
    writeByteToUART('1');
    digitalWrite(PIN_ACTIVITY_LED,LOW);
    
    // Disable the 500khz signal
    TCCR2B = 0;   // No Clock (turn off)    
}


// Read the track using a timings to calculate which MFM sequence has been triggered
void readTrackDataFast() {
    // Configure timer 2 just as a counter in NORMAL mode
    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20);       // Precale = 1  
    
    // First wait for the serial port to be available
    while(!(UCSR0A & (1<<UDRE0)));   
   
    // Signal we're active
    digitalWrite(PIN_ACTIVITY_LED,HIGH);

    // Force data to be stored in a register
    register unsigned char DataOutputByte = 0;

    // While the INDEX pin is high wait if the other end requires us to
    if (readByteFromUART())
        while (PIN_INDEX_PORT & PIN_INDEX_MASK)  {};

    // Prepare the two counter values as follows:
    TCNT2=0;       // Reset the counter

    register unsigned char counter;
    long totalBits=0;
    long target = ((long)RAW_TRACKDATA_LENGTH)*(long)8;

    while (totalBits<target) {
        for (register unsigned char bits=0; bits<4; bits++) {
            // Wait while pin is high
            
            while (PIN_READ_DATA_PORT & PIN_READ_DATA_MASK) {};
            counter = TCNT2, TCNT2 = 0;  // reset - must be done with a COMMA
           
            DataOutputByte<<=2;   

            // DO NOT USE BRACES HERE, use the "," or the optomiser messes it up
            if (counter<80) DataOutputByte|=B00000001,totalBits+=2; else    // this accounts for just a '1' or a '01' as two '1' arent allowed in a row
            if (counter>111) DataOutputByte|=B00000011,totalBits+=4; else DataOutputByte|=B00000010,totalBits+=3;
            
            // Wait until pin is high again
            while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) {};
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
    
    // First wait for the serial port to be available
    while(!(UCSR0A & (1<<UDRE0)));   
   
    // Signal we're active
    digitalWrite(PIN_ACTIVITY_LED,HIGH);

    // Force data to be stored in a register
    register unsigned char DataOutputByte = 0;

    // While the INDEX pin is high wait if the other end requires us to
    if (readByteFromUART())
        while (PIN_INDEX_PORT & PIN_INDEX_MASK)  {};

    // Prepare the two counter values as follows:
    TCNT2=0;       // Reset the counter

    register unsigned char counter;
    long totalBits=0;
    long target = ((long)RAW_HD_TRACKDATA_LENGTH)*(long)8;

    while (totalBits<target) {
        for (register unsigned char bits=0; bits<4; bits++) {
            // Wait while pin is high
            
            while (PIN_READ_DATA_PORT & PIN_READ_DATA_MASK) {};
            counter = TCNT2, TCNT2 = 0;  // reset - must be done with a COMMA
           
            DataOutputByte<<=2;   

            // DO NOT USE BRACES HERE, use the "," or the optomiser messes it up
            if (counter<40) DataOutputByte|=B00000001,totalBits+=2; else    // this accounts for just a '1' or a '01' as two '1' arent allowed in a row
            if (counter>55) DataOutputByte|=B00000011,totalBits+=4; else DataOutputByte|=B00000010,totalBits+=3;
            
            // Wait until pin is high again
            while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) {};
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

static char *i2a(unsigned int i, char *a, unsigned r) {
  if(i/r>0) a=i2a(i/r,a,r);
  *a = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz@$!?"[i % r];
  return a+1;
}

/* Make a statistics of the data seen on a track to determine the density of the media */
void measureTrackData() {
   /* for the statistics */
    int t1=0;
    int t2=0;
    int t3=0;
    int t4=0;
    int t5=0;
    int t6=0;
    // Configure timer 2 just as a counter in NORMAL mode
    TCCR2A = 0 ;              // No physical output port pins and normal operation
    TCCR2B = bit(CS20);       // Precale = 1  
       
    // Signal we're active
    digitalWrite(PIN_ACTIVITY_LED,HIGH);

    // Force data to be stored in a register
    register unsigned char DataOutputByte = 0;

    // While the INDEX pin is high wait 
    while (PIN_INDEX_PORT & PIN_INDEX_MASK)  {};

    // Prepare the two counter values as follows:
    TCNT2=0;       // Reset the counter

    register unsigned char counter;
    long totalBits=0;
    long target = ((long)RAW_TRACKDATA_LENGTH/4)*(long)8;

    while (totalBits<target) {
        for (register unsigned char bits=0; bits<4; bits++) {
            // Wait while pin is high
            
            while (PIN_READ_DATA_PORT & PIN_READ_DATA_MASK) {};
            counter = TCNT2, TCNT2 = 0;  // reset - must be done with a COMMA
           
            DataOutputByte<<=2;   

            // DO NOT USE BRACES HERE, use the "," or the optomiser messes it up	    
            if      (counter<40) t1++;    
            else if (counter<55) t2++;
			else t3++;
	       
            if (counter<80) t4++; 
            else if (counter<112) t5++; 
            else t6++; 
	    
			totalBits+=2;
            
            // Wait until pin is high again
            while (!(PIN_READ_DATA_PORT & PIN_READ_DATA_MASK)) {};
        }
    }

    // turn off the status LED
    digitalWrite(PIN_ACTIVITY_LED,LOW);

    // Disable the counter
    TCCR2B = 0;      // No Clock (turn off)
    
    /* Now output the result: */
    char a[8];
    byte i;
    i=0;
    writeByteToUART(10);
    *i2a(t1,a,10)=0;
    while(a[i]) writeByteToUART(a[i++]);
    writeByteToUART('-');
    *i2a(t2,a,10)=0;
    i=0;
    while(a[i]) writeByteToUART(a[i++]);
    writeByteToUART('-');
    *i2a(t3,a,10)=0;
    i=0;
    while(a[i]) writeByteToUART(a[i++]);
    writeByteToUART(10);
    *i2a(t4,a,10)=0;
    i=0;
    while(a[i]) writeByteToUART(a[i++]);
    writeByteToUART('-');
    *i2a(t5,a,10)=0;
    i=0;
    while(a[i]) writeByteToUART(a[i++]);
    writeByteToUART('-');
    *i2a(t6,a,10)=0;
    i=0;
    while(a[i]) writeByteToUART(a[i++]);
    writeByteToUART(10);
    if(t5<100 && t6<100) {
      writeByteToUART('H');
      disktypeHD=1;
    } else {
      writeByteToUART('D');
      disktypeHD=0;
    }
    writeByteToUART('D');
    writeByteToUART(10);
}


// The main command loop
void loop() {
    PIN_CTS_PORT &= (~PIN_CTS_MASK);            // Allow data incoming
    PIN_WRITE_GATE_PORT|=PIN_WRITE_GATE_MASK;   // always turn writing off
  
    // Read the command from the PC
    byte command = readByteFromUART();

    switch (command) {
  
        // Command: "?" Means information about the firmware
        case '?':writeByteToUART('1');  // Success
                 writeByteToUART('V');  // Followed
                 writeByteToUART('1');  // By
                 writeByteToUART('.');  // Version
                 writeByteToUART('7');  // Number
                 break;
  
        // Command "." means go back to track 0
        case '.': if (!driveEnabled) writeByteToUART('0'); else {
                   if (goToTrack0())    // reset 
                      writeByteToUART('1');
                   else writeByteToUART('#');
                 }  
                 break;
  
        // Command "#" means goto track.  Should be formatted as #00 or #32 etc
        case '#': // Goto Track
                  if (!driveEnabled) {
                      readByteFromUART();
                      readByteFromUART();
                      writeByteToUART('0'); 
                  } else
                  if (gotoTrackX()) {
                      smalldelay(100); // wait for drive
                      writeByteToUART('1');
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
        case '>': if (!driveEnabled) writeByteToUART('0'); else
                  if (!inWriteMode) writeByteToUART('0'); else {
                     writeByteToUART('1');
                     if(disktypeHD) 
						            writeTrackFromUART_HD(); 
					          else 
						            writeTrackFromUART();
                   }
                   break;

        // Command "X" Erase current track (writes 0xAA to it)
        case 'X': if (!driveEnabled) writeByteToUART('0'); else
                  if (!inWriteMode) writeByteToUART('0'); else {
                     writeByteToUART('1');
                     if (disktypeHD) 
          						 eraseTrack_HD();
          					 else
          						 eraseTrack();
                  }
                  break;

        // Command "H" Set HD disk type
        case 'H': 
                  disktypeHD = 1;
                  writeByteToUART('1');                  
                  break;
				  
        // Command "D" Set DD or SD disk type
        case 'D': 
                  disktypeHD = 0;
                  writeByteToUART('1');                  
                  break;
				  
        // Command "M" measure data timings 
        case 'M': if(!driveEnabled) writeByteToUART('0'); 
	          else {
                     writeByteToUART('1');
                     measureTrackData();
                  }
                  break;
  
        // Turn off the drive motor
        case '-': digitalWrite(PIN_DRIVE_ENABLE_MOTOR,HIGH);
                  digitalWrite(PIN_WRITE_GATE,HIGH);
                  driveEnabled = 0;
                  writeByteToUART('1');
                  inWriteMode = 0;
                  break;
  
       // Turn on the drive motor and setup in READ MODE
       case '+':  if (inWriteMode) {
                     // Ensure writing is turned off
                     digitalWrite(PIN_DRIVE_ENABLE_MOTOR,HIGH);
                     digitalWrite(PIN_WRITE_GATE,HIGH);
                     smalldelay(100);
                     driveEnabled = 0;
                     inWriteMode = 0;
                  }
       
                  if (!driveEnabled) {
                     digitalWrite(PIN_DRIVE_ENABLE_MOTOR,LOW);
                     driveEnabled = 1;
                     smalldelay(750); // wait for drive
                  }
                  writeByteToUART('1');
                  break;
  
       // Turn on the drive motor and setup in WRITE MODE
       case '~':  if (driveEnabled) {
                      digitalWrite(PIN_WRITE_GATE,HIGH);
                      digitalWrite(PIN_DRIVE_ENABLE_MOTOR,HIGH);
                      driveEnabled = 0;
                      smalldelay(100);
                  }
                  // We're writing!
                  digitalWrite(PIN_WRITE_GATE,LOW);
                  // Gate has to be pulled LOW BEFORE we turn the drive on
                  digitalWrite(PIN_DRIVE_ENABLE_MOTOR,LOW);
                  // Raise the write gate again                    
                  digitalWrite(PIN_WRITE_GATE,HIGH);                
                  smalldelay(750); // wait for drive
                  
                  // At this point we can see the status of the write protect flag
                  if (digitalRead(PIN_WRITE_PROTECTED) == LOW) {
                      writeByteToUART('0');
                      inWriteMode = 0;
                      digitalWrite(PIN_DRIVE_ENABLE_MOTOR,HIGH);
                      //digitalWrite(PIN_WRITE_GATE,HIGH);
                  } else  {
                     inWriteMode = 1;
                     driveEnabled = 1;           
                     writeByteToUART('1');
                  }                
                  break;

           case '&': runDiagnostic();
                     break;
  
  
      // We don't recognise the command!
      default:
                 writeByteToUART('!'); // error
                 break;
     }
}
