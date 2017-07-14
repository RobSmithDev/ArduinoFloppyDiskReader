/* ArduinoFloppyReader
*
* Copyright (C) 2017 Robert Smith (@RobSmithDev)
* http://amiga.robsmithdev.co.uk
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


////////////////////////////////////////////////////////////////////////////////////////
// This sketch manages the interface between the floppy drive and the computer        //
// as well as the low-level disk reading.  For more information and how to connect    //
// your Arduino to a floppy drive and computer visit http://amiga.robsmithdev.co.uk   //
////////////////////////////////////////////////////////////////////////////////////////

//#define F_CPU 16000000UL               // Current clock frequency - this is already defined by the Arduino code
#define BAUDRATE 1000000                 // The baudrate that we want to communicate over (1M)
#define BAUD_PRESCALLER_NORMAL_MODE      (((F_CPU / (BAUDRATE * 16UL))) - 1)
#define BAUD_PRESCALLER_DOUBLESPEED_MODE (((F_CPU / (BAUDRATE * 8UL))) - 1)
#define UART_USE_DOUBLESPEED_MODE        // We're using double speed mode

#define MOTOR_TRACK_DECREASE   HIGH      // Motor directions for PIN settings
#define MOTOR_TRACK_INCREASE   LOW

#define PIN_DRIVE_ENABLE_MOTOR   5        // Turn on and off the motor on the drive
#define PIN_DETECT_TRACK_0       8        // Used to see if the drive is at track 0

#define PIN_MOTOR_DIR            6        // Stepper motor output to choose the direction the head should move
#define PIN_MOTOR_STEP           7        // Stepper motor step line to move the head position
#define PIN_HEAD_SELECT          9        // Choose upper and lower head on the drive
#define PIN_ACTIVITY_LED         13       // Standard LED on Arduinos.  We're just using this as a read status flag

#define PIN_INT0_PIN             2         // The Pin that is used for INT0 input.  This is physically connected to the PIN_RAW_FLOPPYDTA pin 
#define PIN_RAW_FLOPPYDATA       4         // The pin the floppy drive data is connected to.  This is the external Timer0/Counter0 trigger pin


// Paula on the Amiga used to find the SYNC WORDS and then read 0x1900 further WORDS.  We're not doing that (the PC is) so we need to read more data than this to compensate for this (so we read 1.25 this amount)
#define RAW_TRACKDATA_LENGTH   (0x1900*2+0x800)      // Number of bytes to read.  0x1900x2 (standard read) + 0x800 - hald the read again

// The current track that the head is over
int currentTrack = 0;

// If the drive has been switched on or not
bool driveEnabled  = 0;



// We're using Timer0 as a counter.  This is normally used by the Arduino delay() function. This is a poor rough approxamation, but enough for what we need
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
    while (( UCSR0A & ( 1 << RXC0 )) == 0){};    // Wait for data to be available
    return UDR0;                                 // Read it
}

// Directly write a byte to the UART0
inline void writeByteToUART(const char value) {
    while(!(UCSR0A & (1<<UDRE0)));                // Wait until the last byte has been sent
    UDR0 = value;                                 // And send another
}


// Main arduino setup 
void setup() {
    // Prepre the pin inputs and outputs
    pinMode(PIN_DRIVE_ENABLE_MOTOR, OUTPUT);
    digitalWrite(PIN_DRIVE_ENABLE_MOTOR,HIGH);
    pinMode(PIN_HEAD_SELECT, OUTPUT);
    digitalWrite(PIN_HEAD_SELECT,LOW);    
    
    pinMode(PIN_MOTOR_DIR, OUTPUT);
    pinMode(PIN_MOTOR_STEP,OUTPUT);
    pinMode(PIN_DETECT_TRACK_0, INPUT_PULLUP);
    pinMode(PIN_RAW_FLOPPYDATA,INPUT_PULLUP);
    pinMode(PIN_INT0_PIN,INPUT_PULLUP);
    pinMode(PIN_ACTIVITY_LED,OUTPUT);
    digitalWrite(PIN_ACTIVITY_LED,LOW);
   
    // Disable all interrupts and mask them all off
    cli();
    TIMSK0=0;
    TIMSK1=0;
    TIMSK2=0;

    // Configure Timer 0 as a counter, triggered as a result a falling edge on port pin 4, as follows:
    // See https://sites.google.com/site/qeewiki/books/avr-guide/counter-atmega328 for more information
    TCCR0A = B00000000;    
    TCCR0B = (1 << CS02) | (1 << CS01) | (0 << CS00);  // Turn on counter 0, T0, on Clock FALLING edge
    // TCCR0B = (1 << CS02) | (1 << CS01) | (1 << CS00);  // Turn on counter 0, T0, on Clock RISING edge
    TIFR0 = 0;

    // We also want an interrupt generated in response to this.  INT0 is the best for this.  Pin 4 and Pin 2 are wired together
    EICRA = (1 <<ISC01 ) | (0 << ISC00 );    // FALLING EDGE triggers the interrupt on Pin 2
    // EICRA = (1 <<ISC01 ) | (1 << ISC00 );    // RISING Edge
    EIMSK = (1 << INT0);  // Enable an interrupt to occur when this happens    

    // Make sure no other interrupts are enabled or allowed to execute.  The timing is so important we dont want anything else messing it up
    PCICR = 0;
    PCIFR = 0;
    PCMSK0 = 0;
    PCMSK2 = 0;
    PCMSK1 = 0;

    // Setup the USART
    prepSerialInterface();
}


// Rewinds the head back to Track 0
void goToTrack0() {
    digitalWrite(PIN_MOTOR_DIR,MOTOR_TRACK_DECREASE);   // Set the direction to go backwards
    while (digitalRead(PIN_DETECT_TRACK_0) != LOW) stepDirectionHead();   // Keep moving the head until we see the TRACK 0 detection pin
    
    currentTrack = 0;    // Reset the track number
}

// Goto a specific track.  During testing it was easier for the track number to be supplied as two ASCII characters, so I left it like this
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


// We declare a variable called targetPhase and make it available to the ASM code
volatile byte targetPhase asm ("targetPhase");


// Trigger ISR from T0 pin - we literally just use this to re-sync the bit-phase
// We could have done this with the TIMER0_OVF_vect, but INT0_vect is the highest priority and TIMER0_OVF_vect is already used by the Arduino library
// As we're in "NAKED" mode the compiler doesnt do anything to preserve state so we have to do *everything* which means we can write a much smaller ISR
ISR (INT0_vect, ISR_NAKED) {
  // Three instructions probably means the phase is 3 points out of alignment
  asm volatile("push __tmp_reg__");                                         // Preserve the tmp_register
  asm volatile("lds __tmp_reg__, targetPhase");                             // Copy the phase value into the tmp_register
  asm volatile("sts %0, __tmp_reg__" : : "M" (_SFR_MEM_ADDR(TCNT2)));       // Copy the tmp_register into the memory location where TCNT2 is (timer 2 count position - 500khz phase)
  asm volatile("pop __tmp_reg__");                                          // Restore the tmp_register
  asm volatile("reti");                                                     // And exit the ISR

  // The above equates to this without NAKED mode
  // TCNT2=targetPhase;
}


// Read the track using a very basic Phase-Locked Loop implementation (we just re-sync our timer when we see a pulse)
void readTrackDataPLL() {
    // Read the sync-phase position as HEX from 0123456789ABCDEF.  Theriticially around 7/8 should be the best 
    byte phase = readByteFromUART();
    if ((phase>='0') && (phase<='9')) phase-='0'; else
    if ((phase>='A') && (phase<='F')) phase=phase-'A'+10; else phase=7; // on error default to 7
    targetPhase = phase;
    
    // Configure timer 2 as PWM, Phase-correct, counter/timer.  But we're not going to actually output this on any pins.  
    // The counter counts until ot gets to the value in OCR2A.  This would be for 300rpm floppy.
    // This is calculated as:  25 uSec period (500khz)  - OCRA = (PERIOD / (TICKS/PRESCALER)) / 2  = (2uS / 62.5ns) / 2
    TCCR2A = bit(WGM20) ;       // No physical output port pins
    TCCR2B = bit(WGM22) | 1;   // Precale = 1  
    OCR2A = 16;                // Number of Counts 16 - which gives 500khz.  

    // Uncomment the following to see the 500Khz(ish) square wave on pin 11
    //TCCR2A|= bit (COM2A0);
    //                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    pinMode(11, OUTPUT); 
    
    // First wait for the serial port to be available
    while(!(UCSR0A & (1<<UDRE0)));   
   
    // Signal we're active
    digitalWrite(PIN_ACTIVITY_LED,HIGH);

    // Prepare the two counter values as follows:
    TCNT2=0;       // Reset the 500khz signal to trigger at maximum interval from now
    TCNT0=0;       // reset pulse detection counter

    // Force data to be stored in a register
    register unsigned char DataOutputByte = 0;

    // Start interrupts.  There should only be one that actually triggers
    sei();

    // loop for the pre-arranged amount of bytes
    for (register unsigned int a=0; a<RAW_TRACKDATA_LENGTH; a++) {
        // Loop for all 8 bits
        for (register unsigned char b=0; b<8; b++) {                                             
            DataOutputByte<<=1;                     // Make space for the next bit
            
            // Wait for the 500khz overflow - this gets messed with in the ISR
            while (!(TIFR2&_BV(TOV2))) {};
            
            TIFR2|=_BV(TOV2);                       // And reset the 500khz overflow flag
            
            if (TCNT0) {                            // Did we detect any external pulses? 
              DataOutputByte|=1;                    // Yes.  Add a bit to relate to this
              TCNT0=0;                              // Reset our pulse counter 
            }
        }
        
        // When 8 bits have been received send them over the serial port
        UDR0 = DataOutputByte;
    }    

    // Turn off interrupts 
    cli();

    // turn off the status LED
    digitalWrite(PIN_ACTIVITY_LED,LOW);

    // Disable the 500khz signal
    TCCR2A = bit(WGM20);       // No physical output port pins
    TCCR2B = bit(WGM22) | 0;   // No Clock (turn off)    
}



// The main command loop
void loop() {
  // Read the command from the PC
  byte command = readByteFromUART();

  switch (command) {

      // Command: "?" Means information about the firmware
      case '?':writeByteToUART('1');  // Success
               writeByteToUART('V');  // Followed
               writeByteToUART('1');  // By
               writeByteToUART('.');  // Version
               writeByteToUART('0');  // Number
               break;

      // Command "." means go back to track 0
      case '.': if (!driveEnabled) writeByteToUART('0'); else {
                 goToTrack0();    // reset 
                 writeByteToUART('1');
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
      case '<': if (!driveEnabled) writeByteToUART('0'); else {
                   writeByteToUART('1');
                   readTrackDataPLL();
                }
                break;

      // Command ">" Write track to the drive
      case '>': writeByteToUART('0');  // Not implemented yet
                break;

      // Turn off the drive motor
      case '-': digitalWrite(PIN_DRIVE_ENABLE_MOTOR,HIGH);
                driveEnabled = 0;
                writeByteToUART('1');
                break;

     // Turn on the drive motor
     case '+': digitalWrite(PIN_DRIVE_ENABLE_MOTOR,LOW);
                driveEnabled = 1;
                smalldelay(750); // wait for drive
                writeByteToUART('1');
                break;

    // We don't recognise the command!
    default:
               writeByteToUART('!'); // error
               break;
   }
}
