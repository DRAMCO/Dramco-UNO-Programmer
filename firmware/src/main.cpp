// Standalone AVR ISP programmer
// August 2011 by Limor Fried / Ladyada / Adafruit
// Jan 2011 by Bill Westfield ("WestfW")
//
// this sketch allows an Arduino to program a flash program
// into any AVR if you can fit the HEX file into program memory
// No computer is necessary. Two LEDs for status notification
// Press button to program a new chip. Piezo beeper for error/success 
// This is ideal for very fast mass-programming of chips!
//
// It is based on AVRISP
//
// using the following pins:
// 10: slave reset
// 11: MOSI
// 12: MISO
// 13: SCK
//  9: 8 MHz clock output - connect this to the XTAL1 pin of the AVR
//     if you want to program a chip that requires a crystal without
//     soldering a crystal in
// ----------------------------------------------------------------------


#include "optiLoader.h"
#include "SPI.h"

#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

/*
 * Pins to target
 */
#define SCK 13
#define MISO 12
#define MOSI 11
#define CLOCK 9     // self-generate 8mhz clock - handy!

#define BUTTON 5
#define PIEZOPIN A3

#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
#define RST_PIN -1

SSD1306AsciiWire oled;


// Global Variables
int pmode=0;
byte pageBuffer[128];		       /* One page of flash */

byte resetPins[] = {3, 2, A2, A3};
byte resetPointer = 0; 
bool errorHappened = false; 

void printDebug(const char* str){
  Serial.println(str);
  oled.println(str);
}

void error(const char *string) {
  printDebug(string); 
  digitalWrite(LED_ERR, HIGH); 
  digitalWrite(LED_ERR, LOW);
  errorHappened = true;
}

void start_pmode () {
  pinMode(13, INPUT); // restore to default

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV128); 
  
  debug("...spi_init done");
  // following delays may not work on all targets...
  pinMode(resetPins[resetPointer], OUTPUT);
  digitalWrite(resetPins[resetPointer], HIGH);
  pinMode(SCK, OUTPUT);
  digitalWrite(SCK, LOW);
  delay(50);
  digitalWrite(resetPins[resetPointer], LOW);
  delay(50);
  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  debug("...spi_transaction");
  spi_transaction(0xAC, 0x53, 0x00, 0x00);
  debug("...Done");
  pmode = 1;
}

void end_pmode () {
  SPI.end();
  digitalWrite(MISO, LOW);		/* Make sure pullups are off too */
  pinMode(MISO, INPUT);
  digitalWrite(MOSI, LOW);
  pinMode(MOSI, INPUT);
  digitalWrite(SCK, LOW);
  pinMode(SCK, INPUT);
  digitalWrite(resetPins[resetPointer], HIGH);
  pmode = 0;
}

/*
 * target_poweron
 * begin programming
 */
boolean target_poweron ()
{
  pinMode(LED_PROGMODE, OUTPUT);
  digitalWrite(LED_PROGMODE, HIGH);
  digitalWrite(resetPins[resetPointer], LOW);  // reset it right away.
  delay(100);
  Serial.print("Starting Program Mode");
  start_pmode();
  Serial.println(" [OK]");
  return true;
}

boolean target_poweroff ()
{
  end_pmode();
  digitalWrite(LED_PROGMODE, LOW);
  return true;
}

void setup () {
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(System5x7);
  oled.setScrollMode(SCROLL_MODE_AUTO);
  
  oled.clear();
  
  Serial.begin(115200);			/* Initialize serial for status msgs */
  printDebug("Dramco UNO Programmer");
  
  pinMode(PIEZOPIN, OUTPUT);

  pinMode(LED_PROGMODE, OUTPUT);
  pulse(LED_PROGMODE,2);
  pinMode(LED_ERR, OUTPUT);
  pulse(LED_ERR, 2);

  pinMode(BUTTON, INPUT);     // button for next programming
  digitalWrite(BUTTON, HIGH); // pullup
  
  pinMode(CLOCK, OUTPUT);
  // setup high freq PWM on pin 9 (timer 1)
  // 50% duty cycle -> 8 MHz
  OCR1A = 0;
  ICR1 = 1;
  // OC1A output, fast PWM
  TCCR1A = _BV(WGM11) | _BV(COM1A1);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // no clock prescale
  
  for(byte i = 0; i<4; i++){
    pinMode(resetPins[i], OUTPUT);
    digitalWrite(resetPins[i], HIGH);
  }

}

void loop (void) {
  if(resetPointer == 0){
    printDebug("Hit BUTTON to start.");
    while (1) {
      if((! digitalRead(BUTTON)) || (Serial.read() == 'G'))
        break;  
    }
  }else{
    printDebug("Trying next.");
  }

  target_poweron();			/* Turn on target power */

  uint16_t signature = 0;
  image_t *targetimage;
        
  if (! (signature = readSignature()))		// Figure out what kind of CPU
    error("Signature fail");
  else
   printDebug("OK");
  
  if (! (targetimage = findImage(signature)))	// look for an image
    error("Image fail");

  if(!errorHappened){
    printDebug("Erasing chip");
    eraseChip();
    printDebug("OK");
    
    if (! programFuses(targetimage->image_progfuses))  // get fuses ready to program
      error("Programming Fuses fail");
    
    if (! verifyFuses(targetimage->image_progfuses, targetimage->fusemask) ) {
      error("Failed to verify fuses");
    } 
    Serial.println("Fuses set & verified");
    end_pmode();
  }
  
  byte *hextext;
  uint16_t pageaddr;
  uint8_t pagesize;
  uint16_t chipsize;
  uint16_t i; 
  if(!errorHappened){
    start_pmode();

    printDebug("Bootloading..."); 
    
    hextext = targetimage->image_hexcode1;  
    pageaddr = 0;
    pagesize = pgm_read_byte(&targetimage->image_pagesize);
    Serial.print("Page size: "); Serial.println(pagesize, DEC);
    chipsize = pgm_read_word(&targetimage->chipsize);
    Serial.print("Chip size: "); Serial.println(chipsize, DEC);
    i = 0;
  
    while (pageaddr < chipsize && hextext) {
       oled.print("#");
       if(i % 20 == 0)
        oled.println();
       Serial.print("Writing address "); Serial.println(pageaddr, HEX);
       byte *hextextpos = readImagePage (hextext, pageaddr, pagesize, pageBuffer);
            
       boolean blankpage = true;
       for (uint8_t i=0; i<pagesize; i++) {
         if (pageBuffer[i] != 0xFF) blankpage = false;
       }          
       if (! blankpage) {
         if (! flashPage(pageBuffer, pageaddr, pagesize))  
           error("Flash programming failed");
       }
       hextext = hextextpos;
       pageaddr += pagesize;
  
       i++;
    }
    oled.println();
  }

  if(!errorHappened){
    printDebug("Programming..."); 
  
    hextext = targetimage->image_hexcode;  
    pageaddr = 0;
    pagesize = pgm_read_byte(&targetimage->image_pagesize);
    Serial.print("Page size: "); Serial.println(pagesize, DEC);
    chipsize = pgm_read_word(&targetimage->chipsize);
    Serial.print("Chip size: "); Serial.println(chipsize, DEC);
    i = 0;
  
    while (pageaddr < chipsize && hextext) {
       oled.print("#");
       if(i % 20 == 0)
        oled.println();
       Serial.print("Writing address "); Serial.println(pageaddr, HEX);
       byte *hextextpos = readImagePage (hextext, pageaddr, pagesize, pageBuffer);
            
       boolean blankpage = true;
       for (uint8_t i=0; i<pagesize; i++) {
         if (pageBuffer[i] != 0xFF) blankpage = false;
       }          
       if (! blankpage) {
         if (! flashPage(pageBuffer, pageaddr, pagesize))  
           error("Flash programming failed");
       }
       hextext = hextextpos;
       pageaddr += pagesize;
  
       i++;
    }
    oled.println();
  }

  if(!errorHappened){
    // Set fuses to 'final' state
    if (! programFuses(targetimage->image_normfuses))
      error("Programming Fuses fail");
  
    delay(100);
    end_pmode();
    delay(100);
    start_pmode();
    delay(100);
    
    printDebug("Verifing flash...");
    if (! verifyImage(targetimage->image_hexcode) ) {
      error("Failed to verify chip");
    } else {
      printDebug("OK");
    }
  
    if (! verifyFuses(targetimage->image_normfuses, targetimage->fusemask) ) {
      error("Failed to verify fuses");
    } else {
      printDebug("OK");
    }
    target_poweroff();      /* turn power off */
  }
  
  Serial.print("RESETPOINTER----------");
  Serial.println(resetPointer);
  Serial.print("PIN----------");
  Serial.println(resetPins[resetPointer]);
  resetPointer++;
  if(resetPointer >= 4){
    resetPointer = 0;
  }
  errorHappened = false;
  
  delay(5000);
}

