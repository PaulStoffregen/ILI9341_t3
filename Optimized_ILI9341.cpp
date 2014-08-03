// https://github.com/PaulStoffregen/Optimized_ILI9341
// http://forum.pjrc.com/threads/26305-Highly-optimized-ILI9341-(320x240-TFT-color-display)-library

/***************************************************
  This is our library for the Adafruit ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "Optimized_ILI9341.h"
#include <SPI.h>

// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
Optimized_ILI9341::Optimized_ILI9341(uint8_t cs, uint8_t dc, uint8_t rst) : Adafruit_GFX(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT) {
  _cs   = cs;
  _dc   = dc;
  _rst  = rst;
}

    
inline void Optimized_ILI9341::spiwrite(uint8_t c)
{
	SPI.transfer(c);
}

void Optimized_ILI9341::writebegin(void) {
}

void Optimized_ILI9341::writecommand(uint8_t c)
{
	// Serial.print("SPI CMD: ");
	// Serial.print(c, HEX);
	SPI0.PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0);
	while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
        //Serial.println(" completed");
}

void Optimized_ILI9341::writedata(uint8_t c)
{
        //Serial.print("SPI data: ");
        //Serial.print(c, HEX);
	SPI0.PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
	while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
        //Serial.println(" completed");
}

void Optimized_ILI9341::writedata16(uint16_t d)
{
	SPI0.PUSHR = d | (pcs_data << 16) | SPI_PUSHR_CTAS(1);
	while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
}


void Optimized_ILI9341::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
 uint16_t y1) {

  writecommand(ILI9341_CASET); // Column addr set
  writedata16(x0);   // XSTART 
  writedata16(x1);   // XEND
  writecommand(ILI9341_PASET); // Row addr set
  writedata16(y0);   // YSTART
  writedata16(y1);   // YEND
  writecommand(ILI9341_RAMWR); // write to RAM
}


void Optimized_ILI9341::pushColor(uint16_t color) {
  writebegin();
  writedata16(color);
}

void Optimized_ILI9341::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  setAddrWindow(x,y,x+1,y+1);
  writedata16(color);
}


void Optimized_ILI9341::drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((y+h-1) >= _height) h = _height-y;
  setAddrWindow(x, y, x, y+h-1);
  while (h--) {
    writedata16(color);
  }
}


void Optimized_ILI9341::drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  setAddrWindow(x, y, x+w-1, y);
  while (w--) {
    writedata16(color);
  }
}

void Optimized_ILI9341::fillScreen(uint16_t color) {
  fillRect(0, 0,  _width, _height, color);
}

// fill a rectangle
void Optimized_ILI9341::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);
  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      writedata16(color);
    }
  }
}



#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void Optimized_ILI9341::setRotation(uint8_t m) {

  writecommand(ILI9341_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     writedata(MADCTL_MX | MADCTL_BGR);
     _width  = ILI9341_TFTWIDTH;
     _height = ILI9341_TFTHEIGHT;
     break;
   case 1:
     writedata(MADCTL_MV | MADCTL_BGR);
     _width  = ILI9341_TFTHEIGHT;
     _height = ILI9341_TFTWIDTH;
     break;
  case 2:
    writedata(MADCTL_MY | MADCTL_BGR);
     _width  = ILI9341_TFTWIDTH;
     _height = ILI9341_TFTHEIGHT;
    break;
   case 3:
     writedata(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
     _width  = ILI9341_TFTHEIGHT;
     _height = ILI9341_TFTWIDTH;
     break;
  }
}


void Optimized_ILI9341::invertDisplay(boolean i) {
  writecommand(i ? ILI9341_INVON : ILI9341_INVOFF);
}










uint8_t Optimized_ILI9341::spiread(void) {
	uint8_t r = 0;
	r = SPI.transfer(0x00);
	return r;
}

uint8_t Optimized_ILI9341::readdata(void) {
  uint8_t r;
       // Try to work directly with SPI registers...
       // First wait until output queue is empty
        uint16_t wTimeout = 0xffff;
        while (((SPI0.SR) & (15 << 12)) && (--wTimeout)) ; // wait until empty
        
//       	SPI0_MCR |= SPI_MCR_CLR_RXF; // discard any received data
//		SPI0_SR = SPI_SR_TCF;
        
        // Transfer a 0 out... 
        writedata(0);   
        
        // Now wait until completed. 
        wTimeout = 0xffff;
        while (((SPI0.SR) & (15 << 12)) && (--wTimeout)) ; // wait until empty
        r = SPI0.POPR;  // get the received byte... should check for it first...
    return r;
}
 
 

uint8_t Optimized_ILI9341::readcommand8(uint8_t c, uint8_t index)
{
    uint16_t wTimeout = 0xffff;
    uint8_t r;
    while (((SPI0.SR) & (15 << 12)) && (--wTimeout)) ; // wait until empty
    
    // Make sure the last frame has been sent...
    SPI0.SR = SPI_SR_TCF;   // dlear it out;
    wTimeout = 0xffff;
    while (!((SPI0.SR) & SPI_SR_TCF) && (--wTimeout)) ; // wait until it says the last frame completed

    // clear out any current received bytes
    wTimeout = 0x10;    // should not go more than 4...
    while ((((SPI0.SR) >> 4) & 0xf) && (--wTimeout))  {
        r = SPI0.POPR;
    }
    
    //writecommand(0xD9); // sekret command
	SPI0.PUSHR = 0xD9 | (pcs_command << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;
//	while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full

    // writedata(0x10 + index);
	SPI0.PUSHR = (0x10 + index) | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
//	while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full

    // writecommand(c);
   	SPI0.PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;
//	while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full

    // readdata
	SPI0.PUSHR = 0 | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
//	while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
        
    // Now wait until completed. 
    wTimeout = 0xffff;
    while (((SPI0.SR) & (15 << 12)) && (--wTimeout)) ; // wait until empty

    // Make sure the last frame has been sent...
    SPI0.SR = SPI_SR_TCF;   // dlear it out;
    wTimeout = 0xffff;
    while (!((SPI0.SR) & SPI_SR_TCF) && (--wTimeout)) ; // wait until it says the last frame completed

    wTimeout = 0x10;    // should not go more than 4...
    // lets get all of the values on the FIFO
    while ((((SPI0.SR) >> 4) & 0xf) && (--wTimeout))  {
        r = SPI0.POPR;
    }
    return r;  // get the received byte... should check for it first...
}


// KJE Added functions to read pixel data...
uint16_t Optimized_ILI9341::readPixel(int16_t x, int16_t y) {
  writecommand(ILI9341_CASET); // Column addr set
  writedata(x >> 8);
  writedata(x & 0xFF);     // XSTART 
  x++;
  writedata(x >> 8);
  writedata(x & 0xFF);     // XEND

  writecommand(ILI9341_PASET); // Row addr set
  writedata(y>>8);
  writedata(y);     // YSTART
  y++;
  writedata(y>>8);
  writedata(y);     // YEND

  writecommand(ILI9341_RAMRD); // write to RAM
  
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);
  uint16_t r = spiread();
  r <<= 8;
  r |= spiread();
  digitalWrite(_cs, HIGH);
   
  return r;

}



// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void Optimized_ILI9341::commandList(uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;

  writebegin();
  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    writecommand(pgm_read_byte(addr++)); //   Read, issue command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
      writedata(pgm_read_byte(addr++));  //     Read, issue argument
    }

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delay(ms);
    }
  }
}


void Optimized_ILI9341::begin(void)
{
	SPI.begin();
	if (SPI.pinIsChipSelect(_cs, _dc)) {
		pcs_data = SPI.setCS(_cs);
		pcs_command = pcs_data | SPI.setCS(_dc);
	} else {
		pcs_data = 0;
		pcs_command = 0;
		return;
	}
	// TODO: use transactions on all access to SPI
	SPI.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));
	SPI.endTransaction();

	// toggle RST low to reset
	if (_rst < 255) {
		pinMode(_rst, OUTPUT);
		digitalWrite(_rst, HIGH);
		delay(5);
		digitalWrite(_rst, LOW);
		delay(20);
		digitalWrite(_rst, HIGH);
		delay(150);
	}

  /*
  uint8_t x = readcommand8(ILI9341_RDMODE);
  Serial.print("\nDisplay Power Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDMADCTL);
  Serial.print("\nMADCTL Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDPIXFMT);
  Serial.print("\nPixel Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDIMGFMT);
  Serial.print("\nImage Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDSELFDIAG);
  Serial.print("\nSelf Diagnostic: 0x"); Serial.println(x, HEX);
*/
  //if(cmdList) commandList(cmdList);
  
  writecommand(0xEF);
  writedata(0x03);
  writedata(0x80);
  writedata(0x02);

  writecommand(0xCF);  
  writedata(0x00); 
  writedata(0XC1); 
  writedata(0X30); 

  writecommand(0xED);  
  writedata(0x64); 
  writedata(0x03); 
  writedata(0X12); 
  writedata(0X81); 
 
  writecommand(0xE8);  
  writedata(0x85); 
  writedata(0x00); 
  writedata(0x78); 

  writecommand(0xCB);  
  writedata(0x39); 
  writedata(0x2C); 
  writedata(0x00); 
  writedata(0x34); 
  writedata(0x02); 
 
  writecommand(0xF7);  
  writedata(0x20); 

  writecommand(0xEA);  
  writedata(0x00); 
  writedata(0x00); 
 
  writecommand(ILI9341_PWCTR1);    //Power control 
  writedata(0x23);   //VRH[5:0] 
 
  writecommand(ILI9341_PWCTR2);    //Power control 
  writedata(0x10);   //SAP[2:0];BT[3:0] 
 
  writecommand(ILI9341_VMCTR1);    //VCM control 
  writedata(0x3e); //对比度调节
  writedata(0x28); 
  
  writecommand(ILI9341_VMCTR2);    //VCM control2 
  writedata(0x86);  //--
 
  writecommand(ILI9341_MADCTL);    // Memory Access Control 
  writedata(0x48);

  writecommand(ILI9341_PIXFMT);    
  writedata(0x55); 
  
  writecommand(ILI9341_FRMCTR1);    
  writedata(0x00);  
  writedata(0x18); 
 
  writecommand(ILI9341_DFUNCTR);    // Display Function Control 
  writedata(0x08); 
  writedata(0x82);
  writedata(0x27);  
 
  writecommand(0xF2);    // 3Gamma Function Disable 
  writedata(0x00); 
 
  writecommand(ILI9341_GAMMASET);    //Gamma curve selected 
  writedata(0x01); 
 
  writecommand(ILI9341_GMCTRP1);    //Set Gamma 
  writedata(0x0F); 
  writedata(0x31); 
  writedata(0x2B); 
  writedata(0x0C); 
  writedata(0x0E); 
  writedata(0x08); 
  writedata(0x4E); 
  writedata(0xF1); 
  writedata(0x37); 
  writedata(0x07); 
  writedata(0x10); 
  writedata(0x03); 
  writedata(0x0E); 
  writedata(0x09); 
  writedata(0x00); 
  
  writecommand(ILI9341_GMCTRN1);    //Set Gamma 
  writedata(0x00); 
  writedata(0x0E); 
  writedata(0x14); 
  writedata(0x03); 
  writedata(0x11); 
  writedata(0x07); 
  writedata(0x31); 
  writedata(0xC1); 
  writedata(0x48); 
  writedata(0x08); 
  writedata(0x0F); 
  writedata(0x0C); 
  writedata(0x31); 
  writedata(0x36); 
  writedata(0x0F); 

  writecommand(ILI9341_SLPOUT);    //Exit Sleep 
  delay(120); 		
  writecommand(ILI9341_DISPON);    //Display on 

}




// end KJE
