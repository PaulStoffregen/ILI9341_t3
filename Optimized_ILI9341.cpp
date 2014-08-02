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
#include <avr/pgmspace.h>
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

#if defined(__SAM3X8E__)
#include <include/pio.h>
  #define SET_BIT(port, bitMask) (port)->PIO_SODR |= (bitMask)
  #define CLEAR_BIT(port, bitMask) (port)->PIO_CODR |= (bitMask)
  #define USE_SPI_LIBRARY
#endif
#ifdef __AVR__
  #define SET_BIT(port, bitMask) *(port) |= (bitMask)
  #define CLEAR_BIT(port, bitMask) *(port) &= ~(bitMask)
#endif
#if defined(__arm__) && defined(CORE_TEENSY)
  #define USE_SPI_LIBRARY
  #define SET_BIT(port, bitMask) digitalWrite(*(port), HIGH);
  #define CLEAR_BIT(port, bitMask) digitalWrite(*(port), LOW);
#endif

// Constructor when using software SPI.  All output pins are configurable.
Optimized_ILI9341::Optimized_ILI9341(int8_t cs, int8_t dc, int8_t mosi,
				   int8_t sclk, int8_t rst, int8_t miso) : Adafruit_GFX(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT) {
  _cs   = cs;
  _dc   = dc;
  _mosi  = mosi;
  _miso = miso;
  _sclk = sclk;
  _rst  = rst;
  _fUseSPILib = false;   // save default;
#if defined(__arm__) && defined(CORE_TEENSY)
  hwSPI = true; // will decide in begin function...
#else  
  hwSPI = false;
#endif
}


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
Optimized_ILI9341::Optimized_ILI9341(int8_t cs, int8_t dc, int8_t rst, boolean FSPILIB) : Adafruit_GFX(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT) {
  _cs   = cs;
  _dc   = dc;
  _rst  = rst;
  hwSPI = true;
  _mosi  = _miso = _sclk = 0;
  _fUseSPILib = FSPILIB;   // save default;
}

#if !defined(__arm__) || !defined(CORE_TEENSY)
void Optimized_ILI9341::writebegin(void) {
}
void Optimized_ILI9341::spiwrite(uint8_t c) {

  //Serial.print("0x"); Serial.print(c, HEX); Serial.print(", ");

  if (hwSPI) {
#if defined (__AVR__)
      uint8_t backupSPCR = SPCR;
    SPCR = mySPCR;
    SPDR = c;
    while(!(SPSR & _BV(SPIF)));
    SPCR = backupSPCR;
#elif defined(USE_SPI_LIBRARY)
#if defined(__SAM3X8E__)
    SPI.setClockDivider(11); // 85MHz / 11 = 7.6 MHz (full! speed!)
#endif
#if defined(__arm__) && defined(CORE_TEENSY)
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
#endif
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.transfer(c);
#endif
  } else {
    // Fast SPI bitbang swiped from LPD8806 library
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) {
	//digitalWrite(_mosi, HIGH); 
        SET_BIT(mosiport, mosipinmask);
      } else {
	//digitalWrite(_mosi, LOW); 
        CLEAR_BIT(mosiport, mosipinmask);
      }
      //digitalWrite(_sclk, HIGH);
      SET_BIT(clkport, clkpinmask);
      //digitalWrite(_sclk, LOW);
      CLEAR_BIT(clkport, clkpinmask);
    }
  }
}


void Optimized_ILI9341::writecommand(uint8_t c) {
  CLEAR_BIT(dcport, dcpinmask);
  //digitalWrite(_dc, LOW);
  CLEAR_BIT(clkport, clkpinmask);
  //digitalWrite(_sclk, LOW);
  CLEAR_BIT(csport, cspinmask);
  //digitalWrite(_cs, LOW);

  spiwrite(c);

  SET_BIT(csport, cspinmask);
  //digitalWrite(_cs, HIGH);
}


void Optimized_ILI9341::writedata(uint8_t c) {
  SET_BIT(dcport, dcpinmask);
  //digitalWrite(_dc, HIGH);
  CLEAR_BIT(clkport, clkpinmask);
  //digitalWrite(_sclk, LOW);
  CLEAR_BIT(csport, cspinmask);
  //digitalWrite(_cs, LOW);
  
  spiwrite(c);

  //digitalWrite(_cs, HIGH);
  SET_BIT(csport, cspinmask);
} 

void Optimized_ILI9341::writedata16(uint16_t d)
{
  SET_BIT(dcport, dcpinmask);
  CLEAR_BIT(csport, cspinmask);
  //Serial.print("D ");
  spiwrite(d >> 8);
  spiwrite(d);
  SET_BIT(csport, cspinmask);
} 

#else
    // Teensy version...
    
inline void Optimized_ILI9341::spiwrite(uint8_t c)
{
    if (_fUseSPILib)  {
        SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
        SPI.setBitOrder(MSBFIRST);
        SPI.setDataMode(SPI_MODE0);
        SPI.transfer(c);
    } else {    
        for (uint8_t bit = 0x80; bit; bit >>= 1) {
            *datapin = ((c & bit) ? 1 : 0);
            *clkpin = 1;
            *clkpin = 0;
        }
    }
}

void Optimized_ILI9341::writebegin(void) {
    if (_fUseSPILib) {
        SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
        SPI.setBitOrder(MSBFIRST);
        SPI.setDataMode(SPI_MODE0);
    }    
}

void Optimized_ILI9341::writecommand(uint8_t c)
{
	if (hwSPI) {
       // Serial.print("SPI CMD: ");
       // Serial.print(c, HEX);
		SPI0.PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0);
		while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
        //Serial.println(" completed");
	} else {
        //Serial.print("SPI CMD: ");
        //Serial.print(c, HEX);
		*rspin = 0;
        *clkpin = 0;
		*cspin = 0;
		spiwrite(c);
		*cspin = 1;
        //Serial.println(" completed");
	}
}

void Optimized_ILI9341::writedata(uint8_t c)
{
	if (hwSPI) {
        //Serial.print("SPI data: ");
        //Serial.print(c, HEX);
		SPI0.PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
		while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
        //Serial.println(" completed");
	} else {
        //Serial.print("SPI data: ");
        //Serial.print(c, HEX);
		*rspin = 1;
        *clkpin = 0;
		*cspin = 0;
		spiwrite(c);
		*cspin = 1;
        //Serial.println(" completed");
	}
}

void Optimized_ILI9341::writedata16(uint16_t d)
{
	if (hwSPI) {
		SPI0.PUSHR = d | (pcs_data << 16) | SPI_PUSHR_CTAS(1);
		while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
	} else {
        //Serial.print("SPI data: ");
        //Serial.print(d, HEX);
		*rspin = 1;
        *clkpin = 0;
		*cspin = 0;
		spiwrite(d >> 8);
		spiwrite(d);
		*cspin = 1;
        //Serial.println(" completed");
	}
}

#endif

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

// Try adding hardware SPI support...
#if defined(__arm__) && defined(CORE_TEENSY)
static bool spi_pin_is_cs(uint8_t pin)
{
	if (pin == 2 || pin == 6 || pin == 9) return true;
	if (pin == 10 || pin == 15) return true;
	if (pin >= 20 && pin <= 23) return true;
	return false;
}

static uint8_t spi_configure_cs_pin(uint8_t pin)
{
        switch (pin) {
                case 10: CORE_PIN10_CONFIG = PORT_PCR_MUX(2); return 0x01; // PTC4
                case 2:  CORE_PIN2_CONFIG  = PORT_PCR_MUX(2); return 0x01; // PTD0
                case 9:  CORE_PIN9_CONFIG  = PORT_PCR_MUX(2); return 0x02; // PTC3
                case 6:  CORE_PIN6_CONFIG  = PORT_PCR_MUX(2); return 0x02; // PTD4
                case 20: CORE_PIN20_CONFIG = PORT_PCR_MUX(2); return 0x04; // PTD5
                case 23: CORE_PIN23_CONFIG = PORT_PCR_MUX(2); return 0x04; // PTC2
                case 21: CORE_PIN21_CONFIG = PORT_PCR_MUX(2); return 0x08; // PTD6
                case 22: CORE_PIN22_CONFIG = PORT_PCR_MUX(2); return 0x08; // PTC1
                case 15: CORE_PIN15_CONFIG = PORT_PCR_MUX(2); return 0x10; // PTC0
        }
        return 0;
}

#define CTAR_24MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0) | SPI_CTAR_DBR)
#define CTAR_16MHz   (SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0) | SPI_CTAR_DBR)
#define CTAR_12MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0))
#define CTAR_8MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0))
#define CTAR_6MHz    (SPI_CTAR_PBR(0) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1))
#define CTAR_4MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1))
#ifdef LATER
void Optimized_ILI9341::setBitrate(uint32_t n)
{
	if (n >= 24000000) {
		ctar = CTAR_24MHz;
	} else if (n >= 16000000) {
		ctar = CTAR_16MHz;
	} else if (n >= 12000000) {
		ctar = CTAR_12MHz;
	} else if (n >= 8000000) {
		ctar = CTAR_8MHz;
	} else if (n >= 6000000) {
		ctar = CTAR_6MHz;
	} else {
		ctar = CTAR_4MHz;
	}
	SIM_SCGC6 |= SIM_SCGC6_SPI0;
	SPI0.MCR = SPI_MCR_MDIS | SPI_MCR_HALT;
	SPI0.CTAR0 = ctar | SPI_CTAR_FMSZ(7);
	SPI0.CTAR1 = ctar | SPI_CTAR_FMSZ(15);
	SPI0.MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
}
#endif //LATER
#endif


void Optimized_ILI9341::begin(void) {
  if (_rst > 0) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, LOW);
  }

  pinMode(_dc, OUTPUT);
  pinMode(_cs, OUTPUT);
#ifdef __AVR__
  csport    = portOutputRegister(digitalPinToPort(_cs));
  cspinmask = digitalPinToBitMask(_cs);
  dcport    = portOutputRegister(digitalPinToPort(_dc));
  dcpinmask = digitalPinToBitMask(_dc);
#endif
#if defined(__SAM3X8E__)
  csport    = digitalPinToPort(_cs);
  cspinmask = digitalPinToBitMask(_cs);
  dcport    = digitalPinToPort(_dc);
  dcpinmask = digitalPinToBitMask(_dc);
#endif
#if defined(__arm__) && defined(CORE_TEENSY) && 0   // try not using these here...
  mosiport = &_mosi;
  clkport = &_sclk;
  rsport = &_rst;
  csport    = &_cs;
  dcport    = &_dc;
  cspinmask = digitalPinToBitMask(_cs);
  dcpinmask = digitalPinToBitMask(_dc);
#endif

  if(hwSPI) { // Using hardware SPI
#ifdef __AVR__
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
#endif
#if defined(__SAM3X8E__)
    SPI.begin();
    SPI.setClockDivider(11); // 85MHz / 11 = 7.6 MHz (full! speed!)
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
#endif
#if defined(__arm__) && defined(CORE_TEENSY)
//===========================================
//  check for hardware SPI support...
//===========================================
    Serial.println("Check for Hardware SPI");
    // set default Clk and data if came in from default constructor..
    if (!_mosi) _mosi = 11;
    if (!_sclk) _sclk = 13;
    if (!_miso) _miso = 12;
    
	if (!_fUseSPILib    // user told us to not use the hardware... 
     &&spi_pin_is_cs(_cs) && spi_pin_is_cs(_dc)
	 && (_mosi == 7 || _mosi == 11)
	 && (_miso == 8 || _miso == 12)
	 && (_sclk == 13 || _sclk == 14)
	 && !(_cs ==  2 && _dc == 10) && !(_dc ==  2 && _cs == 10)
	 && !(_cs ==  6 && _dc ==  9) && !(_dc ==  6 && _cs ==  9)
	 && !(_cs == 20 && _dc == 23) && !(_dc == 20 && _cs == 23)
	 && !(_cs == 21 && _dc == 22) && !(_dc == 21 && _cs == 22) ) {
        Serial.println("Hardware SPI");
		hwSPI = true;
		if (_sclk == 13) {
			CORE_PIN13_CONFIG = PORT_PCR_MUX(2) | PORT_PCR_DSE;
			SPCR.setSCK(13);
		} else {
			CORE_PIN14_CONFIG = PORT_PCR_MUX(2);
			SPCR.setSCK(14);
		}
		if (_mosi == 11) {
			CORE_PIN11_CONFIG = PORT_PCR_MUX(2) | PORT_PCR_DSE;
			SPCR.setMOSI(11);
		} else {
			CORE_PIN7_CONFIG = PORT_PCR_MUX(2);
			SPCR.setMOSI(7);
		}

		if (_miso == 12) {
			CORE_PIN12_CONFIG = PORT_PCR_MUX(2);
			SPCR.setMISO(12);
		} else {
			CORE_PIN8_CONFIG = PORT_PCR_MUX(2);
			SPCR.setMISO(8);
		}
		ctar = CTAR_12MHz;
		pcs_data = spi_configure_cs_pin(_cs);
		pcs_command = pcs_data | spi_configure_cs_pin(_dc);
		SIM_SCGC6 |= SIM_SCGC6_SPI0;
		SPI0.MCR = SPI_MCR_MDIS | SPI_MCR_HALT;
		SPI0.CTAR0 = ctar | SPI_CTAR_FMSZ(7);
		SPI0.CTAR1 = ctar | SPI_CTAR_FMSZ(15);
		SPI0.MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
	} else {
		pinMode(_cs, OUTPUT);
		pinMode(_dc, OUTPUT);
		hwSPI = false;
		cspin = portOutputRegister(digitalPinToPort(_cs));
		rspin = portOutputRegister(digitalPinToPort(_dc));
		clkpin = portOutputRegister(digitalPinToPort(_sclk));
		datapin = portOutputRegister(digitalPinToPort(_mosi));
		*cspin = 1;
		*rspin = 0;
        _fUseSPILib = ((_mosi == 11) &&  (_sclk == 13));

        if (_fUseSPILib) {
            Serial.println("SPI Library");
            SPI.begin();
            SPI.setClockDivider(SPI_CLOCK_DIV2); // 4 MHz
            SPI.setBitOrder(MSBFIRST);
            SPI.setDataMode(SPI_MODE0);
        } else {    
            Serial.println("BitBang SPI");
            pinMode(_sclk, OUTPUT);
            pinMode(_mosi, OUTPUT);
            *clkpin = 0;
            *datapin = 0;
        }
    }
//============================================
#endif
#ifdef __AVR__
    mySPCR = SPCR;
#endif
#if defined(__arm__) && defined(CORE_TEENSY)
// All of the code needed for teensy cases handled above
#else
  } else {
    pinMode(_sclk, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
#ifdef __AVR__
    clkport     = portOutputRegister(digitalPinToPort(_sclk));
    clkpinmask  = digitalPinToBitMask(_sclk);
    mosiport    = portOutputRegister(digitalPinToPort(_mosi));
#endif
#if defined(__SAM3X8E__)
    clkport     = digitalPinToPort(_sclk);
    clkpinmask  = digitalPinToBitMask(_sclk);
    mosiport    = digitalPinToPort(_mosi);
#endif
    mosipinmask = digitalPinToBitMask(_mosi);
    CLEAR_BIT(clkport, clkpinmask);
    CLEAR_BIT(mosiport, mosipinmask);
#endif    
  }

  // toggle RST low to reset
  if (_rst > 0) {
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


// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Optimized_ILI9341::color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
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


////////// stuff not actively being used, but kept for posterity


#if defined(__arm__) && defined(CORE_TEENSY)
// Teensy version
uint8_t Optimized_ILI9341::spiread(void) {
  uint8_t r = 0;

  if (_fUseSPILib) {
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    r = SPI.transfer(0x00);
  } else {

    for (uint8_t i=0; i<8; i++) {
      digitalWrite(_sclk, LOW);
      digitalWrite(_sclk, HIGH);
      r <<= 1;
      if (digitalRead(_miso))
	r |= 0x1;
    }
  }
  //Serial.print("read: 0x"); Serial.print(r, HEX);
  
  return r;
}

uint8_t Optimized_ILI9341::readdata(void) {
  uint8_t r;
  if (hwSPI) {
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
    } else {
        // not using the fast mode
        digitalWrite(_dc, HIGH);
        digitalWrite(_cs, LOW);
        r = spiread();
        digitalWrite(_cs, HIGH);
    }   
    return r;
}

#else // Not Teensy
uint8_t Optimized_ILI9341::spiread(void) {
  uint8_t r = 0;

  if (hwSPI) {
#if defined (__AVR__)
    uint8_t backupSPCR = SPCR;
    SPCR = mySPCR;
    SPDR = 0x00;
    while(!(SPSR & _BV(SPIF)));
    r = SPDR;
    SPCR = backupSPCR;
#elif defined (USE_SPI_LIBRARY)
#if defined(__SAM3X8E__)
    SPI.setClockDivider(11); // 85MHz / 11 = 7.6 MHz (full! speed!)
#endif
#if defined(__arm__) && defined(CORE_TEENSY)
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
#endif
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    r = SPI.transfer(0x00);
#endif
  } else {

    for (uint8_t i=0; i<8; i++) {
      digitalWrite(_sclk, LOW);
      digitalWrite(_sclk, HIGH);
      r <<= 1;
      if (digitalRead(_miso))
	r |= 0x1;
    }
  }
  //Serial.print("read: 0x"); Serial.print(r, HEX);
  
  return r;
}

uint8_t Optimized_ILI9341::readdata(void) {
   digitalWrite(_dc, HIGH);
   digitalWrite(_cs, LOW);
   uint8_t r = spiread();
   digitalWrite(_cs, HIGH);
   
   return r;
}
#endif
 
 

uint8_t Optimized_ILI9341::readcommand8(uint8_t c, uint8_t index) {
#if defined(__arm__) && defined(CORE_TEENSY)
  if (hwSPI) {
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

    // return  readdata();
  } 
#endif // Teensy  
   digitalWrite(_dc, LOW); // command
   digitalWrite(_cs, LOW);
   spiwrite(0xD9);  // woo sekret command?
   digitalWrite(_dc, HIGH); // data
   spiwrite(0x10 + index);
   digitalWrite(_cs, HIGH);

   digitalWrite(_dc, LOW);
//   digitalWrite(_sclk, LOW);
   digitalWrite(_cs, LOW);
   spiwrite(c);
 
   digitalWrite(_dc, HIGH);
   uint8_t r = spiread();
   digitalWrite(_cs, HIGH);
   return r;
}


 
/*

 uint16_t Optimized_ILI9341::readcommand16(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 uint16_t r = spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 uint32_t Optimized_ILI9341::readcommand32(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 
 dummyclock();
 dummyclock();
 
 uint32_t r = spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 */
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



// end KJE
