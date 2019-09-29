#ifndef _ILI9488_t3_
#define _ILI9488_t3_

#include "ILI9341_t3.h"

#define ILI9488_TFTWIDTH  320
#define ILI9488_TFTHEIGHT 480

class ILI9488_t3 : public ILI9341_t3 {
  public:
  	ILI9488_t3(uint8_t _CS, uint8_t _DC, uint8_t _RST = 255, uint8_t _MOSI=11, uint8_t _SCLK=13, uint8_t _MISO=12) :
  	  ILI9341_t3( _CS,  _DC,  _RST ,  _MOSI,  _SCLK,  _MISO) {
  	  _width = ILI9488_TFTWIDTH;
  	  _height = ILI9488_TFTHEIGHT;
  	};

  protected:
    uint16_t hwWidth() override { return ILI9488_TFTWIDTH; }
    uint16_t hwHeight() override { return ILI9488_TFTHEIGHT; }
    const uint8_t* init_commands() override;
    void write16BitColor(uint16_t color, bool last_pixel=false) override;
    void write16BitColor(uint16_t color, uint16_t count, bool last_pixel) override;
};

#endif
