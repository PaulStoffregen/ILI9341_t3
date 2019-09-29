#include "ILI9488_t3.h"

const uint8_t* ILI9488_t3::init_commands() {
  static const uint8_t init_commands[] = {
    16, 0xE0,0x00,0x03,0x09,0x08,0x16,0x0A,0x3F,0x78,0x4C,0x09,0x0A,0x08,0x16,0x1A,0x0F,
    16, 0XE1,0x00,0x16,0x19,0x03,0x0F,0x05,0x32,0x45,0x46,0x04,0x0E,0x0D,0x35,0x37,0x0F,
    3, 0XC0,0x17,0x15,      //Power Control 1    //Vreg1out    //Verg2out
    2, 0xC1,0x41,      //Power Control 2    //VGH,VGL
    4, 0xC5,0x00,0x12,0x80,      //Power Control 3    //Vcom
    2, 0x36,0x48,      //Memory Access;
    2, 0x3A,0x66,      // Interface Pixel Format 	  //18 bit
    2, 0XB0,0x80,      // Interface Mode Control     			 //SDO NOT USE
    2, 0xB1,0xA0,      //Frame rate    //60Hz
    2, 0xB4,0x02,      //Display Inversion Control    //2-dot
    1, 0XB6,
    2, 0x02,0x02,    //Display Function Control  RGB/MCU Interface Control//MCU    //Source,Gate scan dieection
    2, 0XE9,0x00,     // Set Image Functio // Disable 24 bit data
    5, 0xF7,0xA9,0x51,0x2C,0x82,      // Adjust Control    // D7 stream, loose

    0
  };
  return init_commands;
}

void ILI9488_t3::write16BitColor(uint16_t color, bool last_pixel){
  // #if (__STM32F1__)
  //     uint8_t buff[4] = {
  //       (((color & 0xF800) >> 11)* 255) / 31,
  //       (((color & 0x07E0) >> 5) * 255) / 63,
  //       ((color & 0x001F)* 255) / 31
  //     };
  //     spi_port->dmaSend(buff, 3);
  // #else
#if defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
  uint8_t r = (color & 0xF800) >> 11;
  uint8_t g = (color & 0x07E0) >> 5;
  uint8_t b = color & 0x001F;

  r = (r * 255) / 31;
  g = (g * 255) / 63;
  b = (b * 255) / 31;
  uint32_t color24 = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  if (last_pixel)  {
	maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(23));
	_pimxrt_spi->TDR = color24;
	_pending_rx_count++;	//
	waitTransmitComplete();
  } else {
	maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(23) | LPSPI_TCR_CONT);
	_pimxrt_spi->TDR = color24;
	_pending_rx_count++;	//
	waitFifoNotFull();
  }

#elif defined(KINETISK)
  uint8_t r = (color & 0xF800) >> 11;
  r = (r * 255) / 31;
  writedata8_cont(r);

  uint8_t g = (color & 0x07E0) >> 5;
  g = (g * 255) / 63;
  writedata8_cont(g);

  uint8_t b = color & 0x001F;
  b = (b * 255) / 31;
  if (last_pixel)  {
  	writedata8_last(b);
  } else {
  	writedata8_cont(b);
  }
#elif defined(KINETISL)
  uint8_t r = (color & 0xF800) >> 11;

  r = (r * 255) / 31;
  setDataMode();
  outputToSPI(r);
  uint8_t g = (color & 0x07E0) >> 5;
  g = (g * 255) / 63;
  outputToSPIAlready8Bits(g);
  uint8_t b = color & 0x001F;
  b = (b * 255) / 31;
  outputToSPIAlready8Bits(b);
  if (last_pixel) {
	waitTransmitComplete();
  }

#endif
  // #endif
}


void ILI9488_t3::write16BitColor(uint16_t color, uint16_t count, bool last_pixel){
#if defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
  uint8_t r = (color & 0xF800) >> 11;
  uint8_t g = (color & 0x07E0) >> 5;
  uint8_t b = color & 0x001F;

  r = (r * 255) / 31;
  g = (g * 255) / 63;
  b = (b * 255) / 31;
  uint32_t color24 = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  while (count > 1) {
	maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(23) | LPSPI_TCR_CONT);
	_pimxrt_spi->TDR = color24;
	_pending_rx_count++;	//
	waitFifoNotFull();
	count--;
  }

  if (last_pixel)  {
	maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(23));
	_pimxrt_spi->TDR = color24;
	_pending_rx_count++;	//
	waitTransmitComplete();
  } else {
	maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(23) | LPSPI_TCR_CONT);
	_pimxrt_spi->TDR = color24;
	_pending_rx_count++;	//
	waitFifoNotFull();
  }

#elif defined(KINETISK)
  if (count < 2) {
  	write16BitColor(color, last_pixel);
  	return;
  }
  uint8_t r = (color & 0xF800) >> 11;
  r = (r * 255) / 31;
  writedata8_cont(r);

  uint8_t g = (color & 0x07E0) >> 5;
  g = (g * 255) / 63;
  writedata8_cont(g);

  uint8_t b = color & 0x001F;
  b = (b * 255) / 31;

  writedata8_cont(b);

  while (--count) {
	  writedata8_cont(r);
	  writedata8_cont(g);

	  if ((count == 1) && last_pixel)  {
	  	writedata8_last(b);
	  } else {
	  	writedata8_cont(b);
	  }
  }
#elif defined(KINETISL)
  uint8_t r = (color & 0xF800) >> 11;

  r = (r * 255) / 31;
  setDataMode();
  outputToSPI(r);
  uint8_t g = (color & 0x07E0) >> 5;
  g = (g * 255) / 63;
  outputToSPIAlready8Bits(g);
  uint8_t b = color & 0x001F;
  b = (b * 255) / 31;
  outputToSPIAlready8Bits(b);
  while (--count) {
	  outputToSPIAlready8Bits(r);
	  outputToSPIAlready8Bits(g);
	  outputToSPIAlready8Bits(b);
  }

  if (last_pixel) {
		waitTransmitComplete();
  }

#endif
}
