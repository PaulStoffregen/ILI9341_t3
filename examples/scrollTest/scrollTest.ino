/***************************************************
  This is our GFX example for the Adafruit ILI9341 Breakout and Shield
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


#include "SPI.h"
#include <ILI9341_t3.h>
#include "font_ComicSansMS.h"

// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 10
#define TFT_RS 8

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RS);

// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

void setup() {

  Serial.begin(9600);


 
  tft.begin();
  tft.setRotation(3);

  tft.fillScreen(ILI9341_BLACK);
  while (!Serial) ; 
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.enableScroll();
  tft.setScrollTextArea(0,0,120,240);
  tft.setScrollBackgroundColor(ILI9341_GREEN);

  tft.setCursor(180, 100);

  tft.setFont(ComicSansMS_16);
  tft.print("Fixed text");

  tft.setCursor(0, 0);

  tft.setTextColor(ILI9341_BLACK); 

  for(int i=0;i<10;i++){
    tft.print("this is line ");
    tft.println(i);
    delay(500);
  }

  tft.fillScreen(ILI9341_BLACK);
  tft.setScrollTextArea(40,50,120,120);
  tft.setScrollBackgroundColor(ILI9341_GREEN);
  tft.setFontAdafruit();
  tft.setTextSize(1);
  tft.setCursor(40, 50);

  for(int i=0;i<20;i++){
    tft.print("this is line ");
    tft.println(i);
    delay(500);
  }


}



void loop(void) {


}

