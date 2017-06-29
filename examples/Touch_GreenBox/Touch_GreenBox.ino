#include <SPI.h>          // f.k. for Arduino-1.5.2
#include "Adafruit_GFX.h"// Hardware-specific library
#include <HX8347D_kbv.h>
HX8347D_kbv tft;
#include <XPT2046_Touchscreen.h>

//edit begin() method in XPT2046_Touchscreen.cpp source code :
//        attachInterrupt( digitalPinToInterrupt(tirqPin), isrPin, FALLING );  //.kbv

#define XPT_CS  4
#define XPT_IRQ 255       //use 3 if you fix interrupts in library source code

#define TS_LEFT 3900    //The XPT2046_Touchscreen works in Landscape
#define TS_RT   300     //I would expect Touch in Portrait
#define TS_TOP  360
#define TS_BOT  3800

XPT2046_Touchscreen ts(XPT_CS, XPT_IRQ);

// Assign human-readable names to some common 16-bit color values:
#define BLACK       0x0000
#define BLUE        0x001F
#define RED         0xF800
#define GREEN       0x07E0
#define CYAN        0x07FF
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0
#define WHITE       0xFFFF
#define DARK_GREY   0x7BEF
#define LIGHT_GREY  0xC618

void setup()
{
   Serial.begin(9600);
   uint16_t ID = tft.readID(); //
   tft.begin(ID);
   tft.setRotation(1);   //LANDSCAPE

   makeGUI();

   ts.begin();            //.kbv XPT2046_Touchscreen needs to start
}

void makeGUI()
{
   tft.fillScreen(DARK_GREY);
   tft.setTextSize(2);
   tft.setCursor(130, 6);
   tft.println("My Box");
   tft.setTextSize(1);
   tft.setCursor(100, 28);
   tft.println("(Touch to turn green)");

   tft.fillRect(45, 40, tft.width() - 90, tft.height() - 100, WHITE); //x-start, y-start, x-width, y-height
   tft.fillRect(50, 45, tft.width() - 100, tft.height() - 110, BLACK);
}

void loop()
{
   if (ts.touched()) {
       TS_Point p = ts.getPoint(); //XPT_2046_touchscreen returns in Landscape
       uint16_t y = map(p.y, TS_TOP, TS_BOT, 0, tft.height());
       uint16_t x = map(p.x, TS_LEFT, TS_RT, 0, tft.width());
       uint16_t w = tft.width() - 100;
       uint16_t h = tft.height() - 110;
       uint16_t color;
       if ((x > 50 && x < 50 + w) && (y > 45 && y < 45 + h))
           color = GREEN;
       else color = BLACK;
       tft.fillRect(50, 45, w, h, color);
       //        Serial.println("x=" + String(x) + " y=" + String(y) + " z=" + String(p.z) + "   ");
   }
}
