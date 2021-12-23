#pragma once

#include <Adafruit_NeoPixel.h>
// type correct number of pins
// verify num of pixels
#define LED 8
#define NUMPIXELS 20
#define DELAYVAL 500

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

class Led
{
    public:
        
        Led(){
            pixels.begin();
        }

        void runLed()
        {
            pixels.clear();

            for(int i=0; i<NUMPIXELS; i++) {

                pixels.setPixelColor(i, pixels.Color(255, 0, 0));
                pixels.show();
                delay(DELAYVAL);
            }
        }

    
};