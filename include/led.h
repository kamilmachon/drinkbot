#pragma once

#include <Adafruit_NeoPixel.h>

#include "config.h"


Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

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