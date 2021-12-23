#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "Servo.h"
// #include "analogWrite.h"
// #include "NeoPixelBus.h"

#include <chrono>
#include <thread>
#include <vector>
#include <string>

#include <bartender.h>
#include <recipe.h>
#include <html.h>
#include "FS.h"

// #include <led.h>
// #include <led.h>
#define LED 2
#define DRINK_TIME 30
#define AMOUNT_OF_BELTS 150

String processor(const String& var) {
        return String();
}

std::map<int, int> pump_id_to_gpio = {{0, 5},
                                      {1, 4},
                                      {2, 14},
                                      {3, 12}};

Servo shaker_servo;
AsyncWebServer server(8080);
void setup() {
  Serial.begin(115200);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(16, OUTPUT);

  shaker_servo.attach(13);
}

void hardcoded_drink()
{
  shaker_servo.write(180);
  if (digitalRead(0) == LOW)
  {
    // Recipe recipe;
    // recipe.SetPumps(std::vector<float>{10, 5, 3, 5});
    // recipe.SetShakeTime(3);
    // Bartender bartender(shaker_servo);
    // bartender.Start(recipe);
    digitalWrite(pump_id_to_gpio[0], HIGH);
    digitalWrite(pump_id_to_gpio[1], HIGH);
    digitalWrite(pump_id_to_gpio[2], HIGH);
    digitalWrite(pump_id_to_gpio[3], HIGH);
    for (int i = 0; i < DRINK_TIME; ++i)
    {
      if (i > DRINK_TIME/2)
      {
        digitalWrite(pump_id_to_gpio[2], LOW);
      }
      if (i > DRINK_TIME/1.2)
      {
        digitalWrite(pump_id_to_gpio[1], LOW);
        digitalWrite(pump_id_to_gpio[3], LOW);
      }
      delay(1000);
    }
    digitalWrite(pump_id_to_gpio[0], LOW);
    shaker_servo.write(300);
    for (int i = 0; i < AMOUNT_OF_BELTS; ++i)
    {
      digitalWrite(16, HIGH);
      delay(20);
      digitalWrite(16, LOW);
      delay(20);
    }
    shaker_servo.write(180);
    }
};


void loop() {
  hardcoded_drink();
  // Led led;
  // led.runLed();
  // Recipe recipe;
  // recipe.SetPumps(std::vector<float>{5,5,5,5});
  // recipe.SetShakeTime(3);

  // Bartender bartender(shaker_servo);
  // // Serial.print("started\n");
  // bartender.Start(recipe);
  // // Serial.print("finished\n");
  // // Serial.print("dupsko");
  // delay(20000); //100 sec


  
  // analogWrite(10, 255);
  // analogWrite(10, 0);
  // for(int i = 0; i < 255; ++i)
  // {
  //   delay(50);
  // }
  // for (int i = 255; i > 0; --i)
  // {
  //   analogWrite(10, i);
  //   delay(50);
  // }
  // led.runLed();
}



