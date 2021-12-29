#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "Servo.h"


#include <chrono>
#include <thread>
#include <vector>
#include <string>

#include <bartender.h>
#include <recipe.h>
#include <web_server.h>
#include <html.h>
// #include "FS.h"


Servo shaker_servo;
std::unique_ptr<Webserver> web_server;

void setup() 
{
  Serial.begin(115200);
  pinMode(5, OUTPUT); // pump 0 (D1)
  pinMode(4, OUTPUT); // pump 1 (D2)
  pinMode(14, OUTPUT); // pump 2 (D5)
  pinMode(12, OUTPUT); // pump 3 (D6)

  pinMode(16, OUTPUT); // trigger ()

  shaker_servo.attach(13); // D(7)

  web_server = std::make_unique<Webserver>(shaker_servo);
}

Recipe recipe;
Bartender bartender(shaker_servo);


void loop() {
  
  std::vector<float> vec;
  vec.push_back(10.0);
  vec.push_back(20.0);
  vec.push_back(30.0);
  vec.push_back(10.0);

  Serial.println("setting pumps");

  recipe.SetPumps(vec);
  recipe.SetShakeTime(1.0);

  Serial.println("starting");

  bartender.Start(recipe);
  Serial.println("finished");
  delay(1000);

}



