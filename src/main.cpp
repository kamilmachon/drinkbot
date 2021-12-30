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
#include "FS.h"

#include <fstream>


Servo shaker_servo;
std::unique_ptr<Webserver> web_server;
std::unique_ptr<Bartender> bartender;
void setup()
{
  Serial.begin(115200);
  pinMode(5, OUTPUT); // pump 0 (D1)
  pinMode(4, OUTPUT); // pump 1 (D2)
  pinMode(14, OUTPUT); // pump 2 (D5)
  pinMode(12, OUTPUT); // pump 3 (D6)
  pinMode(15, OUTPUT); // shaker (D8)
  pinMode(9, OUTPUT); // shaker down (DS2)
  pinMode(10, OUTPUT); // shaker up (DS3)

  analogWrite(15, 255);



  pinMode(16, OUTPUT); // trigger ()

  shaker_servo.attach(13); // D(7)

  web_server = std::make_unique<Webserver>(shaker_servo);

  Recipe recipe;
  bartender = std::make_unique<Bartender>(shaker_servo);
}



void loop() {
  switch (web_server->currentRequestState){
    case RequestState::received:
    {
      web_server->currentRequestState = RequestState::in_progress;
  analogWrite(15, 255);
  analogWrite(15, 255);
      t(web_server->currentRecipe);
      web_server->currentRequestState = RequestState::idle;
    }
    case RequestState::stop:
    {
      bartender->Stop();
      web_server->currentRequestState = RequestState::idle;
    }
    default:
      delay(100);
  }
      // std::vector<float> vec;
      // vec.push_back(10.0);
      // vec.push_back(20.0);
      // vec.push_back(30.0);
      // vec.push_back(10.0);

      // Serial.println("setting pumps");

      // recipe.SetPumps(vec);
      // recipe.SetShakeTime(5000);

      // Serial.println("starting");

      // bartender.Start(recipe);
      // Serial.println("finished");
      // std::fstream in("/index.html", std::ios_base::in | std::ios_base::binary);
      // for (int i = 0; i< 10; ++i)
      // {
      //   std::string line;
      //   std::getline(in, line);
      //   Serial.printf("%s", line);
      // }


}



