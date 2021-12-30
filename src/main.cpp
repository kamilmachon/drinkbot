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
std::chrono::steady_clock::time_point begin;
void setup()
{
  Serial.begin(115200);
  pinMode(5, OUTPUT); // pump 0 (D1)
  pinMode(4, OUTPUT); // pump 1 (D2)
  pinMode(14, OUTPUT); // pump 2 (D5)
  pinMode(12, OUTPUT); // pump 3 (D6)
  pinMode(15, OUTPUT); // shaker (D8)
  pinMode(9, OUTPUT); // shaker updown (SD2)
  pinMode(10, OUTPUT); // shaker updown (SD3)

  analogWrite(15, 255);
  digitalWrite(9, LOW);
  digitalWrite(9, HIGH);

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
      bartender->Start(web_server->currentRecipe);
      begin = std::chrono::steady_clock::now();
      break;
    }
    case RequestState::in_progress:
    {
      if (bartender->IsPumpActive())
      {
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        auto miliseconds_passed = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        for (auto it = bartender->time_left_on_pumps.begin(); it != bartender->time_left_on_pumps.end(); ++it)
        {
          if (it->second - (static_cast<float>(miliseconds_passed) / 1000.0) < 0 && bartender->pumps_status[it->first])
          {
            bartender->StopPump(it->first);
          }
        }
        delay(10);
      }
      else {
        // TODO - currently blocking call
        bartender->StartShaker(web_server->currentRecipe.GetShakeTime());
        web_server->currentRequestState = RequestState::idle;
      }
      break;
    }
    case RequestState::stop:
    {
      bartender->Stop();
      web_server->currentRequestState = RequestState::idle;
      break;
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



