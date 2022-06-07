#include <Arduino.h>
#include <AsyncElegantOTA.h>

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
#include <config.h>
#include "FS.h"

#include <fstream>


Servo shaker_servo;
std::unique_ptr<Webserver> web_server;
std::unique_ptr<Bartender> bartender;
std::chrono::steady_clock::time_point begin;
void setup()
{
  Serial.begin(115200);
  
  analogWriteFreq(PWM_FREQ);

  analogWrite(PUMPS_PWM_PIN, 0);
  pinMode(M1_FWD_PIN, OUTPUT); // pump 1
  pinMode(M2_FWD_PIN, OUTPUT); // pump 2
  pinMode(M3_FWD_PIN, OUTPUT); // pump 3
  pinMode(M4_FWD_PIN, OUTPUT); // pump 4

  analogWrite(MX_PWM_PIN, 0);
  pinMode(MX_FWD_PIN, OUTPUT); // shaker (D7)

  analogWrite(SRV_PWM_PIN, 0);
  pinMode(SRV_UP_PIN, OUTPUT); // shaker updown (RX)
  pinMode(SRV_DOWN_PIN, OUTPUT); // shaker updown (TX)

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
    // bartender->StartShaker(1);
    // delay(1000);
    // delay(100);
}



