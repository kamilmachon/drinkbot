#pragma once

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "bartender.h"
#include "recipe.h"
#include <vector>
#include <string>
#include <Servo.h>
#include "html.h"

String processor(const String& var) {
        return String();
}

class Webserver
{


public:
    Webserver(Servo &srv, AsyncWebServer* serverr): bartender(srv)
    {
        // this->server = server;
        server = serverr;

        // Initialize SPIFFS
        
        if(!SPIFFS.begin()){
            Serial.println("An Error has occurred while mounting SPIFFS");
            return;
        }

        WiFi.begin(ssid, password);
        // while (WiFi.status() != WL_CONNECTED) {
        //     delay(1000);
        //     Serial.println(WiFi.status());
        //     Serial.println("Connecting to WiFi..");
        // }
        delay(6000);
        Serial.println(WiFi.localIP());


        server->on("/", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(SPIFFS, "/index.html", String(), false, processor);
        });
        server->on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(SPIFFS, "/style.css", "text/css");
        });

        server->on("/send", HTTP_POST, std::bind(&Webserver::handleSendRequest, this, std::placeholders::_1));
        server->on("/stop", HTTP_GET, std::bind(&Webserver::handleStopRequest, this, std::placeholders::_1));
        Serial.println("starting srv");
        server->begin();
        Serial.println("server began");

    }

    void handleSendRequest(AsyncWebServerRequest *request){
        Serial.println("received send request");
        String pump_1 = request->getParam("pump1")->value();
        String pump_2 = request->getParam("pump2")->value();
        String pump_3 = request->getParam("pump3")->value();
        String pump_4 = request->getParam("pump4")->value();
        String shake_time_str = request->getParam("shake_time")->value();
        
        float pump1_ml, pump2_ml, pump3_ml, pump4_ml, shake_time;
        pump1_ml = static_cast<float>(pump_1.toInt());
        pump2_ml = static_cast<float>(pump_2.toInt());
        pump3_ml = static_cast<float>(pump_3.toInt());
        pump4_ml = static_cast<float>(pump_4.toInt());
        shake_time = static_cast<float>(shake_time_str.toInt());

        Recipe recipe;

        std::vector<float> vec;
        vec.push_back(pump1_ml);
        vec.push_back(pump2_ml);
        vec.push_back(pump3_ml);
        vec.push_back(pump4_ml);


        Serial.println("setting pumps");

        recipe.SetPumps(vec);
        recipe.SetShakeTime(shake_time);

        Serial.println("starting");

        bartender.Start(recipe);

        request->send(200, "text/plain", "OK");
    }


    void handleStopRequest(AsyncWebServerRequest *request){
        bartender.Stop();
        request->send(200, "text/plain", "OK");

    }

private:
    AsyncWebServer* server;
    Bartender bartender;
};