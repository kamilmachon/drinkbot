#pragma once

#include <vector>
#include <string>

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Servo.h>

#include "bartender.h"
#include "recipe.h"
#include "html.h"

String processor(const String &var)
{
    return String();
}

class Webserver
{

public:
    Webserver(Servo &srv) : bartender(srv),
                            server(std::make_unique<AsyncWebServer>(8080))
    {
        // Initialize SPIFFS
        if (!SPIFFS.begin())
        {
            Serial.printf("An Error has occurred while mounting SPIFFS");
            return;
        }

        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(1000);
            Serial.println(WiFi.status());
            Serial.println("Connecting to WiFi..");
        }

        delay(6000);
        Serial.println(WiFi.localIP());

        server->on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
            request->send(SPIFFS, "/index.html", String(), false, processor);
        });
        server->on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
            request->send(SPIFFS, "/style.css", "text/css");
        });

        server->on("/send", HTTP_POST, std::bind(&Webserver::handleSendRequest, this, std::placeholders::_1));
        server->on("/stop", HTTP_GET, std::bind(&Webserver::handleStopRequest, this, std::placeholders::_1));
        Serial.println("starting srv");
        server->begin();
        Serial.println("server began");
    }

    void handleSendRequest(AsyncWebServerRequest *request)
    {
        try
        {
            Serial.println("received send request");
            for (size_t i = 0; i < request->args(); ++i)
            {
                Serial.printf("%s\n", request->argName(i));
            }
            // float pump_1 = request->getParam("pump1")->value().toFloat();
            // float pump_2 = request->getParam("pump2")->value().toFloat();
            // float pump_3 = request->getParam("pump3")->value().toFloat();
            // float pump_4 = request->getParam("pump4")->value().toFloat();
            // String shake_time_str = request->getParam("shake_time")->value();

            // Serial.printf("request: pump1: %f, pump2: %f, pump3: %f, pump4: %f", pump_1, pump_2, pump_3, pump_4);

            // float pump1_ml, pump2_ml, pump3_ml, pump4_ml, shake_time;
            // pump1_ml = std::stof(pump_1);
            // pump2_ml = std::stof(pump_2);
            // pump3_ml = std::stof(pump_3);
            // pump4_ml = std::stof(pump_4);
            // shake_time = static_cast<float>(shake_time_str.toInt());
            float shake_time = 1.0;

            Recipe recipe;

            std::vector<float> vec;
            vec.push_back(10.0);
            vec.push_back(10.0);
            vec.push_back(10.0);
            vec.push_back(10.0);

            Serial.println("setting pumps");

            recipe.SetPumps(vec);
            recipe.SetShakeTime(1.0);

            Serial.println("starting");

            bartender.Start(recipe);

            Serial.println("response");

            request->send(200, "text/plain", "OK");
        }
        catch (const std::exception &e)
        {
            Serial.printf("%s\n", e.what());
        }
    }

    void handleStopRequest(AsyncWebServerRequest *request)
    {
        bartender.Stop();
        request->send(200, "text/plain", "OK");
    }

private:
    Bartender bartender;
    std::unique_ptr<AsyncWebServer> server;
};