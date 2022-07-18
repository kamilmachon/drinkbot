#pragma once

#include <vector>
#include <string>

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Servo.h>
#include <LittleFS.h>

#include "bartender.h"
#include "recipe.h"
#include "config.h"

String processor(const String &var)
{
    return String();
}

enum RequestState{
    idle,
    received,
    in_progress,
    stop
};

class Webserver
{

public:
    RequestState currentRequestState;
    Recipe currentRecipe;
    Webserver(Servo &srv) : bartender(srv),
                            server(std::make_unique<AsyncWebServer>(PORT))
    {
        // Initialize LittleFS
        if (!LittleFS.begin())
        {
            Serial.printf("An Error has occurred while mounting LittleFS");
            return;
        }

        // WiFi.begin(SSID, PASSWORD);
        // while (WiFi.status() != WL_CONNECTED)
        // {
        //     delay(1000);
        //     Serial.println(WiFi.status());
        //     Serial.println("Connecting to WiFi..");
        // }

        // ESP as AP
        // Serial.print("Setting soft-AP configuration ... ");
        // Serial.println(WiFi.softAPConfig(local_ip, gateway, subnet) ? "Ready" : "Failed!");

        Serial.print("Setting soft-AP ... ");
        // Serial.println(WiFi.softAP(SSID, PASSWORD, 1, false, 1) ? "Ready" : "Failed!");
        Serial.println(WiFi.softAP(SSID, PASSWORD) ? "Ready" : "Failed!");

        Serial.print("SSID:     ");
        Serial.println(SSID);
        Serial.print("PASSWORD: ");
        Serial.println(PASSWORD);
        Serial.print("IP:       ");
        Serial.println(WiFi.softAPIP());
        

        // delay(6000);
        // Serial.println(WiFi.localIP());

        server->on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
            Serial.print("requested home page");
            request->send(LittleFS, "/index.html", String(), false, processor);

        });
        server->on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
            Serial.print("requested css");
            request->send(LittleFS, "/style.css", "text/css");
        });

        server->on("/send", HTTP_POST, std::bind(&Webserver::handleSendRequest, this, std::placeholders::_1));
        server->on("/stop", HTTP_GET, std::bind(&Webserver::handleStopRequest, this, std::placeholders::_1));
        
        
        
        Serial.println("starting srv");
        AsyncElegantOTA.begin(server.get());
        server->begin();
        Serial.println("server running");
    }

    void handleSendRequest(AsyncWebServerRequest *request)
    {   
        if (currentRequestState != RequestState::idle){
            request->send(400, "text/plain", "kurwa czekaj");
        }
        try
        {
            Serial.println("received send request");
            for (size_t i = 0; i < request->args(); ++i)
            {
                Serial.println(request->argName(i));
            }
            float pump_1 = request->getParam("pump1", true)->value().toFloat();
            float pump_2 = request->getParam("pump2", true)->value().toFloat();
            float pump_3 = request->getParam("pump3", true)->value().toFloat();
            float pump_4 = request->getParam("pump4", true)->value().toFloat();
            
            int shake_time = request->getParam("shake_time", true)->value().toInt();

            // Serial.printf("request: pump1: %f, pump2: %f, pump3: %f, pump4: %f", pump_1, pump_2, pump_3, pump_4);

            // float pump1_ml, pump2_ml, pump3_ml, pump4_ml, shake_time;
            // pump1_ml = std::stof(pump_1);
            // pump2_ml = std::stof(pump_2);
            // pump3_ml = std::stof(pump_3);
            // pump4_ml = std::stof(pump_4);
            // shake_time = static_cast<float>(shake_time_str.toInt());
            // float shake_time = 1.0;

            Recipe recipe;

            std::vector<float> vec;
            vec.push_back(pump_1);
            vec.push_back(pump_2);
            vec.push_back(pump_3);
            vec.push_back(pump_4);

            Serial.println("setting pumps");

            recipe.SetPumps(vec);
            recipe.SetShakeTime(shake_time); //miliseconds
            
            Serial.println("starting");
            currentRecipe = recipe;
            currentRequestState = RequestState::received;
            // bartender.Start(recipe);

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
        // bartender.Stop();
        currentRequestState = RequestState::stop;
        request->send(200, "text/plain", "OK");
    }

private:
    Bartender bartender;
    std::unique_ptr<AsyncWebServer> server;

    
};