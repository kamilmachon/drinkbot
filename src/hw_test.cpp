#include <Arduino.h>


#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <LittleFS.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

#include <config.h>

#include <fstream>



AsyncWebServer server(PORT);
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup_wifi_ap()
{
  Serial.print("Setting soft-AP ... ");
  // Serial.println(WiFi.softAP(SSID, PASSWORD, 1, false, 1) ? "Ready" : "Failed!");
  Serial.println(WiFi.softAP(SSID, PASSWORD) ? "Ready" : "Failed!");

  Serial.print("SSID:     ");
  Serial.println(SSID);
  Serial.print("PASSWORD: ");
  Serial.println(PASSWORD);
  Serial.print("IP:       ");
  Serial.println(WiFi.softAPIP());
}

void setup_hardware()
{
  analogWriteFreq(PWM_FREQ);  
  
  pinMode(M1_FWD_PIN, OUTPUT);    // pump 1
  pinMode(M2_FWD_PIN, OUTPUT);    // pump 2
  pinMode(M3_FWD_PIN, OUTPUT);    // pump 3
  pinMode(M4_FWD_PIN, OUTPUT);    // pump 4
  
  analogWrite(PUMPS_PWM_PIN, 0);  // pumps PWM
  digitalWrite(M1_FWD_PIN, 0);    // pump 1
  digitalWrite(M2_FWD_PIN, 0);    // pump 1
  digitalWrite(M3_FWD_PIN, 0);    // pump 1
  digitalWrite(M4_FWD_PIN, 0);    // pump 1

  
  pinMode(MX_FWD_PIN, OUTPUT);    // mixer enable
  analogWrite(MX_PWM_PIN, 0);     // mixer PWM
  digitalWrite(MX_FWD_PIN, 0);

  
  pinMode(SRV_UP_PIN, OUTPUT);    // servo up
  pinMode(SRV_DOWN_PIN, OUTPUT);  // servo down
  analogWrite(SRV_PWM_PIN, 0);    // servo PWM
  digitalWrite(SRV_UP_PIN, 0);    // servo up
  digitalWrite(SRV_DOWN_PIN, 0);  // servo dow

  pinMode(LED_PIN, OUTPUT);
}

String processor(const String& var){
  return String();
}

void setup_server()
{
  // AsyncElegantOTA.begin(&server);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
            Serial.print("requested home page");
            request->send(LittleFS, "/index_test.html", String(), false, processor);

        });

  server.begin();
}

void setup_led()
{

}

void setup()
{
  Serial.begin(115200);
  delay(2000);
  
  Serial.print("GPIO setup... ");
  setup_hardware();
  Serial.println("done");

  Serial.print("LED setup... ");
  setup_led();
  Serial.println("done");

  setup_wifi_ap();
  
  Serial.print("Server setup...");
  setup_server();
  Serial.println("done");
  Serial.print("Server runnint on port: ");
  Serial.println(PORT);

  Serial.println("Setup complete");
}

unsigned long last_update = 0;
void loop_led()
{
  if (millis() - last_update > 50)
  {
    last_update = millis();
    uint16_t led = (millis() / DELAYVAL) % NUMPIXELS,
            hue = uint16_t(float(millis() % DELAYVAL) * 65535 / DELAYVAL);

    pixels.clear();
    pixels.setPixelColor(led, pixels.ColorHSV(hue));
    pixels.show();
  }
}


void loop() {
  // loop_led();
  
}



