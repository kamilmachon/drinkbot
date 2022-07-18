#include <Arduino.h>

#define SERIAL_DEBUG
// #define LED_TEST
// #define WIFI_TEST
// #define PUMP_TEST
#define HEAD_TEST

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Adafruit_NeoPixel.h>

#include <config.h>


AsyncWebServer server(PORT);


#ifdef WIFI_TEST
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
#endif

#ifdef PUMP_TEST
  void setup_pumps()
  {
    analogWriteFreq(PWM_FREQ);
    // analogWriteRange(100);
    
    pinMode(M1_FWD_PIN, OUTPUT);    // pump 1
    pinMode(M2_FWD_PIN, OUTPUT);    // pump 2
    pinMode(M3_FWD_PIN, OUTPUT);    // pump 3
    pinMode(M4_FWD_PIN, OUTPUT);    // pump 4
    
    analogWrite(PUMPS_PWM_PIN, 0);  // pumps PWM
    digitalWrite(M1_FWD_PIN, 0);    // pump 1
    digitalWrite(M2_FWD_PIN, 0);    // pump 1
    digitalWrite(M3_FWD_PIN, 0);    // pump 1
    digitalWrite(M4_FWD_PIN, 0);    // pump 1
  }

  bool last_motor_state = false;
  int last_motor_pwm_freq = 0;

  void loop_pumps()
  {
    unsigned long time = millis(),
                  update_rate = 5000,
                  iter = time / update_rate;

    int pwm_freq = ((iter % 2) + 1) * 128 -1;
    bool motors_state = time % update_rate < update_rate / 2;

    if ( last_motor_state != motors_state or last_motor_pwm_freq != pwm_freq)
    {
      Serial.print("PWM: ");
      Serial.print(pwm_freq);
      Serial.print(" State: ");
      Serial.println(motors_state ? "1" : "0");
    }

    last_motor_state = motors_state;
    last_motor_pwm_freq = pwm_freq;

    analogWrite(PUMPS_PWM_PIN, pwm_freq);
    digitalWrite(M1_FWD_PIN, motors_state);
    digitalWrite(M2_FWD_PIN, motors_state);
    digitalWrite(M3_FWD_PIN, motors_state);
    digitalWrite(M4_FWD_PIN, motors_state);
  }
#endif

#ifdef HEAD_TEST
  void setup_head()
  {
    pinMode(MX_FWD_PIN, OUTPUT);    // mixer enable
    pinMode(SRV_UP_PIN, OUTPUT);    // servo up
    pinMode(SRV_DOWN_PIN, OUTPUT);  // servo down
    
    
    analogWrite(MX_PWM_PIN, 0);     // mixer PWM
    digitalWrite(MX_FWD_PIN, 0);    // mixer enable

    analogWrite(SRV_PWM_PIN, 0);    // servo PWM
    digitalWrite(SRV_UP_PIN, 0);    // servo up
    digitalWrite(SRV_DOWN_PIN, 0);  // servo dow
  }

  bool last_mx_state, last_up_state, last_down_state;
  void loop_head()
  {
    unsigned long time = millis();
    int mx_pwm_freq,
        srv_pwm_freq,
        iter = time / DELAYVAL;
    bool mx_enable = false,
         srv_up = false,
         srv_down = false;

    mx_pwm_freq = 0, //((iter / 4) % 10) * 20 + 75; // 8V 200
    srv_pwm_freq = ((iter / 4) % 10) * 20 + 75;
    srv_up = iter % 4 == 0;
    srv_down = iter % 4 == 2;

    analogWrite(MX_PWM_PIN, mx_pwm_freq);
    digitalWrite(MX_FWD_PIN, mx_enable);
    
    analogWrite(SRV_PWM_PIN, srv_pwm_freq);
    digitalWrite(SRV_UP_PIN, srv_up);
    digitalWrite(SRV_DOWN_PIN, srv_down);
    
    if (last_down_state != srv_down || last_up_state != srv_up || last_mx_state != mx_enable)
    {
      Serial.print("Mixer PWM: ");
      Serial.print(mx_pwm_freq);
      Serial.print(" EN: ");
      Serial.print(mx_enable ? "1" : "0");
      Serial.print("    Servo PWM: ");
      Serial.print(srv_pwm_freq);
      Serial.print(" /\\: ");
      Serial.print(srv_up ? "1" : "0");
      Serial.print(" \\/: ");
      Serial.println(srv_down ? "1" : "0");
    }
    last_down_state = srv_down;
    last_up_state = srv_up;
    last_mx_state = mx_enable;
  }
#endif

#if defined(LED_TEST) && ! defined(SERIAL_DEBUG)
  Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

  void setup_led()
  {
    // pinMode(LED_PIN, OUTPUT);
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
#endif

void setup()
{
  #ifdef SERIAL_DEBUG
    Serial.begin(115200);
    delay(2000);
  #endif
  
  Serial.println("Setup started");
  
  #if defined(LED_TEST) && ! defined(SERIAL_DEBUG)
    Serial.print("LED setup... ");
    setup_led();
    Serial.println("done");
  #endif
  
  
  #ifdef PUMP_TEST
    Serial.print("Pumps setup... ");
    setup_pumps();
    Serial.println("done");
  #endif

  #ifdef HEAD_TEST
    Serial.print("Head setup... ");
    setup_head();
    Serial.println("done");
  #endif
  
  #ifdef WIFI_TEST
    setup_wifi_ap();
    Serial.print("Server setup...");
    setup_server();
    Serial.println("done");
    Serial.print("Server runnint on port: ");
    Serial.println(PORT);
  #endif

  Serial.println("Setup complete");
}

void loop() {
  #if defined(LED_TEST) && ! defined(SERIAL_DEBUG)
    loop_led();
  #endif

  #ifdef PUMP_TEST
    loop_pumps();
  #endif

  #ifdef HEAD_TEST
    loop_head();
  #endif

  #ifdef HEAD_TEST
    loop_head();
  #endif
  
}



