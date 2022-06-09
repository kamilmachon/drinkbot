#include <Arduino.h>

#define SERIAL_DEBUG
// #define LED_TEST
#define WIFI_TEST
// #define PUMP_TEST
// #define HEAD_TEST

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Adafruit_NeoPixel.h>


#include <config.h>


AsyncWebServer server(PORT);


int web_pump_pwm = 0,
    web_mx_pwm   = 0,
    web_srv_pwm  = 0;
bool web_pump1_state = false,
     web_pump2_state = false,
     web_pump3_state = false,
     web_pump4_state = false,
     web_mixer_state = false,
     web_up_state    = false,
     web_down_state  = false;
unsigned long web_pump_time = 0,
              web_mixer_time = 0,
              web_head_time = 0;


#define SERIAL_FILL_VAR(value, fill) for (int i = fill - ((value == 0) ? 1 : 0), v = value; i>0; i--, v/=10) Serial.print((v == 0) ? " " : "");

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
    if (var == "PPWM")
    {
      return String(web_pump_pwm);
    }
    else if (var == "P1")
    {
      return web_pump1_state ? "checked" : "";
    }
    else if (var == "P2")
    {
      return web_pump2_state ? "checked" : "";
    }
    else if (var == "P3")
    {
      return web_pump3_state ? "checked" : "";
    }
    else if (var == "P4")
    {
      return web_pump4_state ? "checked" : "";
    }
    else if (var == "PT")
    {
      return String(web_pump_time);
    }
    else if (var == "MPWM")
    {
      return String(web_mx_pwm);
    }
    else if (var == "MX")
    {
      return web_mixer_state ? "checked" : "";
    }
    else if (var == "MT")
    {
      return String(web_mixer_time);
    }
    else if (var == "SPWM")
    {
      return String(web_srv_pwm);
    }
    else if (var == "UP")
    {
      return web_up_state ? "checked" : "";
    }
    else if (var == "DOWN")
    {
      return web_down_state ? "checked" : "";
    }
    else if (var == "ST")
    {
      return String(web_head_time);
    }
    else
    {
      Serial.print("   Unknown processor Var:");
      Serial.println(var.c_str());
    }
    return String();
  }

  void setup_server()
  {

    if (!LittleFS.begin())
    {
        Serial.printf("An Error has occurred while mounting LittleFS");
        return;
    }
    // AsyncElegantOTA.begin(&server);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      Serial.println();
      Serial.print("Requested home page ");
      Serial.println(LittleFS.exists("/index_test.html") ? "File found" : "File not found");
      request->send(LittleFS, "/index_test.html", String(), false, processor);
    });

    server.on("/send", HTTP_GET, [](AsyncWebServerRequest *request) {
      Serial.println();
      Serial.println("WEB Update recieved");
      Serial.print("WEB Pumps:");
      if (request->hasParam("PPWM"))
        web_pump_pwm = request->getParam("PPWM")->value().toInt();
      Serial.print(" PWM:");
      Serial.print(web_pump_pwm);
      SERIAL_FILL_VAR(web_pump_pwm, 3)
      
      if (request->hasParam("PT"))  
        web_pump_time = request->getParam("PT")->value().toInt();
      Serial.print(" PT:");
      Serial.print(web_pump_time);
      SERIAL_FILL_VAR(web_pump_time, 4)

      web_pump1_state = request->hasParam("P1");
      Serial.print(" P1:");
      Serial.print(web_pump1_state);
      
      web_pump2_state = request->hasParam("P2");
      Serial.print(" P2:");
      Serial.print(web_pump2_state);

      web_pump3_state = request->hasParam("P3");
      Serial.print(" P3:");
      Serial.print(web_pump3_state);

      web_pump4_state = request->hasParam("P4");
      Serial.print(" P4:");
      Serial.println(web_pump4_state);


      Serial.print("WEB Mixer:");
      if (request->hasParam("MXPWM"))
        web_mx_pwm = request->getParam("MXPWM")->value().toInt();
      Serial.print(" PWM:");
      Serial.print(web_mx_pwm);
      SERIAL_FILL_VAR(web_mx_pwm, 3)

      if (request->hasParam("MT"))
        web_mixer_time = request->getParam("MT")->value().toInt();
      Serial.print(" MT:");
      Serial.print(web_mixer_time);
      SERIAL_FILL_VAR(web_mixer_time, 4)

      web_mixer_state = request->hasParam("MX");
      Serial.print(" MX:");
      Serial.println(web_mixer_state);


      Serial.print("WEB Servo:");
      if (request->hasParam("SPWM"))
        web_srv_pwm = request->getParam("SPWM")->value().toInt();
      Serial.print(" PWM:");
      Serial.print(web_srv_pwm);
      SERIAL_FILL_VAR(web_srv_pwm, 3)

      if (request->hasParam("ST"))
        web_head_time = request->getParam("ST")->value().toInt();
      Serial.print(" ST:");
      Serial.print(web_head_time);
      SERIAL_FILL_VAR(web_head_time, 4)

      web_up_state = request->hasParam("UP");
      Serial.print(" UP:");
      Serial.print(web_up_state);

      web_down_state = request->hasParam("DOWN");
      Serial.print(" DOWN:");
      Serial.println(web_down_state);

      request->redirect("/");
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
    Serial.print("Server running on port: ");
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



