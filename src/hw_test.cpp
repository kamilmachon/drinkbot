#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Adafruit_NeoPixel.h>

#include <config.h>


// #define SERIAL_DEBUG
// #define WEBSERIAL_DEBUG
#define WIFI_TEST

#define PUMP_TEST
// #define PUMP_AUTO_TEST
#define HEAD_TEST
#define LED_TEST
// #define LED_AUTO_TEST

#ifdef WEBSERIAL_DEBUG
  #include <WebSerial.h>
  #define SER WebSerial
#else
  #define SER Serial
#endif


AsyncWebServer server(PORT);


int pump_pwm = 0,
    mixer_pwm   = 0,
    servo_pwm  = 0,
    pump_time   = 0,
    mixer_time  = 0,
    head_time   = 0;
bool pump_enable[4],
     mixer_enable = false,
     up_enable    = false,
     down_enable  = false;
int web_number_of_leds  = NUMPIXELS,
    web_led_red         = 255,
    web_led_green       = 255,
    web_led_blue        = 255;



#ifdef WIFI_TEST
  #define PARAM_INT(name, var, fill) if (request->hasParam(name)) var = request->getParam(name)->value().toInt(); serial_print_parameter(name, var, fill);
  #define PARAM_BOOL(name, var) var = request->hasParam(name); serial_print_parameter(name, var, 0);

  void serial_print_parameter(String name, int value, int fill=0)
  {
    SER.print(" " + name + ":"); 
    SER.print(value); 
    for (int i = fill - ((value == 0) ? 1 : 0), v = value; i>0; i--, v/=10) 
      SER.print((v == 0) ? " " : "");
  }

  void setup_wifi_ap()
  {
    SER.print("Setting soft-AP ... ");
    // SER.println(WiFi.softAP(SSID, PASSWORD, 1, false, 1) ? "Ready" : "Failed!");
    SER.println(WiFi.softAP(SSID, PASSWORD) ? "Ready" : "Failed!");

    SER.print("SSID:     ");
    SER.println(SSID);
    SER.print("PASSWORD: ");
    SER.println(PASSWORD);
    SER.print("IP:       ");
    SER.println(WiFi.softAPIP().toString());

  }

  String processor(const String& var){
    if (var == "PPWM")        return String(pump_pwm);
    else if (var == "P1")     return pump_enable[0] ? "checked" : "";
    else if (var == "P2")     return pump_enable[1] ? "checked" : "";
    else if (var == "P3")     return pump_enable[2] ? "checked" : "";
    else if (var == "P4")     return pump_enable[3] ? "checked" : "";
    else if (var == "PT")     return String(pump_time);
    else if (var == "MPWM")   return String(mixer_pwm);
    else if (var == "MX")     return mixer_enable ? "checked" : "";
    else if (var == "MT")     return String(mixer_time);
    else if (var == "SPWM")   return String(servo_pwm);
    else if (var == "UP")     return up_enable ? "checked" : "";
    else if (var == "DOWN")   return down_enable ? "checked" : "";
    else if (var == "ST")     return String(head_time);
    else if (var == "NLED")   return String(web_number_of_leds);
    else if (var == "MAXLED") return String(NUMPIXELS);
    else if (var == "COLOR")  return "#" + String((web_led_red << 16) + (web_led_green << 8) + web_led_blue, 16);

    SER.print("   Unknown processor Var:");
    SER.println(var.c_str());
    return String();
  }

  void setup_server()
  {

    if (!LittleFS.begin())
    {
        SER.print("An Error has occurred while mounting LittleFS");
        return;
    }

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      SER.println();
      SER.print("Requested home page... ");
      SER.println(LittleFS.exists("/index_test.html") ? "File found" : "File not found");
      request->send(LittleFS, "/index_test.html", String(), false, processor);
    });

    server.on("/send", HTTP_GET, [](AsyncWebServerRequest *request) {
      SER.println();
      SER.println("\nWEB Update recieved");

      SER.print("WEB Pumps:");
      PARAM_INT("PPWM", pump_pwm, 3)
      PARAM_INT("PT", pump_time, 4)
      PARAM_BOOL("P1", pump_enable[0])
      PARAM_BOOL("P2", pump_enable[1])
      PARAM_BOOL("P3", pump_enable[2])
      PARAM_BOOL("P4", pump_enable[3])
      SER.println();
      
      SER.print("WEB Mixer:");
      PARAM_INT("MPWM", mixer_pwm, 3)
      PARAM_INT("MT", mixer_time, 4)
      PARAM_BOOL("MX", mixer_enable)
      SER.println();

      SER.print("WEB Servo:");
      PARAM_INT("SPWM", servo_pwm, 3)
      PARAM_INT("ST", head_time, 4)
      PARAM_BOOL("UP", up_enable)
      PARAM_BOOL("DOWN", down_enable)
      SER.println();
      

      SER.print("WEB Led:  ");
      PARAM_INT("NLED", web_number_of_leds, 3)
      if (request->hasParam("COLOR"))
      {
        int color = (int) strtol(&(request->getParam("COLOR")->value().c_str()[1]), NULL, HEX);
        web_led_blue = color & 0xff;
        web_led_green = (color >> 8) & 0xff;
        web_led_red = color >> 16;
      }
      char msg[64];
      sprintf(msg, "C:%s (r:%i g:%i b:%i)\n",
              request->hasParam("COLOR") ? request->getParam("COLOR")->value().c_str() : "#______",
              web_led_red, web_led_green, web_led_blue);
      SER.print(msg);

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

  #ifdef PUMP_AUTO_TEST
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
        SER.print("PWM: ");
        SER.print(pwm_freq);
        SER.print(" State: ");
        SER.println(motors_state ? "1" : "0");
      }

      last_motor_state = motors_state;
      last_motor_pwm_freq = pwm_freq;

      analogWrite(PUMPS_PWM_PIN, pwm_freq);
      digitalWrite(M1_FWD_PIN, motors_state);
      digitalWrite(M2_FWD_PIN, motors_state);
      digitalWrite(M3_FWD_PIN, motors_state);
      digitalWrite(M4_FWD_PIN, motors_state);
    }
  #else
    unsigned long pump_end_time[4];
    bool pump_running[4];
    int pump_pin[] = {M1_FWD_PIN, M2_FWD_PIN, M3_FWD_PIN, M4_FWD_PIN};

    void loop_pumps()
    {
      unsigned long time = millis();
      analogWrite(PUMPS_PWM_PIN, pump_pwm);
      for (int i=0; i<4; i++)
      {
        if (pump_enable[i])
        {
          if (!pump_running[i])
          {
            pump_running[i] = true;
            pump_end_time[i] = time + pump_time;
            SER.println("+ Pump " + String(i));
          }
          else if(pump_end_time[i] < time)
          {
            pump_running[i] = false;
            pump_enable[i] = false;
            SER.println("- Pump " + String(i));
          }
        }
        else
        {
          pump_running[i] = false;
        }
        digitalWrite(pump_pin[i], pump_running[i]);
      }
    }
  #endif
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

  unsigned long head_end_time, mixer_end_time;
  bool mixer_running, up_running, down_running;
  void loop_head()
  {
    unsigned long time = millis();
    analogWrite(SRV_PWM_PIN, servo_pwm);
    if (up_enable || down_enable)
    {
      if (up_enable && down_enable)
      {
        up_running = false;
        down_running = false;
        up_enable = false;
        down_enable = false;
      }
      else if (!(up_running or down_running))
      {
        up_running = up_enable;
        down_running = down_enable;
        head_end_time = time + head_time;
      }
      else if(head_end_time < time)
      {
        up_running = false;
        down_running = false;
        up_enable = false;
        down_enable = false;
      }
    }
    else
    {
      up_running = false;
      down_running = false;
    }
    digitalWrite(SRV_UP_PIN, up_running);
    digitalWrite(SRV_DOWN_PIN, down_running);

    analogWrite(MX_PWM_PIN, mixer_pwm);
    if (mixer_enable)
    {
      if (!mixer_running)
      {
        mixer_running = true;
        mixer_end_time = time + mixer_time;
    }
      else if(mixer_end_time < time)
      {
        mixer_enable = false;
        mixer_running = false;
      }
    }
    else
    {
      mixer_running = false;
    }
    digitalWrite(MX_FWD_PIN, mixer_running);
  }
#endif

#if defined(LED_TEST) && ! defined(SERIAL_DEBUG)
  Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

  int _nled   = web_number_of_leds,
      _color  = (web_led_red<<16) + (web_led_green<<8) + web_led_blue;  

  #ifdef LED_AUTO_TEST
    void setup_led()
    {
      pixels.begin();
      pixels.clear();
      pixels.show();
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
  #else
    void setup_led()
    {
      pixels.begin();
      pixels.clear();
      for (int i=0; i<_nled; i++)
        pixels.setPixelColor(i, _color);
      pixels.show();
    }

    void loop_led()
    {
      int web_color = (web_led_red<<16) + (web_led_green<<8) + web_led_blue;
      if (_nled != web_number_of_leds or _color != web_color)
      {
        char message[64];
        sprintf(message, "LED n:%i %i->%i (r:%i g:%i b:%i)\n",_nled, _color, web_color, web_led_red, web_led_green, web_led_blue);
        SER.print(message);
        _nled = web_number_of_leds;
        _color = web_color;
        pixels.clear();
        for (int i=0; i<_nled; i++)
          pixels.setPixelColor(i, _color);
        pixels.show();
      }
    }
  #endif
#endif


void setup()
{
  #if defined(SERIAL_DEBUG)
    SER.begin(115200);
  #elif defined(WEBSERIAL_DEBUG)
    SER.begin(&server);
  #endif
  
  SER.println("Setup started");
  
  #if defined(LED_TEST) && ! defined(SERIAL_DEBUG)
    SER.print("LED setup... ");
    setup_led();
    SER.println("done");
  #endif
  
  
  #ifdef PUMP_TEST
    SER.print("Pumps setup... ");
    setup_pumps();
    SER.println("done");
  #endif

  #ifdef HEAD_TEST
    SER.print("Head setup... ");
    setup_head();
    SER.println("done");
  #endif
  
  #ifdef WIFI_TEST
    setup_wifi_ap();
    SER.print("Server setup...");
    setup_server();
    SER.println("done");
    SER.print("Server running on port: ");
    SER.println(PORT);
  #endif

  SER.println("Setup complete");
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
