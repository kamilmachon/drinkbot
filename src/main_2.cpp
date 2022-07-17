#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Adafruit_NeoPixel.h>

#include <config.h>


// #define SERIAL_DEBUG
// #define WEBSERIAL_DEBUG

#ifdef WEBSERIAL_DEBUG
  #include <WebSerial.h>
  #define SER WebSerial
#else
  #define SER Serial
#endif


AsyncWebServer server(PORT);


int pump_pwm = 200,
    mixer_pwm  = 160,
    servo_pwm  = 200,
    pump_time[4] = {0,0,0,0},
    mixer_time  = 5000,
    head_time   = 2000;

bool pump_enable[4] = {0,0,0,0},
     mixer_enable = false,
     up_enable    = false,
     down_enable  = false,
     prepraing = false;

int web_number_of_leds  = NUMPIXELS,
    web_led_red         = 255,
    web_led_green       = 255,
    web_led_blue        = 255;

int ratio = 1000;

unsigned long pump_end_time[4];
bool pump_running[4];
int pump_pin[] = {M1_FWD_PIN, M2_FWD_PIN, M3_FWD_PIN, M4_FWD_PIN};

unsigned long head_end_time, mixer_end_time;
bool mixer_running, up_running, down_running;

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
  // if (var == "PPWM")        return String(pump_pwm);
  // else if (var == "P1")     return pump_enable[0] ? "checked" : "";
  // else if (var == "P2")     return pump_enable[1] ? "checked" : "";
  // else if (var == "P3")     return pump_enable[2] ? "checked" : "";
  // else if (var == "P4")     return pump_enable[3] ? "checked" : "";
  // else if (var == "PT")     return String(pump_time);
  // else if (var == "MPWM")   return String(mixer_pwm);
  // else if (var == "MX")     return mixer_enable ? "checked" : "";
  // else if (var == "MT")     return String(mixer_time);
  // else if (var == "SPWM")   return String(servo_pwm);
  // else if (var == "UP")     return up_enable ? "checked" : "";
  // else if (var == "DOWN")   return down_enable ? "checked" : "";
  // else if (var == "ST")     return String(head_time);
  // else if (var == "NLED")   return String(web_number_of_leds);
  // else if (var == "MAXLED") return String(NUMPIXELS);
  // else if (var == "COLOR")  return "#" + String((web_led_red << 16) + (web_led_green << 8) + web_led_blue, 16);

  // SER.print("   Unknown processor Var:");
  // SER.println(var.c_str());
  return String();
}

enum PreparationState
{
  idle = 0,
  start = 10,
  start_lowering = 100,
  lowering = 110,
  start_pumping = 200,
  pumping = 210,
  start_mixing = 300,
  mixing = 310,
  start_raising = 400,
  raising = 410
} state;

void setup_server()
{

  if (!LittleFS.begin())
  {
      SER.print("An Error has occurred while mounting LittleFS");
      return;
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    SER.print("\nRequested home page... ");
    request->send(LittleFS, "/index.html", String(), false, processor);
  });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/style.css", "text/css");
  });

  server.on("/send", HTTP_GET, [](AsyncWebServerRequest *request) {
    SER.println("\n/send revieced");
    
    if (!prepraing)
    {
      SER.print("Updating recipe\nWEB Pumps:");
      pump_time[0] = request->hasParam("pump1") ? 
          request->getParam("pump1")->value().toInt()*ratio : 
          0.0;
      pump_time[1] = request->hasParam("pump2") ? 
          request->getParam("pump2")->value().toInt()*ratio : 
          0.0;
      pump_time[2] = request->hasParam("pump3") ? 
          request->getParam("pump3")->value().toInt()*ratio : 
          0.0;
      pump_time[3] = request->hasParam("pump4") ? 
          request->getParam("pump4")->value().toInt()*ratio : 
          0.0;
      SER.println();
      
      SER.print("WEB Mixer:");
      mixer_time = request->hasParam("shake_time") ? 
          request->getParam("shake_time")->value().toInt()*ratio : 
          0.0;
      SER.println();

      state = PreparationState::start_lowering;
    }
    else
    {
      SER.println("Already runing");
    }
  });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request) {
    SER.println("/stop");
    // TODO: Stop
  });

  server.onNotFound([](AsyncWebServerRequest *request) {
      request->send(404, "text/plain", "404: Not found");
  });

  state = PreparationState::idle;
  server.begin();
}

void StateMachine()
{
  bool _running = false, _enable = false;
  switch (state)
  {
    case PreparationState::idle:
      // SER.println("-> Idle");
      break;

    case PreparationState::start:
      state = PreparationState::start_lowering;
      SER.println("-> Start");
      break;

    // Lowering head
    case PreparationState::start_lowering:
      down_enable = true;
      state = PreparationState::lowering;
      SER.println("-> Start Lowering");
      break;

    // Waiting for lowering done
    case PreparationState::lowering:
      if (!down_running && !down_enable)
        {
          state = PreparationState::start_pumping;
          SER.println("-> Lowering done");
        }
      break;

    // Runing pumps
    case PreparationState::start_pumping:
        for (int i = 0; i<4; i++)
          pump_enable[i] = true;
        state = PreparationState::pumping;
        SER.println("-> Start Pumping");
        break;

    // Waiting for pumping done
    case PreparationState::pumping:
      for (int i = 0; i<4; i++)
        _running |= pump_running[i],
        _enable |= pump_enable[i];

      if (!_running && !_enable)
        {
          state = PreparationState::start_mixing;
          SER.println("-> Pumping done");
        }
        
      break;

    // Mixing
    case PreparationState::start_mixing:
      mixer_enable = true;
      state = PreparationState::mixing;
      SER.println("-> Start Mixing");
      break;
    
    // Waiting for mixing done
    case PreparationState::mixing:
      if (!mixer_running && !mixer_enable)
      {
        state = PreparationState::start_raising;
        SER.println("-> Mixing done");
      }
      break;
    
    // Rasing head
    case PreparationState::start_raising:
      up_enable = true;
      state = PreparationState::raising;
      SER.println("-> Start raising");
      break;

    // Waiting for rasing head done
    case PreparationState::raising:
      if (!up_running && !up_enable)
      {
        state = PreparationState::idle;
        SER.println("-> Done Raising");
      }
      break;

    default:
      state = PreparationState::idle;
  };
}

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
  digitalWrite(M2_FWD_PIN, 0);    // pump 2
  digitalWrite(M3_FWD_PIN, 0);    // pump 3
  digitalWrite(M4_FWD_PIN, 0);    // pump 4
}

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
        pump_end_time[i] = time + pump_time[i];
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

  // if (state == PreparationState::mixing)
  // {
  //   SER.printf("MX e:%i r:%i dt:%lu\n", mixer_enable, mixer_running, mixer_end_time-millis());
  // }
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


#ifndef SERIAL_DEBUG
  Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

    void setup_led()
    {
      pixels.begin();
      pixels.clear();
      for (int i=0; i<NUMPIXELS; i++) 
        pixels.setPixelColor(i, pixels.Color(255,255,255));
      pixels.show();
    }

    void set_rainbow()
    {
      int rainbow_time = 5000;
      for (int led=0; led<NUMPIXELS; led++)
      {
        int hue = uint32_t(((millis() + (rainbow_time * led / NUMPIXELS)) % rainbow_time) * 65535 / DELAYVAL);
        pixels.setPixelColor(led, pixels.ColorHSV(hue));
      }
    }

    void set_color(uint32_t color)
    { 
      for (int i =0; i<NUMPIXELS; i++)
        pixels.setPixelColor(i, color);
    }

    void loop_led()
    {
      pixels.clear();
      switch (state)
      {
      case idle:
        set_rainbow();
        break;

      case start_lowering:
      case lowering:
        set_color(pixels.Color(255,128,0));
        break;

      case start_pumping:
      case pumping:
        set_color(pixels.Color(128,255,0));
        break;

      case start_mixing:
      case mixing:
        set_color(pixels.Color(255,0,0));
        break;

      case start_raising:
      case raising:
        set_color(pixels.Color(255,128,0));
        break;

      default:
        break;
      }
      pixels.show();
    
    }

#endif


void setup()
{
  // SERIAL
  #if defined(SERIAL_DEBUG)
    SER.begin(115200);
  #elif defined(WEBSERIAL_DEBUG)
    SER.begin(&server);
  #endif
  
  SER.println("Setup started");
  
  // LED
  #if ! defined(SERIAL_DEBUG)
    SER.print("LED setup... ");
    setup_led();
    SER.println("done");
  #endif

  // PUMPS
  SER.print("Pumps setup... ");
  setup_pumps();
  SER.println("done");
  
  // HEAD
  SER.print("Head setup... ");
  setup_head();
  SER.println("done");
  
  // WIFI
  setup_wifi_ap();
  SER.print("Server setup...");
  setup_server();
  SER.println("done");
  SER.print("Server running on port: ");
  SER.println(PORT);

  SER.println("Setup complete");
}

void loop() {
  StateMachine();

  #ifndef SERIAL_DEBUG
    loop_led();
  #endif

    loop_pumps();
    loop_head();
    loop_head();
}
