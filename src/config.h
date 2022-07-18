#pragma once
#include <ESP8266WiFi.h>


// Hardware setup
#define PWM_FREQ 1000       // PWM frequency [Hz]

// Pumps setup
#define PUMPS_PWM_PIN 16    // Pumps PWM pin    (D0)
#define M1_FWD_PIN 14       // Pump 1 run pin   (D5)
#define M2_FWD_PIN 12       // Pump 2 run pin   (D6)
#define M3_FWD_PIN 13       // Pump 3 run pin   (D7)
#define M4_FWD_PIN 15       // Pump 4 run pin   (D8)

// Mixer setup
#define MX_PWM_PIN 2        // Mixer PWM pin    (D4)
#define MX_FWD_PIN 0        // Mixer run pin    (D3)

// Servo setup
#define SRV_PWM_DUTY 
#define SRV_PWM_PIN 3       // Servo PWM pin    (RX)
#define SRV_UP_PIN 5        // Servo up pin     (D1)
#define SRV_DOWN_PIN 4      // Servo down pin   (D2)

// LED setup
#define LED_PIN 1           // LED data pin     (TX)
#define LED 8
#define NUMPIXELS 20
#define DELAYVAL 2000


// WIFI setup
#define SSID "KURWIGRZMOT"
#define PASSWORD "zdupydomordyzaur"

IPAddress local_ip(192,168,0,1);
IPAddress subnet(255,255,255,0);
IPAddress gateway(192,168,0,1);
#define PORT 80 //8082
