#pragma once

#include <recipe.h>
#include <Arduino.h>
#include <Servo.h>

#include <chrono>
#include <map>
#include <thread>
#include <unistd.h>

#include <config.h>

class Bartender
{
public:
    std::map<int, float> time_left_on_pumps;
    std::vector<bool> pumps_status{false, false, false, false};

    Recipe current_recipe;
    Bartender(Servo &servo) : servo_(servo)
    {
    }

    void Start(const Recipe &recipe)
    {
        Serial.print("start\n");
        current_recipe = recipe;
        Serial.print("starting pumps\n");
        StartPumps();
        Serial.print("wait for pumps\n");
        // StopPumpsAfterFinish();
        // Serial.print("shake\n");
        // if (current_recipe.GetShakeTime() > 0)
        // {
        //     StartShaker(current_recipe.GetShakeTime());
        // }
        // Serial.print("finished\n");
    }

    void Stop()
    {
        for (auto pump_it : time_left_on_pumps)
        {
            StopPump(pump_it.first);
        }
        stop_shaking = true;
    }

    void StopPump(int pump_id)
    {
        digitalWrite(pump_id_to_gpio[pump_id], LOW);
        pumps_status[pump_id] = false;
    }
    
    bool IsPumpActive()
    {
        for (auto pump : pumps_status)
        {
            if (pump)
            {
                return true;
            }
        }
        return false;
    }

    void StartShaker(int shake_time) // TODO: fix
    {
        DropShaker();
        analogWrite(MX_PWM_PIN, 64);
        digitalWrite(MX_FWD_PIN, 1);
        // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        // while (std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() < shake_time && stop_shaking)
        // {
        //     end = std::chrono::steady_clock::now();
        //     delay(100);
        // }
        delay(shake_time);
        digitalWrite(MX_FWD_PIN, 0);

        // analogWrite(15, 255);
        PullShaker();
    }

private:
    void StartPumps()
    {
        int count = 0;
        for (auto pump_time : current_recipe.GetPumpsTime())
        {
            if (pump_time > 0)
            {
                time_left_on_pumps[count] = pump_time;
                StartPump(count);
            }
            count++;
        }
    }

    // void StopPumpsAfterFinish()
    // {
    //     std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    //     Serial.print("before while");
    //     // while (IsPumpActive())
    //     while (IsPumpActive())

    //     {
    //         // Serial.print("1");
    //         std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    //         // Serial.print("2");
    //         auto miliseconds_passed = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    //         // Serial.print("3");
    //         for (auto it = time_left_on_pumps.begin(); it != time_left_on_pumps.end(); ++it)
    //         {
    //             // Serial.print("4");
    //             // Serial.printf("pump id: %d \n", it->first);
    //             if (it->second - (static_cast<float>(miliseconds_passed) / 1000.0) < 0 && pumps_status[it->first])
    //             {
    //                 // Serial.print("5");
    //                 StopPump(it->first);
    //             }
    //         }
    //         // Serial.print("6");
    //         delay(10);
    //         // Serial.print("6.5");
    //     }
    //     // Serial.print("7");
    // }

    void StartPump(int pump_id)
    {
        digitalWrite(pump_id_to_gpio[pump_id], HIGH);
        pumps_status[pump_id] = true;
    }

   
    void DropShaker()
    {
        // 9, 10 do inicjalizacji - op[isac w komentarzu jako SD2 i SD3
        // servo_.write(servo_angle_down);
        analogWrite(SRV_PWM_PIN, 128);
        digitalWrite(SRV_DOWN_PIN, HIGH);
        digitalWrite(SRV_UP_PIN, LOW);
        delay(pullupTimeMs);

        digitalWrite(SRV_DOWN_PIN, LOW);
        digitalWrite(SRV_UP_PIN, LOW);
    }

    void PullShaker()
    {
        analogWrite(SRV_PWM_PIN, 128);
        digitalWrite(SRV_DOWN_PIN, LOW);
        digitalWrite(SRV_UP_PIN, HIGH);
        delay(pullupTimeMs);

        digitalWrite(SRV_DOWN_PIN, LOW);
        digitalWrite(SRV_UP_PIN, LOW);
    }

    Servo servo_;

    int servo_angle_up = 180;
    int servo_angle_down = 300;
    bool stop_shaking = false;

    std::map<int, int> pump_id_to_gpio = {{0, M1_FWD_PIN},
                                          {1, M2_FWD_PIN},
                                          {2, M3_FWD_PIN},
                                          {3, M4_FWD_PIN}};

    const int pullupTimeMs = 250;
};