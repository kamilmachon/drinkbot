#pragma once

#include <recipe.h>
#include <Arduino.h>
#include <Servo.h>

#include <chrono>
#include <map>
#include <thread>
#include <unistd.h>

class Bartender
{
public:
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
        StopPumpsAfterFinish();
        Serial.print("shake\n");
        if (current_recipe.GetShakeTime() > 0)
        {
            StartShaker(current_recipe.GetShakeTime());
        }
        Serial.print("finished\n");
    }

    void Stop()
    {
        for (auto pump_it : time_left_on_pumps)
        {
            StopPump(pump_it.first);
        }
        stop_shaking = true;
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

    void StopPumpsAfterFinish()
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        Serial.print("before while");
        // while (IsPumpActive())
        while (IsPumpActive())

        {
            // Serial.print("1");
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            // Serial.print("2");
            auto miliseconds_passed = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            // Serial.print("3");
            for (auto it = time_left_on_pumps.begin(); it != time_left_on_pumps.end(); ++it)
            {
                // Serial.print("4");
                // Serial.printf("pump id: %d \n", it->first);
                if (it->second - (static_cast<float>(miliseconds_passed) / 1000.0) < 0 && pumps_status[it->first])
                {
                    // Serial.print("5");
                    StopPump(it->first);
                }
            }
            // Serial.print("6");
            delay(10);
            // Serial.print("6.5");
        }
        // Serial.print("7");
    }

    void StartPump(int pump_id)
    {
        digitalWrite(pump_id_to_gpio[pump_id], HIGH);
        pumps_status[pump_id] = true;
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
        digitalWrite(15, HIGH);
        // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        // while (std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() < shake_time && stop_shaking)
        // {
        //     end = std::chrono::steady_clock::now();
        //     delay(100);
        // }
        delay(shake_time);
        digitalWrite(15, LOW);
        PullShaker();
    }

    void DropShaker()
    {
        servo_.write(servo_angle_down);
    }

    void PullShaker()
    {
        servo_.write(servo_angle_up);
    }

    std::map<int, float> time_left_on_pumps;
    std::vector<bool> pumps_status{false, false, false, false};

    Recipe current_recipe;
    Servo servo_;

    int servo_angle_up = 180;
    int servo_angle_down = 300;
    bool stop_shaking = false;

    std::map<int, int> pump_id_to_gpio = {{0, 5},
                                          {1, 4},
                                          {2, 14},
                                          {3, 12}};
};