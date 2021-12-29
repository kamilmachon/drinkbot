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
    Bartender(Servo& servo) : servo_(servo)
    {
    }

    void Start(const Recipe &recipe)
    {
        current_recipe = recipe;
        StartPumps();
        StopPumpsAfterFinish();
        if (current_recipe.GetShakeTime() > 0)
        {
            StartShaker(150);
        }
    }

    void Stop()
    {
        for (auto pump_it : time_left_on_pumps)
        {
            StopPump(pump_it.first);
        }
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

    void StopPumpsAfterFinish() //TODO: DEBUG this
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        while (amount_of_active_pumps)
        {
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            auto miliseconds_passed = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            for (auto it = time_left_on_pumps.begin(); it != time_left_on_pumps.end(); ++it)
            {
                if (it->second - (static_cast<float>(miliseconds_passed)/1000.0) < 0)
                {
                    StopPump(it->first); // i will call this multiple times. TODO: better way?
                }
            }
            delay(10);
        }
    }

    void StartShaker(int amount_of_belts)
    {
        DropShaker();
        for (int i = 0; i < amount_of_belts; ++i)
        {
            digitalWrite(16, HIGH);
            delay(20);
            digitalWrite(16, LOW);
            delay(20);
        }
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

    void StartPump(int pump_id)
    {
        digitalWrite(pump_id_to_gpio[pump_id], HIGH);
        amount_of_active_pumps++; // This doesn;t work
        // Serial.print(amount_of_active_pumps);
    }

    void StopPump(int pump_id)
    {
        digitalWrite(pump_id_to_gpio[pump_id], LOW);
        amount_of_active_pumps--;
    }

    std::map<int, float> time_left_on_pumps;
    int amount_of_active_pumps = 0;


                                           
    Recipe current_recipe;
    Servo servo_;

    int servo_angle_up = 180;
    int servo_angle_down = 300;

    std::map<int, int> pump_id_to_gpio = { {0, 5},
                                           {1, 4},
                                           {2, 14},
                                           {3, 12} };
};