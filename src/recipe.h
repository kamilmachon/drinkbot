#pragma once

#include <vector>

class Recipe
{
    public:
        void SetPumps(const std::vector<float>& amouts)
        {
            for(auto a : amouts)
            {
                amouts_.push_back(MililitersToSeconds(a));
            }
        }   

        std::vector<float> GetPumpsTime()
        {
            return amouts_;
        }

        void SetShakeTime(float time)
        {
            use_shaker = true;
            shake_time = time;
        }

        float GetShakeTime()
        {
            return shake_time;
        }

    private:

    float MililitersToSeconds(float time)
    {
        return time*ml_to_time;
    }

    bool use_shaker = false;
    float shake_time;

    float ml_to_time = 1;

    std::vector<float> amouts_;
};