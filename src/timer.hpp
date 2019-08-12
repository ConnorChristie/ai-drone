#pragma once

#include <time.h>
#include <chrono>

typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;

struct CallStat
{
    typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;

    CallStat():
        numberOfCalls(0), totalDuration(0.0), lastCallDuration(0.0), smoothedDuration(-1.0) {
    }

    void setCallDuration(ms value)
    {
        lastCallDuration = value.count();
        numberOfCalls++;
        totalDuration += lastCallDuration;
        if (smoothedDuration < 0)
        {
            smoothedDuration = lastCallDuration;
        }
        double alpha = 0.1;
        smoothedDuration = smoothedDuration * (1.0 - alpha) + lastCallDuration * alpha;
    }

    void start()
    {
        lastCallStart = std::chrono::high_resolution_clock::now();
    }

    void finish()
    {
        auto t = std::chrono::high_resolution_clock::now();
        setCallDuration(std::chrono::duration_cast<ms>(t - lastCallStart));
    }

    double avarageTotalDuration()
    {
        return totalDuration / numberOfCalls;
    }

    size_t numberOfCalls;
    double totalDuration;
    double lastCallDuration;
    double smoothedDuration;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastCallStart;
};