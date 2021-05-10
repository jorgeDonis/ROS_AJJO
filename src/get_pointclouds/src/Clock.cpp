#include "Clock.hpp"

void Clock::tik()
{
    t_0 = std::chrono::high_resolution_clock::now();
}

void Clock::tok()
{
    t_1 = std::chrono::high_resolution_clock::now();
}

double Clock::seconds_spent() const
{
    return std::chrono::duration<double, std::milli>(t_1 - t_0).count() / 1e3;
}