#pragma once

#include <chrono>

class Clock
{
    private:
        std::chrono::_V2::system_clock::time_point t_0;
        std::chrono::_V2::system_clock::time_point t_1;
    public:
        void tik();
        void tok();
        double seconds_spent() const;
};