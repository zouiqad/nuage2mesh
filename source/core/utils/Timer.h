#ifndef TIMER_H
#define TIMER_H

#include "patterns/events/TimerEndEvent.h"
#include "patterns/singleton/EventDispatcher.h"
#include <chrono>
#include <string>
#include <iostream>

namespace n2m {
    class Timer {
    public:
        Timer(const std::string &taskName) : taskName(taskName),
                                             startTime(
                                                 std::chrono::high_resolution_clock::now()) {
        }

        ~Timer() {
            float duration = this->stop();
            std::cout << taskName << "finished executing," << " elapsed time: " <<
                    duration << "ms" << std::endl;

            // send event
            TimerEndEvent timer_end_event(duration);
            EventDispatcher::Instance().publish(timer_end_event);
        }

        void start() {
            startTime = std::chrono::high_resolution_clock::now();
        }

        float stop() {
            auto endTime = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                endTime - startTime).count();
            return duration / 1000.0f; // Converts to float (1ms = 1000mu)
        }

    private:
        std::string taskName;
        std::chrono::high_resolution_clock::time_point startTime;
    };
}

#endif

