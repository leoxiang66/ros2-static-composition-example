#include <cstdint>
#ifndef LTIMER_H
#define LTIMER_H

#include <cstdlib>
#include <chrono>
#include <iostream>
#include <limits>
#include <thread>
#include <sstream>
#include <iomanip>

class LTimer
{
public:
    static const int Ksec2micro = 1000000;

    // 构造函数
    LTimer(uint64_t sync_point = 10 * Ksec2micro);

    // 获取当前时间的微秒时间戳
    uint64_t getCurrentTimeUs() const;

    // 同步到第一个时间间隔
    void syncToFirstInterval();

    // 将纳秒时间戳转换为日期字符串
    std::string nanosec2date(uint64_t nanoseconds);

private:
    uint64_t sync_point_; // 同步点
};

// 构造函数定义
LTimer::LTimer(uint64_t sync_point) : sync_point_(sync_point) {}

// 获取当前时间的微秒时间戳
uint64_t LTimer::getCurrentTimeUs() const
{
    auto now = std::chrono::high_resolution_clock::now();
    auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
    return now_us;
}

// 同步到第一个时间间隔
void LTimer::syncToFirstInterval()
{
    uint64_t current_time_us = getCurrentTimeUs();
    uint64_t sleep_time_us = sync_point_ - (current_time_us % sync_point_);
    uint64_t next_interval_us = current_time_us + sleep_time_us;

    std::cout << "Wait until " << nanosec2date(next_interval_us * 1000) << std::endl;

    // 睡眠
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_us) - std::chrono::microseconds(800));
}

// 将纳秒时间戳转换为日期字符串
std::string LTimer::nanosec2date(uint64_t nanoseconds)
{
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::nanoseconds(nanoseconds));
    auto time_point = std::chrono::system_clock::time_point(seconds);
    std::time_t time_t_value = std::chrono::system_clock::to_time_t(time_point);

    char buffer[100];
    if (std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", std::localtime(&time_t_value)))
    {
        int64_t remaining_nanos = nanoseconds % 1000000000;
        std::ostringstream oss;
        oss << buffer << "." << std::setw(9) << std::setfill('0') << remaining_nanos;
        return oss.str();
    }
    return "";
}

#endif // LTIMER_H