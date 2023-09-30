#ifndef TIME_STAMP_HPP
#define TIME_STAMP_HPP
#include <chrono>
#include <mutex>
#include <vector>
#include <sys/time.h>
struct timeval sub_timeval(struct timeval tv1, struct timeval tv2)
{
    struct timeval res;

    res.tv_sec = tv1.tv_sec - tv2.tv_sec;
    res.tv_usec = tv1.tv_usec - tv2.tv_usec;

    return res;
}

struct timeval add_timeval(struct timeval tv1, struct timeval tv2)
{
    struct timeval res;

    res.tv_sec = tv1.tv_sec + tv2.tv_sec;
    res.tv_usec = tv1.tv_usec + tv2.tv_usec;

    return res;
}
struct timeval div_timeval(struct timeval tv, double divisor)
{
    struct timeval res;

    res.tv_sec = tv.tv_sec / divisor;
    res.tv_usec = tv.tv_usec / divisor;

    return res;
}
struct timeval mul_timeval(struct timeval tv, double multiplier)
{
    struct timeval res;

    res.tv_sec = tv.tv_sec * multiplier;
    res.tv_usec = tv.tv_usec * multiplier;

    return res;
}
struct timeval mod_timeval(struct timeval tv, int mod)
{
    struct timeval res;

    res.tv_sec = tv.tv_sec % mod;
    res.tv_usec = tv.tv_usec % mod;

    return res;
}

class my_time
{
private:
public:
    struct timeval time_val;
    unsigned long long time_ms;
    unsigned long long time_us;
    my_time();
    my_time(timeval &tv)
    {
        time_val = tv;
        time_ms = tv.tv_sec * 1000 + tv.tv_usec / 1000;
        time_us = tv.tv_sec * 1000000 + tv.tv_usec;
    }
    my_time(const my_time &mt)
    {
        this->time_ms = mt.time_ms;
        this->time_us = mt.time_us;
        this->time_val = mt.time_val;
    }
    void ms_init(unsigned long long ms)
    {
        this->time_ms = ms;
        this->time_us = ms * 1000;
        this->time_val.tv_sec = ms / 1000;
        this->time_val.tv_usec = ms % 1000;
    }
    void us_init(unsigned long long us)
    {
        this->time_us = us;
        this->time_ms = us / 1000;
        this->time_val.tv_sec = us / 1000000;
        this->time_val.tv_usec = us % 1000000;
    }
    my_time operator+(const my_time &mt)
    {
        my_time res;
        res.time_ms = this->time_ms + mt.time_ms;
        res.time_us = this->time_us + mt.time_us;
        res.time_val = add_timeval(this->time_val, mt.time_val);
        return res;
    }
    my_time operator-(const my_time &mt)
    {
        my_time res;
        res.time_ms = this->time_ms - mt.time_ms;
        res.time_us = this->time_us - mt.time_us;
        res.time_val = sub_timeval(this->time_val, mt.time_val);
        return res;
    }
    my_time operator/(double divisor)
    {
        my_time res;
        res.time_ms = this->time_ms / divisor;
        res.time_us = this->time_us / divisor;
        res.time_val = div_timeval(this->time_val, divisor);
        return res;
    }
    my_time operator*(double multiplier)
    {
        my_time res;
        res.time_ms = this->time_ms * multiplier;
        res.time_us = this->time_us * multiplier;
        res.time_val = mul_timeval(this->time_val, multiplier);
        return res;
    }
    my_time operator%(int mod)
    {
        my_time res;
        res.time_ms = this->time_ms % mod;
        res.time_us = this->time_us % mod;
        res.time_val = mod_timeval(this->time_val, mod);
        return res;
    }

    my_time operator=(const my_time &mt)
    {
        this->time_ms = mt.time_ms;
        this->time_us = mt.time_us;
        this->time_val = mt.time_val;
        return *this;
    }
    void update()
    {
        gettimeofday(&(this->time_val), NULL);
        this->time_ms = time_val.tv_sec * 1000 + time_val.tv_usec / 1000;
        this->time_us = time_val.tv_sec * 1000000 + time_val.tv_usec;
    }
    ~my_time();
};

my_time::my_time(/* args */)
{
    this->update();
}

my_time::~my_time()
{
}

class time_stamp
{
private:
    my_time mcu_time;
    my_time now_time;
    my_time time_diff;
    std::vector<my_time> time_diff_his;
    std::mutex time_mutex;
    // 平滑参数
    float k = 2.0;
    int n = 0;

public:
    time_stamp(/* args */);
    time_stamp(time_stamp &ts)
    {
        this->time_diff = ts.time_diff;
        this->mcu_time = ts.mcu_time;
        this->now_time = ts.now_time;
    }
    ~time_stamp();
    my_time GetTimeStamp()
    {
        std::lock_guard<std::mutex> lock(time_mutex);
        gettimeofday(&(this->now_time.time_val), NULL);
        my_time tep = this->now_time + this->time_diff;
        return tep;
    }
    my_time GetTimeStamp(my_time &mt)
    {
        std::lock_guard<std::mutex> lock(time_mutex);
        // printf("%ld\t%ld\n", now_time.time_ms, time_diff.time_ms);
        my_time tep = mt - this->time_diff;
        return tep;
    }
    // sigmoid 函数
    float sigmoid(int n, float k)
    {
        float w = 1.0 / (1 + exp(-k * n));
        return w;
    }

    void UpdateTimeDiff(my_time mcu_time)
    {
        std::lock_guard<std::mutex> lock(time_mutex);
        this->mcu_time = mcu_time;
        this->now_time.update();
        my_time tep = this->now_time - this->mcu_time;
        while (time_diff_his.size() > 14)
            time_diff_his.erase(time_diff_his.begin(), time_diff_his.begin() + 1);
        for (int i = 0; i < time_diff_his.size(); i++)
        {
            this->time_diff = tep * this->sigmoid(i, this->k) + this->time_diff * (1 - this->sigmoid(i, this->k));
        }
        time_diff_his.push_back(this->time_diff);
    }
    my_time GetTimeDiff()
    {
        return time_diff;
    }
};

time_stamp::time_stamp(/* args */)
{
    mcu_time.update();
    now_time.update();
    time_diff = now_time - mcu_time;
    time_diff_his.resize(100);
    time_diff_his.push_back(time_diff);
}

time_stamp::~time_stamp()
{
}

#endif