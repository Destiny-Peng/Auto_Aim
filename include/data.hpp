#ifndef DATA_HPP_
#define DATA_HPP_

#include <cstdint> // uint8_t等类型
#include <math.h>
#include "serial.hpp"
#include "time_stamp.hpp"

struct Send_Flag_Pack
{
    bool target_found = false;
    bool fire = false;
    bool burstShoot = false;
    bool exitRune = false;
    bool rune_fire = false;
    bool rune_burst_shoot = false;
    bool isProgramWorkingProperly = true;
    uint8_t mode = 0x01;
    uint16_t pitch_resolution = 1000;
    uint16_t yaw_resolution = 1000;
};

struct Send_Target_Data_Pack
{
    uint8_t send_digit;
    int16_t pred_pitch;
    int16_t pred_yaw;
    uint8_t pitch_palstance = 0;
    uint8_t yaw_palstance = 0;
};

class pose_pack
{
private:
public:
    my_time pack_time;
    double ptz_pitch = 0, ptz_yaw = 0, ptz_roll = 0;
    pose_pack(/* args */);
    pose_pack(double pitch, double yaw, double roll, const my_time &pack_time)
    {
        this->pack_time = pack_time;
        this->ptz_pitch = pitch;
        this->ptz_yaw = yaw;
        this->ptz_roll = roll;
    }
    ~pose_pack();
    pose_pack operator-(const pose_pack &pack)
    {
        pose_pack result;
        result.pack_time = this->pack_time;
        result.ptz_pitch = this->ptz_pitch - pack.ptz_pitch;
        result.ptz_yaw = this->ptz_yaw - pack.ptz_yaw;
        result.ptz_roll = this->ptz_roll - pack.ptz_roll;
        return result;
    }
    pose_pack operator+(const pose_pack &pack)
    {
        pose_pack result;
        result.pack_time = this->pack_time;
        result.ptz_pitch = this->ptz_pitch + pack.ptz_pitch;
        result.ptz_yaw = this->ptz_yaw + pack.ptz_yaw;
        result.ptz_roll = this->ptz_roll + pack.ptz_roll;
        return result;
    }
    pose_pack operator/(const double divisor)
    {
        pose_pack result;
        result.pack_time = this->pack_time;
        result.ptz_pitch = this->ptz_pitch / divisor;
        result.ptz_yaw = this->ptz_yaw / divisor;
        result.ptz_roll = this->ptz_roll / divisor;
        return result;
    }
    pose_pack operator*(const double multiplier)
    {
        pose_pack result;
        result.pack_time = this->pack_time;
        result.ptz_pitch = this->ptz_pitch * multiplier;
        result.ptz_yaw = this->ptz_yaw * multiplier;
        result.ptz_roll = this->ptz_roll * multiplier;
        return result;
    }
};

pose_pack::pose_pack(/* args */)
{
    pack_time.update();
    this->ptz_pitch = 0;
    this->ptz_yaw = 0;
    this->ptz_roll = 0;
}

pose_pack::~pose_pack()
{
}

class my_data
{
private:
    std::mutex data_mtx;

public:
    my_data(){};
    ~my_data(){};
    my_time pose_mcu_time;
    my_time base_mcu_time;
    uint8_t mode = 0;
    double bullet_speed = 0;
    bool isRightMouseButtonPressing = false;
    bool fired = false;
    bool request_fire = false;
    pose_pack tep;
    Send_Flag_Pack sendFlagPack;
    Send_Target_Data_Pack sendTargetDataPack;
    ThreadSafeDeque<pose_pack> pose_pack_queue;
    ThreadSafeDeque<SerialData> ReceiveQueue;
    ThreadSafeDeque<SerialData> SendQueue;
    time_stamp ts;
    int read_pack()
    {
        std::lock_guard<std::mutex> lock(data_mtx);
        if (!this->ReceiveQueue.empty())
        {
            SerialData pack = this->ReceiveQueue.front();
            this->ReceiveQueue.pop_front();
            // 处理data
            // 存储真实数据的数组
            uint8_t real_data[MAX_LEN];
            // printf("%d\t%d\t%d\n",pack.len,pack.pack_type,ReceiveQueue.size());

            // 遍历原始数据,过滤重复的0xFC并存入real_data
            int real_len = 0;
            // printf("%d\n",pack.len);
            for (int i = 0; i < pack.len; i++)
            {
                // printf("%x\t",pack.storage[i]);
                if (pack.storage[i] == 0xFC && pack.storage[i + 1] == 0xFC)
                {
                    // 跳过重复的0xFC
                    real_data[real_len++] = pack.storage[i];
                    i++;
                }
                else
                {
                    real_data[real_len++] = pack.storage[i];
                }
            }
            // printf("\n");
            // 在real_data上计算校验和
            uint16_t sum = 0;
            for (int i = 0; i < real_len - 1; i++)
            {
                // printf("%x\t",real_data[i]);
                sum += real_data[i];
            }
            // printf("\n");

            uint8_t right_sum = real_data[real_len - 1];
            sum = sum & 0xff;
            // printf("%d\t %d\n", sum, right_sum);
            if (sum == right_sum)
            {
                if (pack.pack_type == 1)
                {
                    // pose_pack
                    //  获取云台pitch轴高八位低八位
                    uint8_t ptz_pitch_high8 = real_data[0];
                    uint8_t ptz_pitch_low8 = real_data[1];
                    double ptz_pitch = (static_cast<int16_t>((static_cast<uint16_t>(ptz_pitch_high8) << 8) | ptz_pitch_low8)) / 180.0;

                    // 获取云台yaw轴高八位低八位
                    uint8_t ptz_yaw_high8 = real_data[2];
                    uint8_t ptz_yaw_low8 = real_data[3];
                    double ptz_yaw = (static_cast<int16_t>((static_cast<uint16_t>(ptz_yaw_high8) << 8) | ptz_yaw_low8)) / 180.0;
                    // 获取云台roll轴高八位低八位
                    uint8_t ptz_roll_high8 = real_data[4];
                    uint8_t ptz_roll_low8 = real_data[5];
                    double ptz_roll = (static_cast<int16_t>((static_cast<uint16_t>(ptz_roll_high8) << 8) | ptz_roll_low8)) / 180.0;
                    // 获取当前姿态电控时刻
                    this->pose_mcu_time.ms_init(real_data[6] * 5);
                    my_time mcu_time = this->base_mcu_time + this->pose_mcu_time % 100;
                    // printf("%ld\t%ld\t%ld\n",mcu_time.time_ms,this->ts.GetTimeDiff().time_ms,this->ts.GetTimeStamp().time_ms);
                    // printf("%ld\t", mcu_time.time_ms);

                    // 取姿态包的十位和个位，取对时包的其他位
                    // 维护队列长度
                    pose_pack_queue.updata_size(); // bug here

                    pose_pack_queue.push_front(pose_pack(ptz_pitch, ptz_yaw, ptz_roll, mcu_time));
                    return 1;
                }
                else if (pack.pack_type == 0)
                {
                    // 获取电控基准时刻的低中高八位
                    uint8_t base_mcu_time_low8 = real_data[0];
                    uint8_t base_mcu_time_mid8 = real_data[1];
                    uint8_t base_mcu_time_high8 = real_data[2];
                    this->base_mcu_time.ms_init(((static_cast<uint32_t>(base_mcu_time_high8) << 16) | (static_cast<uint32_t>(base_mcu_time_mid8) << 8) | base_mcu_time_low8) * 100);
                    // printf("%ld\t%ld\t%ld\n",this->base_mcu_time.time_ms,this->ts.GetTimeDiff().time_ms,this->ts.GetTimeStamp().time_ms);
                    ts.UpdateTimeDiff(this->base_mcu_time);
                    // 获取自瞄模式的高四位
                    this->mode = real_data[3] & 0x0f;
                    // printf("%d\n",this->mode);
                    // 获取子弹速度的高四位低八位
                    uint8_t bullet_speed_high4 = real_data[3] & 0xf0;
                    uint8_t bullet_speed_low8 = real_data[4];
                    this->bullet_speed = ((static_cast<uint16_t>(bullet_speed_high4) << 4) | bullet_speed_low8) / 100.0;

                    // 获取鼠标右键标志位
                    this->isRightMouseButtonPressing = real_data[5] & 0x01;
                    // 获取开火标志位
                    this->fired = (real_data[5] & 0x40) >> 6;
                    this->request_fire = real_data[5] & 0x80 >> 7;
                    return 1;
                }
            }
            else
                return 0;
        }
    }
    int write_pack(pose_pack &pose)
    {
        std::lock_guard<std::mutex> lock(data_mtx);
        uint8_t origin[MAX_LEN] = {0};
        uint8_t sum = 0;
        // 先标志位包，再目标数据包

        uint8_t flags = this->sendFlagPack.isProgramWorkingProperly;
        flags = flags << 1 | this->sendFlagPack.rune_burst_shoot;
        flags = flags << 1 | this->sendFlagPack.rune_fire;
        flags = flags << 1 | this->sendFlagPack.exitRune;
        flags = flags << 1 | this->sendFlagPack.burstShoot;
        flags = flags << 1 | this->sendFlagPack.fire;
        flags = flags << 1 | this->sendFlagPack.target_found;

        origin[0] = flags;
        origin[1] = this->sendFlagPack.mode;
        origin[2] = this->sendFlagPack.pitch_resolution >> 8;
        origin[3] = this->sendFlagPack.pitch_resolution & 0xff;
        origin[4] = this->sendFlagPack.yaw_resolution >> 8;
        origin[5] = this->sendFlagPack.yaw_resolution & 0xff;
        sum = (origin[0] + origin[1] + origin[2] + origin[3] + origin[4] + origin[5]) & 0xff;
        origin[6] = sum;
        uint8_t len = 7;

        this->sendTargetDataPack.pred_yaw = pose.ptz_yaw * 180.0;
        this->sendTargetDataPack.pred_pitch = pose.ptz_pitch * 180.0;
        // printf("%lf\t%lf\n", pose.ptz_yaw,pose.ptz_pitch);

        my_time mt;
        mt = this->ts.GetTimeStamp(mt);
        this->sendTargetDataPack.send_digit = mt.time_ms % 1000 / 10;
        // printf("%d\t%d\t%d\n", this->sendTargetDataPack.pred_yaw, this->sendTargetDataPack.pred_pitch, this->sendTargetDataPack.send_digit);
        uint8_t type = 2;
        if (this->sendFlagPack.target_found)
        {
            origin[7] = this->sendTargetDataPack.send_digit;
            origin[8] = this->sendTargetDataPack.pred_pitch >> 8;
            origin[9] = this->sendTargetDataPack.pred_pitch & 0xff;
            origin[10] = this->sendTargetDataPack.pred_yaw >> 8;
            origin[11] = this->sendTargetDataPack.pred_yaw & 0xff;
            origin[12] = this->sendTargetDataPack.pitch_palstance & 0xff;
            origin[13] = this->sendTargetDataPack.yaw_palstance & 0xff;
            sum = 0;
            sum = (origin[7] + origin[8] + origin[9] + origin[10] + origin[11] + origin[12] + origin[13]) & 0xff;
            origin[14] = sum;
            len = 15;
            type = 3;
        }
        // printf("%ld\n", mt.time_ms);
        SerialData pack((uint8_t *)origin, len, type, mt);
        this->SendQueue.push_front(pack);
        this->sendFlagPack.target_found = 0;
        return 1;
    }
    pose_pack get_info(my_time mt)
    {
        std::lock_guard<std::mutex> lock(data_mtx);
        pose_pack pack1, pack2;
        pack1 = pose_pack_queue.front();
        // printf("pack1:%lf\n", pack1.ptz_pitch);
        // printf("%ld\n",pack1.pack_time.time_ms);
        pack2 = pose_pack_queue.second();
        // printf("pack2:%lf\n", pack2.ptz_pitch);
        // printf("%ld\n",pack2.pack_time.time_ms);
        pose_pack pack = pack1 - pack2;
        // printf("%ld\t%ld\t%ld\n", mt.time_ms, pack2.pack_time.time_ms, mt.time_ms - pack2.pack_time.time_ms);
        // printf("pack/5:%lf\n", (pack / 5).ptz_pitch);
        double ms1 = mt.time_ms;
        double ms2 = pack2.pack_time.time_ms;
        // printf("%lf\t%lf\t%lf\n",ms1,ms2,ms1-ms2);
        // printf("%lf\n", ((pack / 5) * (ms1-ms2)).ptz_pitch);

        pack = (pack / 5) * (ms1-ms2) + pack2;
        // printf("pack:%lf\n", pack.ptz_pitch);

        pack.pack_time = mt;
        return pack;
    }
};

#endif