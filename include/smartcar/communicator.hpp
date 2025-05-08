#ifdef ENABLE_SERIAL_MODULE
/**
 * @file communicator.h
 * @brief 单片机通信类实现
 *
 * 使用libSerial库实现通信
 */

#pragma once

#ifdef __linux__
#include <SerialStream.h>
#endif
#include <smartcar/config.hpp>
#include <smartcar/motion_data.hpp>

/**
 * @brief 存储类结构体
 */
struct DataPacket {
    char parity_0;
    char parity_1;
    char parity_2;
    char parity_3;
    float yaw;
    float speed;
    char  tag;
    char ring_info;
    char stop_flag;
    char parity_15;
};

/**
 * @brief 单片机通信类
 */
class Communicator
{
public:
    float yaw_str;
    float motion_speed_str;
    char stop_flag_str;
    char * sent_data;
#ifdef __linux__
    LibSerial::SerialStream serial;
#endif

    /**
     * @brief 初始化函数
     *
     * @param config类引用
     */
    void init(const Config &config);

    /**
     * @brief 发送数据 
     */
    void send_bytes(MotionData &data);

private:
    std::vector<char> tag_buffer;
    int buffer_size = 10; 
    std::unordered_map<char, int> tag_count;

    /**
     * @brief 更新tag/info的缓冲区 
     */
    void update_buffer(std::vector<char>& buffer, char value, std::unordered_map<char, int>& count);

    /**
     * @brief 获取缓冲区中出现最多的字符 
     */
    char get_most_frequent(const std::unordered_map<char, int>& count);
};
#endif