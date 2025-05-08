/**
 * @file motion_data.h
 * @brief 存储待发送给单片机的运动控制数据
 *
 * @author lqf
 * @date 2024.3.22
 */

#pragma once

#include <smartcar/config.hpp>

/**
 * @brief 车辆运动数据存储类
 */
class MotionData
{
public:
    double yaw;          ///< 车辆偏角（弧度值）
    double motion_speed; ///< 运行速度（cm/s）
    bool is_curve;     ///< 是否为弯道的标志
    bool stop_flag;      ///< 判断小车是否停止的标志信号
    bool bridge_flag;    ///< 小车是否进入坡道的标志信号
    bool danger_flag;    ///< 小车是否进入危险区的标志信号
    bool reverse_flag;   ///< 判断小车是否需要倒车的信号

    /**
     * @brief 初始化函数
     *
     * @param config Config类引用
     */
    void init(const Config &config);
};
