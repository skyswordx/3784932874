/**
 * @file crossing_detector.h
 * @brief 十字路识别类实现
 */

#pragma once

#include <smartcar/config.hpp>
#include <smartcar/utils.hpp>

/**
 * @brief 十字路识别类
 */
class CrossingDetector
{
public:
    /**
     * @brief 初始化函数
     *
     * @param config Config类引用
     *
     * @todo 编写初始化函数，从Config类中读取配置信息，存入成员变量中
     */
    void init();

    /**
     * @brief 判断是否进入十字路
     *
     * @todo 编写检测函数
     */
    void detect(LaneLine& lane_left, LaneLine& lane_right, TrackState& track_state);

private:
    int counter;
    bool crossing_left_out;
    bool crossing_right_out;
    int last_left_corner_y;
    int last_right_corner_y;
};
