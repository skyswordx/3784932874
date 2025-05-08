/**
 * @file ring_detector.hpp
 * @brief 环路识别类实现
 */

#pragma once

#include <smartcar/config.hpp>
#include <smartcar/utils.hpp>

/**
 * @brief 环路识别类
 */
class RingDetector
{
  public:
    /**
     * @brief 初始化函数
     *
     * 从Config类中读取相关参数
     *
     * @param config Config配置信息读取类引用
     *
     * @todo 编写初始化函数，从config中读取参数存储到成员变量中
     */
    void init();

    /**
     * @brief 判断是否进入环路
     *
     * @param motion 车辆运动控制器引用，用于读取处理后的车道线数据
     * @return 是否进入环路
     *
     * @todo 编写环路检测逻辑
     */
    void detect(LaneLine& lane_left, LaneLine& lane_right, TrackState& track_state);

  private:
    int counter;
    int counter_run; 
};
