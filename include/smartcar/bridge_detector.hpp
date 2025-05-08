/**
 * @file bridge_detector.hpp
 * @brief 坡道识别类实现
 */

#pragma once

#include <smartcar/config.hpp>
#include <smartcar/utils.hpp>

/**
 * @brief 坡道识别类
 */
class BridgeDetector
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
     * @brief 判断是否进入坡道区域
     *
     * @param ai_results ai模型识别结果
     * @param track_state 当前道路状态
     */
    void detect(const std::vector<PredictResult>& ai_results, TrackState& track_state);

  private:
    int enter_counter;
    int quit_counter;
    int enter_threshold;
    int quit_threshold;
};
