/**
 * @file danger_detector.hpp
 * @brief 危险区识别类实现
 */

#pragma once

#include <smartcar/config.hpp>
#include <smartcar/utils.hpp>

/**
 * @brief 斑马线识别类
 */
class DangerDetector
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
     * @brief 判断是否进入危险区区域
     *
     * @param ai_results ai模型识别结果
     * @param lane_left 左车道线数据
     * @param lane_right 右车道线数据
     * @param track_state 当前道路状态
     */
    void detect(const std::vector<PredictResult>& ai_results, const LaneLine& lane_left, const LaneLine& lane_right, TrackState& track_state);

  private:
    int enter_counter;
    int left_counter;
    int right_counter;
    int quit_counter;
    int enter_threshold;
    int left_threshold;
    int right_threshold;
    int quit_threshold;

    /**
     * @brief 计算车道线上点x值的平均值
     */
    int calculateAverageX(const LaneLine& lane);
};
