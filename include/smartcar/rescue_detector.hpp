/**
 * @file rescue_detector.hpp
 * @brief 救援区识别类实现
 */

#pragma once

#include <smartcar/config.hpp>
#include <smartcar/utils.hpp>
#include <stack>

/**
 * @brief 识别类
 */
class RescueDetector
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
     * @brief 救援区处理
     *
     * @param ai_results ai模型识别结果
     * @param lane_left 左车道线数据
     * @param lane_right 右车道线数据
     * @param track_state 当前道路状态
     */
    void detect(const std::vector<PredictResult>& ai_results, LaneLine& lane_left, LaneLine& lane_right, TrackState& track_state);

  private:
    int enter_left_counter;
    int enter_right_counter;
    int quit_counter;
    int exit_counter;
    int enter_threshold;
    int quit_threshold;
    bool is_left;                       ///< 左转/右转标志
    int garaging_threshold;             ///< 车辆进行入库转向的锥桶阈值
    std::stack<LaneLine> stack_left;    ///< 存储左车道线的栈
    std::stack<LaneLine> stack_right;   ///< 存储右车道线的栈
    std::vector<cv::Point> cones_left;  ///< 左侧锥桶位置数组
    std::vector<cv::Point> cones_right; ///< 右侧锥桶位置数组
    std::vector<cv::Point> cone_points;  ///< GARAGING阶段用于巡线的锥桶点

    /**
     * @brief 获取锥桶位置信息 
     */
    void searchCones(const std::vector<PredictResult>& ai_results);

    /**
     * @brief 根据x值对锥桶排序（从左到右，从小到大）
     */
    void sortConesByX(std::vector<cv::Point>& cone_points);

    /**
     * @brief 搜索最高（y值最小）的锥桶 
     */
    cv::Point searchHeighestCone();
};

