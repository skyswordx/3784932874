/**
 * @file motion_controller.h
 * @brief 智能车运动控制实现
 *
 * @author lqf
 * @date 2024.3.3
 * @version 0.0
 */

#pragma once

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <smartcar/config.hpp>
#include <smartcar/motion_data.hpp>
#include <smartcar/utils.hpp>
#include <smartcar/crossing_detector.hpp>
#include <smartcar/ring_detector.hpp>
#include <smartcar/crosswalk_detector.hpp>
#include <smartcar/danger_detector.hpp>
#include <smartcar/bridge_detector.hpp>
#include <smartcar/rescue_detector.hpp>

/**
 * @brief 智能车运动控制类
 *
 * 实现巡线逻辑，同时计算车辆运动的相关信息
 */
class MotionController
{
  public:
    /**
     * @brief 初始化函数
     *
     * @param config Config类引用
     *
     * @author lqf
     */
    void init(const Config &config);

    /**
     * @brief 巡线处理函数
     *
     * @param gray_image 待处理的灰度图像
     *
     * @author lqf
     */
    void process(const cv::Mat &gray_image, const std::vector<PredictResult>& ai_results, MotionData &data);

    /**
     * @brief （暂时弃用）将车道线搜索结果输出至图像中
     *
     * @param image 用于获取输出图像宽高信息的输入图像
     * @return 带有车道线标记的图像
     *
     * @author lqf
     */
    cv::Mat display(const cv::Mat &image);

    /**
     * @brief 绘制透视变换后的原始赛道图
     */
    cv::Mat getPerspectiveTransformedImage(const cv::Mat &input_image);

    /**
     * @brief 绘制识别出来的两侧车道线
     */
    void drawBilateralLaneLine(cv::Mat &image);

    /**
     * @brief 绘制识别出来的两侧车道线（透视变换之前）
     */
    void drawBilateralLaneLineBeforePT(cv::Mat &image);

    /**
     * @brief 绘制拟合得到的车道中线 
     */
    void drawMidLaneLine(cv::Mat image);

    /**
     * @brief 绘制角点识别结果 
     */
    void drawCorners(cv::Mat &image);

    /**
     * @brief 绘制赛道状态判断结果 
     */
    void drawTrackState(cv::Mat &image);

    /**
     * @brief 绘制预瞄点
     */
    void drawLookahead(cv::Mat &image);

    /**
     * @brief 绘制标识角点有效区域的横线 
     */
    void drawValidCornerLine(cv::Mat &image);

    /**
     * @brief 绘制垂直中线 
     */
    void drawVerticalLine(cv::Mat &image);

    /**
     * @brief 在控制台输出赛道状态
    */
    void printTrackState();

  private:
    TrackState track_state;     ///< 道路状态

    CrossingDetector crossing; ///< 十字路检测部件
    RingDetector ring; ///< 圆环检测部件
    CrosswalkDetector crosswalk;  ///< 斑马线检测部件
    DangerDetector danger;  ///< 危险区检测部件
    BridgeDetector bridge;  ///< 坡道区检测部件
    RescueDetector rescue;  ///< 救援区检测部件

    // 车道巡线涉及变量
    int direction[4][2]{{0, -1}, {1, 0}, {0, 1}, {-1, 0}}; ///< 方向：上、右、下、左（迷宫巡线法中使用）
    int lane_len;                                          ///< 需要检测的车道线像素点数量
    int min_lane_len;                                      ///< 车道线有效的最少点数
    int lane_sampling_interval;                            ///< 车道线采样间距
    int bias_x;                                            ///< 搜索车道线的起始点x坐标偏移值（x为列）
    int bias_y;                                            ///< 搜索车道线的起始点y坐标偏移值（y为行）
    double upper_limit; ///< 向上搜索车道线起点最多可搜索的长度（相对于图像高度的百分比）
    int block_size;     ///< 自适应阈值二值化计算时用到的正方形采样范围边长
    int binary_bias;    ///< 自适应阈值二值化计算时用到的经验误差值
    int start_confidence; ///< 确定当前点为车道线起点的置信度要求（找到多少个满足要求的点后确定找到了车道线起点）
    int color_delta;      ///< 颜色差阈值
    double bias_yaw_times;     ///< 角度误差修正倍数 
    LaneLine lane_left;                         ///< 左车道线
    LaneLine lane_right;                        ///< 右车道线       
    std::vector<cv::Point> lane_center;  ///< 基于左车道线平移得到的中线像素点的vector数组
    std::vector<int> lane_left_bool;     ///< 记录某个y值上是否已有边线记录点
    std::vector<int> lane_right_bool;
    // std::vector<cv::Point2f> center_points;     ///< 基于左车道线平移得到的中线像素点的vector数组
    std::vector<cv::Point2d> points_to_process; ///< 待处理的像素点
    std::vector<cv::Point2d> track_points;      ///< 赛道轨迹点

    // 透视变换相关变量
    cv::Point2f pt_src_points[4];
    cv::Point2f pt_dst_points[4];
    cv::Mat pt_matrix;

    // 角点计算相关变量
    int valid_corner_y;     ///< 需进行角点判断的最小的y坐标
    double valid_corner_threshold;    ///< 角点有效的最小角度
    double curve_threshold;           ///< 判断车道线是否为曲线的角点角度阈值

    // 连通性方法得到的压缩二值化图像
    cv::Mat track_points_image;

    // 巡线处理计算变量
    double yaw;          ///< 车辆偏角
    double motion_speed; ///< 运行速度（cm/s）
    LaneType lane_type;  ///< 当前巡线的道路类型（直道/弯道）
    bool stop_flag;      ///< 判断小车是否停止的标志信号
    bool bridge_flag;    ///< 判断小车是否进入坡道的标志信号
    bool reverse_flag;   ///< 判断小车是否需要倒车的标志信号
    cv::Point2f lookahead_point;  ///< 预瞄点
    /**
     * @brief 车道线检测
     *
     * 广搜遍历车道
     *
     * @param gray_image 需要进行车道线检测的灰度图像
     */
    void trackDetect(const cv::Mat &gray_image);

    /**
     * @brief 左车道线检测
     *
     * 使用迷宫法搜索左车道线
     *
     * @param gray_iamge 需要进行车道线检测的灰度图像
     */
    void laneLeftDetect(const cv::Mat &gray_image);

    /**
     * @brief 右车道线检测
     *
     * 使用迷宫法搜索右车道线
     * 方法与左车道线的搜索相同，针对右线搜索进行了一些小修改
     * 思路注释详见左车道线搜索函数
     *
     * @param gray_image 需要进行车道线检测的灰度图像
     */
    void laneRightDetect(const cv::Mat &gray_image);

    /**
     * @brief 对车道线执行透视变换，变换至俯视视角
     *
     * 过滤掉变换后坐标中含有负值的点。
     */
    void lanePerspectiveTransform();


    /**
     * @brief 利用透视变换后的车道线求中线
     *
     * @todo 计算中线位置
     */
    void laneCenterCalculate();

    /**
     * @brief 检测车道线角点
     *
     * @param lane 待检测车道线
     *
     * @todo 实现角点计算逻辑
     */
    void laneCornerDetect(LaneLine &lane);

    /**
     * @brief 判断车道线的曲直情况
     */
    void checkLaneCurvature(LaneLine &lane);

    /**
     * @brief 元素识别
     */
    void recognize(const std::vector<PredictResult>& ai_results);

    /**
     * @brief 直接转角控制
     */
    void basicSteeringControl();

    /**
     * @brief 向MotionData写入计算好的运动数据
     *
     * @param data MotionData实例引用
     */
    void writeMotionData(MotionData &data);
    
};
