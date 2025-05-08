/**
 * @file utils.hpp
 * @brief 公共函数实现
 *
 * 各个部件用到的工具函数在这里实现
 *
 * @author lqf
 * @date 2024.3.3
 * @version 0.0
 */

#pragma once

#include <opencv2/opencv.hpp>

extern int IMAGE_HEIGHT;
extern int IMAGE_WIDTH;

/**
 * @brief 预定义颜色常量
*/
namespace Color {
    const cv::Scalar Red(0, 0, 255);       // 红色
    const cv::Scalar Green(0, 255, 0);     // 绿色
    const cv::Scalar Blue(255, 0, 0);      // 蓝色
    const cv::Scalar Black(0, 0, 0);       // 黑色
    const cv::Scalar White(255, 255, 255); // 白色
    const cv::Scalar Yellow(0, 255, 255);  // 黄色
    const cv::Scalar Cyan(255, 255, 0);    // 青色
    const cv::Scalar Magenta(255, 0, 255); // 品红色
    const cv::Scalar Gray(128, 128, 128);  // 灰色
    const cv::Scalar Orange(0, 165, 255);   // 橙色
}

/**
 * @brief AI检测结果类型
*/
struct PredictResult
{
    int type;          ///< ID
    std::string label; ///< 标签
    float score;       ///< 置信度
    int x;             ///< 坐标
    int y;             ///< 坐标
    int width;         ///< 尺寸
    int height;        ///< 尺寸
};

/**
 * @brief 巡线所需的初始数据结构体
*/
struct InputData
{
    cv::Mat input_image;    ///< 待巡线处理图像
    std::vector<PredictResult> input_ai_result;    ///< ai检测结果
};

/**
 * @brief 车道线类型
 */
enum class LaneType
{
    INVALID,
    STRAIGHT,
    CURVE
};

/**
 * @brief 赛道状态
*/
enum class TrackState
{
    BEGIN,         ///< 小车起跑阶段
    PRESTOP,       ///< 小车预备停车阶段
    STOP,          ///< 小车停车阶段
    NORMAL,        ///< 正常行驶阶段
    CROSSING_IN,   ///< 十字路进入阶段
    CROSSING_OUT,  ///< 十字路驶出阶段
    RING_BEGIN,
    RING_IN,
    RING_RUN,
    RING_OUT,
    RING_END,
    BRIDGE,        ///< 坡道行驶阶段
    DANGER,        ///< 危险区路段
    DANGER_LEFT,   ///< 检测到障碍物在道路左侧
    DANGER_RIGHT,  ///< 检测到障碍物在道路右侧
    RESCUE_READY,  ///< 救援区准备阶段
    RESCUE_TURN,   ///< 车辆转向进入救援区
    RESCUE_GARAGING,  ///< 车辆在救援区内沿锥桶行驶
    RESCUE_STOP,   ///< 救援区内停车
    RESCUE_EXITING, ///< 车辆退出救援区
};

/**
 * @brief 角点数据类型
*/
struct Corner
{
    bool valid;         ///< 角点是否有效
    cv::Point2f pos;    ///< 角点坐标
    double angle;       ///< 角点偏角
    int idx;            ///< 角点在透视变换后车道线数组中的下标
};

/**
 * @brief 车道线数据类型 
 */
struct LaneLine
{
    std::vector<cv::Point2f> points;
    std::vector<cv::Point> H_points;
    LaneType type;
    Corner corner;
};

/**
 * @brief 自适应阈值二值化计算
 *
 * 计算某一个像素点的二值化结果
 * 阈值计算方式为以某一点为中心的正方形范围内所有点像素值的均值
 *
 * @param gray_image 待二值化的灰度图像
 * @param x 待二值化的像素点x坐标（x为列）
 * @param y 待二值化的像素点y坐标（y为行）
 * @param block_size 某一点二值化计算时正方形采样范围的边长（奇数）
 * @param bias 经验误差值
 * @return 返回某个点的二值化计算结果
 */
uchar calAdaptiveThreshold(const cv::Mat &gray_image, int x, int y, int block_size, int bias);

/**
 * @brief 连通图方法中使用的颜色阈值 
 */
bool calColorDeltaThreshold(const cv::Mat &gray_image, const cv::Point2f &start, const cv::Point2f &end, int delta);

/**
 * @brief 向量内积
 * @param v1 参与内积的第一个向量
 * @param v2 参与内积的第二个向量
 */
double dotProduct(const cv::Point2f& v1, const cv::Point2f& v2);

/**
 * @brief 向量外积
 * @param v1 参与外积的第一个向量
 * @param v2 参与外积的第二个向量
 */
double crossProduct(const cv::Point2f& v1, const cv::Point2f& v2);

/**
 * @brief 向量求模
 * @param v 待求模向量
 */
double norm(const cv::Point2f& v);

/**
 * @brief 阶乘计算
 *
 * @param x
 * @return x!
 */
int factorial(int x);

/**
 * @brief 贝塞尔曲线
 *
 * @param dt
 * @param 生成贝塞尔曲线的控制点
 * @return 贝塞尔曲线对应的数组
 */
std::vector<cv::Point> bezier(double dt, const std::vector<cv::Point>& input);