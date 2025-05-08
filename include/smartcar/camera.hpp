/**
 * @file camera.hpp
 * @brief 图像读取及预处理类
 *
 * @author lqf
 * @date 2024.4.24
 */
#pragma once

#include <smartcar/config.hpp>
#include <smartcar/utils.hpp>

#include <opencv2/opencv.hpp>
#include <iostream>

/**
 * @brief 图像预处理类 
 * 
 * 包含图像处理的各种函数（图像去畸等）
 */
class Camera
{
public:
    ~Camera();

    /**
     * @brief 初始化函数 
     */
    void init(const Config& config);

    /**
     * @brief 从videocapture中获取图像
    */
    bool read(cv::Mat& image);

    /**
     * @brief 使用OpenCV函数获取灰度图像 
     */
    cv::Mat convertToGray(const cv::Mat& image);

    /**
     * @brief 图像畸变去除 
     * 
     * @param image 待去畸图像
     */
    cv::Mat correct(const cv::Mat& image);

    /**
     * @brief 图像预处理
     */
    void preprocess(cv::Mat& image);

private:
    std::string video_src;    ///< 视频源路径
    int video_width;          ///< 读入视频图像宽度
    int video_height;         ///< 读入视频图像高度
    int resize_width;         ///< 缩放后图像宽度
    int resize_height;        ///< 缩放后图像高度
    double alpha;             ///< 图像对比度增益
    double beta;              ///< 图像亮度增益
    bool min_gray_enable;            ///< 使用三通道中像素值最小值进行灰度转换方法启用使能
    bool correct_enable;      ///< 图像去畸启用使能
    std::string calibration_file_path;  ///< 标定文件路径

    cv::Mat cameraMatrix;   ///< 相机内参矩阵
    cv::Mat distCoeffs;     ///< 相机畸变矩阵
    cv::VideoCapture cap;   ///< OpenCV视频读取类
};