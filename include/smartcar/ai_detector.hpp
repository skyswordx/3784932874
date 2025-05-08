#ifdef ENABLE_AI_MODULE
/**
 * @file ai_detector.h
 * @brief ai模型识别类实现
 */

#pragma once

#include <smartcar/utils.hpp>
#include <onnxruntime_cxx_api.h>
#include <predictor_api.h>
#include <opencv2/opencv.hpp>
#include <json.hpp>


/**
 * @brief ai模型识别类
 */
class AIDetector
{
public:
    // std::vector<PredictResult> results; // AI推理结果
    float score = 0.5;                  // AI检测置信度

    /**
     * @brief 初始化函数
     *
     * @param config Config类引用
     */
    void init();

    std::vector<PredictResult> inference(cv::Mat img);

    void buildNms(const std::string &model_dir);

    void transposeAndCopyToTensor(const cv::Mat &src, NDTensor &dst);

    std::shared_ptr<std::unordered_map<std::string, NDTensor>> preprocess(
        cv::Mat frame,
        const std::vector<int64_t> &input_size);

    void run(const std::unordered_map<std::string, NDTensor> &feeds);

    NDTensor get_output(int index);

    std::vector<PredictResult> render();

    /*
    * @brief 绘制AI模型识别结果
    */
    void drawBox(cv::Mat &img, std::vector<PredictResult>& results);

    /**
     * @brief 获取Opencv颜色
     *
     * @param index 序号
     * @return cv::Scalar
     */
    cv::Scalar getCvcolor(int index);

private:
    std::vector<std::string> labels;
    // onnx info
    std::pair<std::vector<std::string>, std::vector<const char *>> onnx_input_names_;
    std::pair<std::vector<std::string>, std::vector<const char *>> onnx_out_names_;
    Ort::Env onnx_env_;
    // predictor
    std::shared_ptr<PPNCPredictor> predictor_nna_;
    std::shared_ptr<PPNCPredictor> predictor_nms_;
    std::shared_ptr<Ort::Session> predictor_onnx_;

};
#endif
