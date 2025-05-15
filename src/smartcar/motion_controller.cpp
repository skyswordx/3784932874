#include <smartcar/motion_controller.hpp>

void MotionController::init(const Config &config)
{
    // 初始化各个部件
    crosswalk.init();
    danger.init();
    bridge.init();
    crossing.init();
    rescue.init();

    restaurant.init();
    charging.init();
    temp_stop.init();

    // 从Config中读取参数
    try
    {
        lane_len = config.get<int>("motion_lane_len");
        min_lane_len = config.get<int>("motion_min_lane_len");
        lane_sampling_interval = config.get<int>("motion_lane_sampling_interval");
        bias_x = config.get<int>("motion_bias_x");
        bias_y = config.get<int>("motion_bias_y");
        upper_limit = config.get<double>("motion_upper_limit");
        block_size = config.get<int>("motion_block_size");
        binary_bias = config.get<int>("motion_binary_bias");
        start_confidence = config.get<int>("motion_start_confidence");
        color_delta = config.get<int>("motion_color_delta");
        bias_yaw_times = config.get<double>("motion_bias_yaw_times");

        // 读取透视变换参数
        std::vector<std::vector<double>> pt_src_input = config.get<std::vector<std::vector<double>>>("pt_src");
        std::vector<std::vector<double>> pt_dst_input = config.get<std::vector<std::vector<double>>>("pt_dst");
        for (int i = 0; i < 4; i++)
        {
            cv::Point2f src_point(pt_src_input[i][0], pt_src_input[i][1]);
            pt_src_points[i] = src_point;
            cv::Point2f dst_point(pt_dst_input[i][0], pt_dst_input[i][1]);
            pt_dst_points[i] = dst_point;
        }

        // 角点相关参数
        valid_corner_threshold = config.get<double>("motion_valid_corner_threshold");
        valid_corner_y = config.get<int>("motion_valid_corner_y");
        curve_threshold = config.get<double>("motion_curve_threshold");

        lane_left_bool.resize(IMAGE_HEIGHT);
        lane_right_bool.resize(IMAGE_HEIGHT);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Motion] An error occurred while reading config: " << e.what() << std::endl;
        return;
    }

    // 初始化赛道和车道线状态
    stop_flag = false;
    bridge_flag = false;
    reverse_flag = false;
    track_state = TrackState::BEGIN;
    lane_left.type = LaneType::INVALID;
    lane_right.type = LaneType::INVALID;

    // 为车道线数组预先分配内存
    lane_left.points.reserve(lane_len);
    lane_right.points.reserve(lane_len);

    // 初始化赛道轨迹图
    track_points_image = cv::Mat::zeros(28, 32, CV_8UC1);

    // 计算透视变换矩阵
    pt_matrix = cv::getPerspectiveTransform(pt_src_points, pt_dst_points);
    std::cout << "[Motion] Ready!!!" << std::endl;
}

void MotionController::process(const cv::Mat &gray_image, const std::vector<PredictResult>& ai_results, MotionData &data)
{
    // 左右巡线
    laneLeftDetect(gray_image);
    laneRightDetect(gray_image);
    // 透视变换
    lanePerspectiveTransform();
    // 角点检测
    laneCornerDetect(lane_left);
    laneCornerDetect(lane_right);
    // 判断是否为弯道
    checkLaneCurvature(lane_left);
    checkLaneCurvature(lane_right);
    // 元素识别，并更新状态机
    recognize(ai_results);
    // 中线计算
    laneCenterCalculate();
    // 计算转角误差
    basicSteeringControl();
    // 写入向下位机发送的数据
    writeMotionData(data);

    printTrackState();

}

cv::Mat MotionController::display(const cv::Mat &image)
{

    // cv::Mat result = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);

    // // 检查并将巡线结果输出至图像中
    // for (const auto &point : lane_left_H)
    // {
    //     if (point.x >= 0 && point.x < image.cols && point.y >= 0 && point.y < image.rows)
    //     {
    //         result.at<cv::Vec3b>(static_cast<int>(point.y), static_cast<int>(point.x)) = cv::Vec3b(255, 255, 255);
    //     }
    // }
    // for (const auto &point : lane_right_H)
    // {
    //     if (point.x >= 0 && point.x < image.cols && point.y >= 0 && point.y < image.rows)
    //     {
    //         result.at<cv::Vec3b>(static_cast<int>(point.y), static_cast<int>(point.x)) = cv::Vec3b(255, 255, 255);
    //     }
    // }
    // for (const auto &point : lane_center)
    // {
    //     if (point.x >= 0 && point.x < image.cols && point.y >= 0 && point.y < image.rows)
    //     {
    //         result.at<cv::Vec3b>(static_cast<int>(point.y), static_cast<int>(point.x)) = cv::Vec3b(0, 0, 255);
    //     }
    // }

    // std::string text = "";
    // for (int i = 0; i < 2; i++) 
    // {
    //     if (corners[i].valid) 
    //     {
    //         cv::Scalar color;
    //         if (corners[i].angle > 0) color = cv::Scalar(0, 0, 255);
    //         else color = cv::Scalar(0, 255, 0);
    //         cv::circle(result, corners[i].pos, 3, color, -1);
    //         if (i == 0)
    //             text += "Left: " + std::to_string(corners[i].angle) + "  " + std::to_string(corners[i].pos.x) + "  " + std::to_string(corners[i].pos.y) + "\n";
    //         if (i == 1)
    //             text += "Right: " + std::to_string(corners[i].angle)  + "  " + std::to_string(corners[i].pos.x) + "  " + std::to_string(corners[i].pos.y) + "\n";
    //     }
    // }
    // cv::putText(result, text, cv::Point(20, 20), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0));

    // image.convertTo(result, -1, 0.5);

    // // 放到原图右上角
    // // TAA 抗锯齿（X）抗抖动（V）
    // track_points_image *= 0.5;
    // for (const auto &point : track_points)
    // {
    //     if (point.x >= 0 && point.x < image.cols && point.y >= 0 && point.y < image.rows)
    //     {
    //         track_points_image.at<uchar>(static_cast<int>(point.y / 10), static_cast<int>(point.x / 10)) = 255;
    //     }
    // }
    // cv::Mat roi = result(
    //     cv::Rect(result.cols - track_points_image.cols, 0, track_points_image.cols, track_points_image.rows));
    // track_points_image.copyTo(roi);
    // return result;
}

cv::Mat MotionController::getPerspectiveTransformedImage(const cv::Mat &input_image)
{
    cv::Mat output_image;
    cv::warpPerspective(input_image, output_image, pt_matrix, input_image.size());
    return output_image;
}

void MotionController::drawBilateralLaneLine(cv::Mat &image)
{
    cv::Scalar color;
    switch (lane_left.type)
    {
        case LaneType::INVALID:
            color = Color::Gray;
            break;
        case LaneType::STRAIGHT:
            color = Color::Yellow;
            break;
        case LaneType::CURVE:
            color = Color::Orange;
            break;
    }
    for (const auto &point : lane_left.H_points)
    {
        if (point.x >= 0 && point.x < image.cols && point.y >= 0 && point.y < image.rows)
        {
            cv::circle(image, cv::Point(point.x, point.y), 1, color, -1);
        }
    }

    switch (lane_right.type)
    {
        case LaneType::INVALID:
            color = Color::Gray;
            break;
        case LaneType::STRAIGHT:
            color = Color::Yellow;
            break;
        case LaneType::CURVE:
            color = Color::Orange;
            break;
    }
    for (const auto &point : lane_right.H_points)
    {
        if (point.x >= 0 && point.x < image.cols && point.y >= 0 && point.y < image.rows)
        {
            cv::circle(image, cv::Point(point.x, point.y), 1, color, -1);
        }
    }
}

void MotionController::drawBilateralLaneLineBeforePT(cv::Mat &image)
{
    cv::Scalar color;
    switch (lane_left.type)
    {
        case LaneType::INVALID:
            color = Color::Gray;
            break;
        case LaneType::STRAIGHT:
            color = Color::Yellow;
            break;
        case LaneType::CURVE:
            color = Color::Orange;
            break;
    }
    for (const auto &point : lane_left.points)
    {
        if (point.x >= 0 && point.x < image.cols && point.y >= 0 && point.y < image.rows)
        {
            cv::circle(image, cv::Point(point.x, point.y), 1, color, -1);
        }
    }

    switch (lane_right.type)
    {
        case LaneType::INVALID:
            color = Color::Gray;
            break;
        case LaneType::STRAIGHT:
            color = Color::Yellow;
            break;
        case LaneType::CURVE:
            color = Color::Orange;
            break;
    }
    for (const auto &point : lane_right.points)
    {
        if (point.x >= 0 && point.x < image.cols && point.y >= 0 && point.y < image.rows)
        {
            cv::circle(image, cv::Point(point.x, point.y), 1, color, -1);
        }
    }
}

void MotionController::drawMidLaneLine(cv::Mat image)
{
    for (const auto &point : lane_center)
    {
        if (point.x >= 0 && point.x < image.cols && point.y >= 0 && point.y < image.rows)
        {
            cv::circle(image, cv::Point(point.x, point.y), 1, Color::Red, -1);
        }
    }
}

void MotionController::drawCorners(cv::Mat &image)
{
    cv::Scalar color;
    if (lane_left.corner.valid) 
    {
        if (lane_left.corner.angle > 0) color = Color::Red;
        else color = Color::Green;
        cv::circle(image, lane_left.corner.pos, 5, color, -1);
    }
    if (lane_right.corner.valid) 
    {
        if (lane_right.corner.angle > 0) color = Color::Red;
        else color = Color::Green;
        cv::circle(image, lane_right.corner.pos, 5, color, -1);
    }
}

void MotionController::drawTrackState(cv::Mat &image)
{
    std::string text;
    cv::Scalar color;
    switch (track_state)
    {
        case TrackState::BEGIN:
            text = "BEGIN";
            color = Color::Black;
            break;

        case TrackState::PRESTOP:
            text = "PRESTOP";
            color = Color::Black;
            break;
        
        case TrackState::STOP:
            text = "STOP";
            color = Color::Black;
            break;

        case TrackState::NORMAL:
            text = "NORMAL";
            color = Color::Green;
            break;

        case TrackState::DANGER:
            text = "DANGER";
            color = Color::Orange;
            break;
        
        case TrackState::DANGER_LEFT:
            text = "DANGER_LEFT";
            color = Color::Orange;
            break;

        case TrackState::DANGER_RIGHT:
            text = "DANGER_RIGHT";
            color = Color::Orange;
            break;

        case TrackState::CROSSING_IN:
            text = "CROSSING_IN";
            color = Color::Blue;
            break;
        
        case TrackState::CROSSING_OUT:
            text = "CROSSING_OUT";
            color = Color::Blue;
            break;
        
        case TrackState::RING_BEGIN:
            text = "RING_BEGIN";
            color = Color::Red;
        break;

        case TrackState::RING_IN:
            text = "RING_IN";
            color = Color::Red;
            break;

        case TrackState::RING_RUN:
            text = "RING_RUN";
            color = Color::Red;
            break;
        
        case TrackState::RING_OUT:
            text = "RING_OUT";
            color = Color::Red;
            break;

        case TrackState::RING_END:
            text = "RING_END";
            color = Color::Red;
            break;
        
        case TrackState::BRIDGE:
            text = "BRIDGE";
            color = Color::Gray;
            break;
        
        case TrackState::RESCUE_READY:
            text = "RESCUE_READY";
            color = Color::Magenta;
            break;

        case TrackState::RESCUE_TURN:
            text = "RESCUE_TURN";
            color = Color::Magenta;
            break;

        case TrackState::RESCUE_GARAGING:
            text = "RESCUE_GARAGING";
            color = Color::Magenta;
            break;

        case TrackState::RESCUE_STOP:
            text = "RESCUE_STOP";
            color = Color::Magenta;
            break;

        case TrackState::RESCUE_EXITING:
            text = "RESCUE_EXITING";
            color = Color::Magenta;
            break;

        default:
            return;
    }

    cv::Point text_org(5, 17.5);
    int padding = 5;
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.5;
    int thickness = 1;
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);

    cv::Rect background(
        0,
        0,
        text_size.width + 2 * padding,
        text_size.height + 2 * padding + baseline
    );

    cv::rectangle(image, background, color, cv::FILLED);
    cv::putText(image, text, text_org, font_face, font_scale, Color::White, thickness);
}

void MotionController::drawLookahead(cv::Mat &image)
{
    cv::circle(image, lookahead_point, 8, Color::Blue, -1);
}

void MotionController::drawValidCornerLine(cv::Mat &image)
{
    cv::Point start(0, valid_corner_y);
    cv::Point end(image.cols-1, valid_corner_y);
    cv::line(image, start, end, Color::Green, 1);
}

void MotionController::drawVerticalLine(cv::Mat &image)
{
    cv::Point start(IMAGE_WIDTH/2, IMAGE_HEIGHT-1);
    cv::Point end(IMAGE_WIDTH/2, 0);
    cv::line(image, start, end, Color::Green, 1);
}

void MotionController::printTrackState()
{
    std::string text;
    switch (track_state)
    {
        case TrackState::BEGIN:
            text = "BEGIN";
            break;
        case TrackState::PRESTOP:
            text = "PRESTOP";
            break;
        case TrackState::STOP:
            text = "STOP";
            break;
        case TrackState::NORMAL:
            text = "NORMAL";
            break;
        case TrackState::DANGER:
            text = "DANGER";
            break;
        case TrackState::DANGER_LEFT:
            text = "DANGER_LEFT";
            break;
        case TrackState::DANGER_RIGHT:
            text = "DANGER_RIGHT";
            break;
        case TrackState::CROSSING_IN:
            text = "CROSSING_IN";
            break;
        case TrackState::CROSSING_OUT:
            text = "CROSSING_OUT";
            break;
        case TrackState::RING_BEGIN:
            text = "RING_BEGIN";
            break;
        case TrackState::RING_IN:
            text = "RING_IN";
            break;
        case TrackState::RING_RUN:
            text = "RING_RUN";
            break;
        case TrackState::RING_OUT:
            text = "RING_OUT";
            break;
        case TrackState::RING_END:
            text = "RING_END";
            break;
        case TrackState::BRIDGE:
            text = "BRIDGE";
            break;
        default:
            text = "UNKNOWN";
            break;
    }
    std::cout << "[Motion] State: " << text << std::endl;
}

void MotionController::trackDetect(const cv::Mat &gray_image)
{
    points_to_process.clear();
    track_points.clear();

    // 参数变量 (后续考虑使用config读取)

    // 寻找车道线起始点
    // 图像底部中心点的x、y坐标分别减去bias_x，bias_y的结果值作为搜索起点
    int width = gray_image.cols;
    int height = gray_image.rows;
    int start_x = std::max(width / 2 - bias_x, 0);
    int start_y = std::max(height - 1 - bias_y, 0);

    points_to_process.emplace_back(start_x, start_y);
    points_to_process.emplace_back(start_x - 10, start_y);
    points_to_process.emplace_back(start_x - 10, start_y - 10);
    points_to_process.emplace_back(start_x + 10, start_y);
    points_to_process.emplace_back(start_x + 10, start_y - 10);

    cv::Mat visited = cv::Mat::zeros(gray_image.size(), CV_8U);

    std::vector<cv::Point2d> directions = {{0, -5}, {0, 5}, {-5, 0}, {5, 0}};

    while (!points_to_process.empty())
    {
        cv::Point2d current = points_to_process.back();
        points_to_process.pop_back();

        for (const auto &dir : directions)
        {
            cv::Point2d next = current + dir;
            if (next.x < 0 || next.x >= width || next.y < 0 || next.y >= height || visited.at<uchar>(next))
            {
                continue;
            }
            visited.at<uchar>(next) = 1;
            if (calColorDeltaThreshold(gray_image, current, next, color_delta))
            {
                points_to_process.emplace_back(next);
                track_points.emplace_back(next);
            }
        }
    }
}

void MotionController::laneLeftDetect(const cv::Mat &gray_image)
{
    // 清空左车道线vector
    auto &lane_points = lane_left.points;
    lane_points.clear();

    // 参数变量 (后续考虑使用config读取)

    // 寻找车道线起始点
    // 图像底部中心点的x、y坐标分别减去bias_x，bias_y的结果值作为搜索起点
    int width = gray_image.cols;
    int height = gray_image.rows;
    int start_x = std::max(width / 2 - bias_x, 0);
    int start_y = std::max(height - 1 - bias_y, 0);
    if ((track_state == TrackState::BEGIN || track_state == TrackState::PRESTOP) && crosswalk.is_detected)
    {
        // std::cout << "START Y: " << start_y << " CROSSWALK: " << crosswalk.crosswalk_y << std::endl;
        if (start_y <= crosswalk.crosswalk_y + 30 && start_y >= crosswalk.crosswalk_y - 15)
            start_y = crosswalk.crosswalk_y - 20;
    }

    // 从搜索起点开始向左遍历每个点，每遇到一个点使用自适应二值化计算二值化像素值
    // 如果当前遍历到的像素点二值化结果为0，且与先前的点pre二值化像素值不同，说明可能遍历到了车道线的位置
    // 如果检测到指定数量（start_confidence）的二值化结果为0且与先前pre点二值化像素值不同的点，则认为pre记录的点是车道线起点
    // 否则更新pre为最新遍历到的点，继续向左遍历
    uchar start_color;
    bool start_confirmed = false;
    int count = 0;
    int pre_x = start_x;
    int pre_y = start_y;
    uchar pre_color = calAdaptiveThreshold(gray_image, start_x, start_y, block_size, binary_bias);
    for (--start_x; start_x >= 0; --start_x)
    {
        uchar cur_color = calAdaptiveThreshold(gray_image, start_x, start_y, block_size, binary_bias);
        if (cur_color == 0 && cur_color != pre_color)
        {
            count++;
            if (count == start_confidence)
            {
                start_x = pre_x;
                start_y = pre_y;
                start_color = pre_color;
                start_confirmed = true;
                break;
            }
        }
        else
        {
            pre_x = start_x;
            pre_y = start_y;
            pre_color = cur_color;
        }
    }
    // 遍历整行后都没有找到起点就退出函数，本次巡线结果为丢线
    // if (!start_confirmed)
    //     return;
    // 如果没有找到起点，向上遍历
    if (!start_confirmed)
    {
        int limit_y = static_cast<int>(upper_limit * height);
        start_x = 0;
        for (--start_y; start_y >= limit_y; --start_y)
        {
            uchar cur_color = calAdaptiveThreshold(gray_image, start_x, start_y, block_size, binary_bias);
            if (cur_color == 0 && cur_color != pre_color)
            {
                count++;
                if (count == start_confidence)
                {
                    start_x = pre_x;
                    start_y = pre_y;
                    start_color = pre_color;
                    start_confirmed = true;
                    break;
                }
            }
            else
            {
                pre_x = start_x;
                pre_y = start_y;
                pre_color = cur_color;
            }
            if (start_confirmed)
            {
                break;
            }
        }
    }

    // 如果遍历完整个图像后仍然没有找到起点，退出函数，本次巡线结果为丢线
    if (!start_confirmed)
    {
        return;
    }

    // 找到车道线起点后开始左巡线
    // 使用迷宫法寻找lane_len个标记车道线边缘的像素点
    // 迷宫法介绍参见巡线思路说明文档
    count = 0;
    int rotate_times = 0;
    int front_dir = 0;
    int left_dir = 3;
    int min_y = height;
    while (count < lane_len)
    {
        int next_x = start_x + direction[front_dir][0];
        int next_y = start_y + direction[front_dir][1];
        if (next_x < 0 || next_y < 0 || next_x >= width || next_y >= height)
            return;
        uchar next_color = calAdaptiveThreshold(gray_image, next_x, next_y, block_size, binary_bias);
        if (next_color == start_color)
        {
            int next_left_x = next_x + direction[left_dir][0];
            int next_left_y = next_y + direction[left_dir][1];
            if (next_left_x < 0 || next_left_y < 0 || next_left_x >= width || next_left_y >= height)
                return;
            uchar next_left_color =
                calAdaptiveThreshold(gray_image, next_left_x, next_left_y, block_size, binary_bias);
            if (next_left_color == start_color)
            {
                start_x = next_left_x;
                start_y = next_left_y;
                front_dir = left_dir;
                left_dir = (left_dir - 1) % 4;
                if (left_dir < 0)
                    left_dir += 4;
            }
            else
            {
                start_x = next_x;
                start_y = next_y;
            }
            // if (start_y <= min_y + 2)
            // {
                count++;
                if (count % lane_sampling_interval == 0) lane_points.emplace_back(start_x, start_y);
                rotate_times = 0;
                // if (start_y <= min_y) min_y = start_y;
                // min_y = start_y;
            // }
            // else
            //     break;
        }
        else
        {
            front_dir = (front_dir + 1) % 4;
            left_dir = (left_dir + 1) % 4;
            rotate_times++;
            if (rotate_times > 2)
                break;
        }
    }

    // 车道线点数目小于最低数量要求时本次巡线无效
    // if ((count / lane_sampling_interval) < min_lane_len)
    // {
    //     lane_left.points.clear();
    // }
}

void MotionController::laneRightDetect(const cv::Mat &gray_image)
{
    auto &lane_points = lane_right.points;
    lane_points.clear();

    // 寻找初始点
    int width = gray_image.cols;
    int height = gray_image.rows;
    int start_x = std::max(width / 2 + bias_x, 0);
    int start_y = std::max(height - 1 - bias_y, 0);
    if ((track_state == TrackState::BEGIN || track_state == TrackState::PRESTOP) && crosswalk.is_detected)
    {
        if (start_y <= crosswalk.crosswalk_y + 30 && start_y >= crosswalk.crosswalk_y - 15)
            start_y = crosswalk.crosswalk_y - 20;
    }

    uchar start_color;
    bool start_confirmed = false;
    int count = 0;
    int pre_x = start_x;
    int pre_y = start_y;
    uchar pre_color = calAdaptiveThreshold(gray_image, start_x, start_y, block_size, binary_bias);
    for (++start_x; start_x < width; ++start_x)
    {
        uchar cur_color = calAdaptiveThreshold(gray_image, start_x, start_y, block_size, binary_bias);
        if (cur_color == 0 && cur_color != pre_color)
        {
            count++;
            if (count == start_confidence)
            {
                start_x = pre_x;
                start_y = pre_y;
                start_color = pre_color;
                start_confirmed = true;
                break;
            }
        }
        else
        {
            pre_x = start_x;
            pre_y = start_y;
            pre_color = cur_color;
        }
    }
    // if (!start_confirmed)
    //     return;
    // 如果没有找到起点，向上遍历
    if (!start_confirmed)
    {
        int limit_y = static_cast<int>(upper_limit * height);
        start_x = 0;
        start_x = width - 1;
        for (--start_y; start_y >= limit_y; --start_y)
        {
            uchar cur_color = calAdaptiveThreshold(gray_image, start_x, start_y, block_size, binary_bias);
            if (cur_color == 0 && cur_color != pre_color)
            {
                count++;
                if (count == start_confidence)
                {
                    start_x = pre_x;
                    start_y = pre_y;
                    start_color = pre_color;
                    start_confirmed = true;
                    break;
                }
            }
            else
            {
                pre_x = start_x;
                pre_y = start_y;
                pre_color = cur_color;
            }
            if (start_confirmed)
            {
                break;
            }
        }
    }

    if (!start_confirmed)
    {
        return;
    }

    // 开始右巡线
    count = 0;
    int rotate_times = 0;
    int front_dir = 0;
    int right_dir = 1;
    int min_y = height;
    while (count < lane_len)
    {
        int next_x = start_x + direction[front_dir][0];
        int next_y = start_y + direction[front_dir][1];
        if (next_x < 0 || next_y < 0 || next_x >= width || next_y >= height)
            return;
        uchar next_color = calAdaptiveThreshold(gray_image, next_x, next_y, block_size, binary_bias);
        if (next_color == start_color)
        {
            int next_right_x = next_x + direction[right_dir][0];
            int next_right_y = next_y + direction[right_dir][1];
            if (next_right_x < 0 || next_right_y < 0 || next_right_x >= width || next_right_y >= height)
                return;
            uchar next_right_color =
                calAdaptiveThreshold(gray_image, next_right_x, next_right_y, block_size, binary_bias);
            if (next_right_color == start_color)
            {
                start_x = next_right_x;
                start_y = next_right_y;
                front_dir = right_dir;
                right_dir = (right_dir + 1) % 4;
            }
            else
            {
                start_x = next_x;
                start_y = next_y;
            }
            // if (start_y <= min_y + 2)
            // {
                count++;
                if (count % lane_sampling_interval == 0) lane_points.emplace_back(start_x, start_y);
                rotate_times = 0;
                // if (start_y <= min_y) min_y = start_y;
                // min_y = start_y;
            // }
            // else
            //     break;
        }
        else
        {
            front_dir = (front_dir - 1) % 4;
            if (front_dir < 0)
                front_dir += 4;
            right_dir = (right_dir - 1) % 4;
            if (right_dir < 0)
                right_dir += 4;
            rotate_times++;
            if (rotate_times > 2)
                break;
        }
    }

    // 车道线点数目小于最低数量要求时本次巡线无效
    // if ((count / lane_sampling_interval) < min_lane_len) 
    // {
    //     lane_right.points.clear();
    // }
}

void MotionController::lanePerspectiveTransform()
{
    std::vector<cv::Point2f> temp_left_H;
    std::vector<cv::Point2f> temp_right_H;
    lane_left.H_points.clear();
    lane_right.H_points.clear();
    // std::fill(lane_left_bool.begin(), lane_left_bool.end(), 0);
    // std::fill(lane_right_bool.begin(), lane_right_bool.end(), 0);
    lane_left.type = LaneType::INVALID;
    lane_right.type = LaneType::INVALID;

    // 对左侧车道线应用透视变换
    if (!lane_left.points.empty()) {
        cv::perspectiveTransform(lane_left.points, temp_left_H, pt_matrix);
        // 过滤负坐标点
        for (const auto& point : temp_left_H) {
            int x = static_cast<int>(point.x);
            int y = static_cast<int>(point.y);
            if (x >= 0 && y >= 0) {
                lane_left.H_points.push_back(cv::Point(x, y));
                // lane_left_bool[y] = 1;
            }
        }

        // 判断点数是否满足要求
        if (lane_left.H_points.size() >= min_lane_len)
            lane_left.type = LaneType::STRAIGHT;
    }

    // 对右侧车道线应用透视变换
    if (!lane_right.points.empty()) {
        cv::perspectiveTransform(lane_right.points, temp_right_H, pt_matrix);
        // 过滤负坐标点
        for (const auto& point : temp_right_H) {
            int x = static_cast<int>(point.x);
            int y = static_cast<int>(point.y);
            if (x >= 0 && y >= 0) {
                lane_right.H_points.push_back(cv::Point(x, y));
                // lane_right_bool[y] = 1;
            }
        }

        // 判断点数是否满足要求
        if (lane_right.H_points.size() >= min_lane_len)
            lane_right.type = LaneType::STRAIGHT;
    }

}

void MotionController::laneCenterCalculate()
{
    lane_center.clear();

    cv::Point2f diff;        // 相邻点的向量差
    cv::Point2f normal;      // 单位法向量
    cv::Point2f center_point;// 中点
    int step_length = 5;     // 步长
    float diff_distance = 48.0; // 沿法线偏移量

    LaneLine* selected_lane = nullptr;
    static bool is_lane_left = false; // 标记选中的是否为左侧车道线

    // 选择逻辑
    // DANGER_LEFT: 选择右边线计算中线
    if (track_state == TrackState::DANGER_LEFT)
    {
        is_lane_left = false;
        selected_lane = &lane_right;
    }
    // DANGER_RIGHT: 选择左边线计算中线
    else if (track_state == TrackState::DANGER_RIGHT)
    {
        is_lane_left = true;
        selected_lane = &lane_left;
    }
    // 默认情况: 
    else
    {
        if (lane_left.type != LaneType::INVALID && lane_right.type != LaneType::INVALID)
        {
            if (is_lane_left)
            {
                if (lane_right.H_points.size() >= lane_left.H_points.size() + 20)
                {
                    is_lane_left = false;
                    selected_lane = &lane_right;
                }
                else
                {
                    selected_lane = &lane_left;
                }
            }
            else
            {
                if (lane_left.H_points.size() >= lane_right.H_points.size() + 20)
                {
                    is_lane_left = true;
                    selected_lane = &lane_left;
                }
                else
                {
                    selected_lane = &lane_right;
                }
            }
        }
        else if (lane_left.type != LaneType::INVALID)
        {
            is_lane_left = true;
            selected_lane = &lane_left;
        } 
        else if (lane_right.type != LaneType::INVALID)
        {
            is_lane_left = false;
            selected_lane = &lane_right;
        } 
    }

    // 确定道路类型（直道/弯道）
    if (selected_lane)
        lane_type = selected_lane->type;

    // 确定用于计算车道中线的车道线范围(start->end)以及偏移距离(diff_distance)
    if (selected_lane && selected_lane->H_points.size() > step_length) {
        int start, end;
        if (track_state == TrackState::NORMAL || track_state == TrackState::BEGIN || track_state == TrackState::PRESTOP || track_state == TrackState::DANGER
            || track_state == TrackState::RESCUE_READY)
        {
            start = 0;
            end = selected_lane->H_points.size() - step_length;
        }
        else if (track_state == TrackState::DANGER_LEFT || track_state == TrackState::DANGER_RIGHT)
        {
            start = 0;
            end = selected_lane->H_points.size() - step_length;
            diff_distance = 35;
        }
        else if (track_state == TrackState::RESCUE_TURN || track_state == TrackState::RESCUE_GARAGING || track_state == TrackState::RESCUE_EXITING)
        {
            step_length = 1;
            start = 0;
            end = selected_lane->H_points.size() - step_length;
            diff_distance = 25;
        }
        else if (track_state == TrackState::BRIDGE)
        {
            start = 0;
            end = 30;
        }
        else if (track_state == TrackState::CROSSING_IN)
        {
            if (selected_lane->corner.valid)
            {
                start = 0;
                end = selected_lane->corner.idx - 5;
            }
            else
                return;
        }
        else if (track_state == TrackState::CROSSING_OUT)
        {
            if (selected_lane->corner.valid)
            {
                start = selected_lane->corner.idx;
                end = selected_lane->H_points.size() - step_length;
            }
            else
                return;
        }
        else if (track_state == TrackState::RING_BEGIN)
        {
            start = 0;
            end = selected_lane->H_points.size() - step_length;
        }
        else if (track_state == TrackState::RING_IN)
        {
            start = 0;
            end = selected_lane->H_points.size() * 2 / 3;
        }
        else if (track_state == TrackState::RING_RUN)
        {
            start = 0;
            end = selected_lane->H_points.size() - step_length;
        }
        else if (track_state == TrackState::RING_OUT)
        {
            start = 0;
            end = selected_lane->H_points.size() - step_length;
        }
        else if (track_state == TrackState::RING_END)
        {
            start = 0;
            end = selected_lane->H_points.size() - step_length;
        }
        else 
            return;

        // 计算车道中线
        int min_y = IMAGE_HEIGHT;
        for (int i = start; i < end; ++i) {
            diff.x = selected_lane->H_points[i].x - selected_lane->H_points[i + step_length].x;
            diff.y = selected_lane->H_points[i].y - selected_lane->H_points[i + step_length].y;
            float norm = std::sqrt(diff.x * diff.x + diff.y * diff.y);
            
            if (is_lane_left) {
                normal.x = diff.y / norm;
                normal.y = -diff.x / norm;
            } else {
                normal.x = -diff.y / norm;
                normal.y = diff.x / norm;
            }

            center_point.x = (selected_lane->H_points[i].x + selected_lane->H_points[i + step_length].x) / 2.0;
            center_point.y = (selected_lane->H_points[i].y + selected_lane->H_points[i + step_length].y) / 2.0;
            center_point.x += diff_distance * normal.x;
            center_point.y += diff_distance * normal.y;
            if (center_point.y > min_y) continue;
            else min_y = center_point.y;
            lane_center.emplace_back(center_point);
        }

        // std::cout << lane_center.size() << std::endl;
    }
}

void MotionController::laneCornerDetect(LaneLine &lane)
{
    lane.corner.valid = false;
    
    // 进行角点检测
    int step = 10;
    double max_angle = 1.0;
    int max_angle_direction = 0;
    cv::Point2f max_corner;
    int max_corner_idx;

    for (int i = 0; i < lane.H_points.size(); i++) 
    {
        if (i - step < 0 || i + step >= lane.H_points.size()) continue;
        // 如果点的y坐标小于阈值就不考虑
        if (lane.H_points[i].y <= valid_corner_y) continue; 

        cv::Point2f v1(lane.H_points[i].x - lane.H_points[i - step].x, lane.H_points[i].y - lane.H_points[i - step].y);
        cv::Point2f v2(lane.H_points[i + step].x - lane.H_points[i].x, lane.H_points[i + step].y - lane.H_points[i].y);
        double angle = dotProduct(v1, v2) / (norm(v1) * norm(v2));
        int direction = (crossProduct(v1, v2) > 0) ? 1 : -1;

        if (angle < max_angle)  
        {
            max_angle = angle;
            max_angle_direction = direction;
            max_corner = lane.H_points[i];
            max_corner_idx = i;
        }
    }

    max_angle = acos(max_angle) * (180.0 / M_PI);
    if (max_angle > valid_corner_threshold) 
    {
        lane.corner.valid = true;
    }
    lane.corner.pos = max_corner;
    lane.corner.angle = max_angle_direction * max_angle;
    lane.corner.idx = max_corner_idx;
}

void MotionController::checkLaneCurvature(LaneLine &lane)
{
    if (track_state == TrackState::BRIDGE)
    {
        lane.type = LaneType::STRAIGHT;
        return;
    }
        
    if (lane.type != LaneType::INVALID) 
    {
        // 待检测车道线最大角度大于阈值时认为这是弯道
        if (fabs(lane.corner.angle) >= curve_threshold)
        {
            lane.type = LaneType::CURVE;
        }
    }
}

/**
 * @brief 元素识别
 */
void MotionController::recognize(const std::vector<PredictResult>& ai_results)
{
    // ai元素识别结果处理
    crosswalk.detect(ai_results, track_state);
    danger.detect(ai_results, lane_left, lane_right, track_state);
    bridge.detect(ai_results, track_state);
    // rescue.detect(ai_results, lane_left, lane_right, track_state);
    crossing.detect(lane_left, lane_right, track_state);
    //ring.detect(lane_left, lane_right, track_state);

    restaurant.detect(ai_results, track_state);
    charging.detect(ai_results, track_state);
    temp_stop.detect(ai_results, track_state);
}

void MotionController::basicSteeringControl()
{
    stop_flag = false;
    bridge_flag = false;
    reverse_flag = false;

    // 初始状态下保持直线行驶
    // if (track_state == TrackState::BEGIN)
    // {
    //     yaw = 0;
    //     return;
    // }

    // 坡道状态下不打角
    // if (track_state == TrackState::BRIDGE)
    // {
    //     // yaw = 0;
    //     bridge_flag = true;
    //     // return;
    // }

    // 结束状态下发送停车标志
    if (track_state == TrackState::STOP || track_state == TrackState::RESCUE_STOP || track_state == TrackState::RESTAURANT_STOPPED || track_state == TrackState::CHARGING_CHARGING || track_state == TrackState::TEMP_STOP_STOPPED)
    {
        // std::cout << "STOP!!!\n";
        stop_flag = true;
        return;
    }

    if (track_state == TrackState::RESCUE_EXITING || track_state == TrackState::CHARGING_REVERSING)
    {
        reverse_flag = true;
    }

    int lookahead_idx;
    if (!lane_center.empty())
    {
        lookahead_idx = (lane_center.size() > 20) ? 20 : (lane_center.size() - 1);
    }
    else
    {
        return;
    }
    int counter = 0;
    double look_ahead_x = 0;
    double look_ahead_y = 0;
    for(int i = 0; i < lane_center.size(); i++){
        if(lane_center[i].y >= 95 && lane_center[i].y <= 155){
            look_ahead_x += lane_center[i].x;
            look_ahead_y += lane_center[i].y;
            counter ++;
        }
    }
    if (counter == 0) 
    {
        return;
    }
    else
    {
        look_ahead_x = look_ahead_x / counter;
        look_ahead_y = look_ahead_y / counter;
    }
    cv::Point2f bottom_point(160.0, 180.0);
    lookahead_point = cv::Point2f(look_ahead_x, look_ahead_y);
    yaw = bias_yaw_times * atan2(lookahead_point.x - bottom_point.x, bottom_point.y - lookahead_point.y);
}

void MotionController::writeMotionData(MotionData &data)
{
    data.bridge_flag = false;
    data.danger_flag = false;
    data.is_curve = false;
    data.stop_flag = false;
    data.reverse_flag = false;
    
    data.yaw = yaw;
    data.motion_speed = motion_speed;
    data.stop_flag = stop_flag;
    data.bridge_flag = bridge_flag;
    data.reverse_flag = reverse_flag;
    data.is_curve = (lane_type == LaneType::CURVE);
    if (stop_flag) std::cout << "Write Stop Flag...\n";
    if (track_state == TrackState::DANGER || track_state == TrackState::DANGER_LEFT || track_state == TrackState::DANGER_RIGHT) data.danger_flag = true;
}
