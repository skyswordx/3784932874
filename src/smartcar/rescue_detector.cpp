#include <smartcar/rescue_detector.hpp>

void RescueDetector::init()
{
    enter_left_counter = 0;
    enter_right_counter = 0;
    quit_counter = 0;
    exit_counter = 0;
    enter_threshold = 3;
    quit_threshold = 3;
    garaging_threshold = 120;
    is_left = false;
}

void RescueDetector::detect(const std::vector<PredictResult>& ai_results, LaneLine& lane_left, LaneLine& lane_right, TrackState& track_state)
{
    switch(track_state)
    {
        // 检测到平民伤员标志或者危险人物标志后进入READY状态
        case(TrackState::NORMAL):
        {
            bool is_left_detected = false;
            bool is_right_detected = false;
            for (auto& item : ai_results)
            {
                if (item.label == "tumble" || item.label == "patient")
                {
                    enter_left_counter++;
                    is_left_detected = true;
                    break;
                }
                else if (item.label == "thief" || item.label == "evil")
                {
                    enter_right_counter++;
                    is_right_detected = true;
                    break;
                }
            }
            if (!is_left_detected) enter_left_counter = (enter_left_counter > 0) ? (enter_left_counter-1) : 0;
            if (!is_right_detected) enter_right_counter = (enter_right_counter > 0) ? (enter_right_counter-1) : 0;

            // 满足阈值条件后进入READY状态
            if (enter_left_counter >= enter_threshold)
            {
                track_state = TrackState::RESCUE_READY;
                is_left = true;
                enter_left_counter = 0;
                enter_right_counter = 0;
            }
            else if (enter_right_counter >= enter_threshold)
            {
                track_state = TrackState::RESCUE_READY;
                is_left = false;
                enter_left_counter = 0;
                enter_right_counter = 0;
            }

            break;
        }

        // 处于READY状态时，当锥桶与车辆之间的距离满足某个阈值后，开始左/右转向
        case(TrackState::RESCUE_READY):
        {
            // 超时保护
            for (auto& item : ai_results)
            {
                if (item.label == "tumble" || item.label == "patient" || item.label == "thief" || item.label == "evil")
                {
                    exit_counter = 0;
                    break;
                }
            }
            exit_counter++;
            if (exit_counter > 30)
            {
                exit_counter = 0;
                track_state = TrackState::NORMAL;
                break;
            }

            searchCones(ai_results);

            // 查找锥桶中y坐标最大值
            int max_y = 0;
            if (is_left)
            {
                for (auto& cone : cones_left)
                {
                    if (cone.y > max_y) max_y = cone.y;
                }
                if (max_y > garaging_threshold) quit_counter++;

                // 满足阈值条件后进行左/右转
                if (quit_counter >= 2)
                {
                    track_state = TrackState::RESCUE_TURN;
                    quit_counter = 0;
                }
            } 
            else
            {
                for (auto& cone : cones_right)
                {
                    if (cone.y > max_y) max_y = cone.y;
                }
                if (max_y > garaging_threshold) quit_counter++;

                // 满足阈值条件后进行左/右转
                if (quit_counter >= 2)
                {
                    track_state = TrackState::RESCUE_TURN;
                    quit_counter = 0;
                }
            }
            
            break;
        }

        // 车辆入库1阶段：左/右固定角转向
        case(TrackState::RESCUE_TURN):
        {
            if (is_left)
            {
                cv::Point start(IMAGE_WIDTH/2, IMAGE_HEIGHT-10);
                cv::Point end(0, 70);
                cv::Point mid(0.6*(start.x+end.x), 0.4*(start.y+end.y));
                std::vector<cv::Point> input = {start, mid, end};
                LaneLine new_lane;
                new_lane.H_points = bezier(0.05, input);
                new_lane.type = LaneType::CURVE;
                lane_right = new_lane;
                stack_right.push(new_lane);
                lane_left.H_points.clear();
                lane_left.points.clear();
                lane_left.type = LaneType::INVALID;
            }
            else
            {
                cv::Point start(IMAGE_WIDTH/2, IMAGE_HEIGHT-10);
                cv::Point end(IMAGE_WIDTH-1, 70);
                cv::Point mid(0.6*(start.x+end.x), 0.4*(start.y+end.y));
                std::vector<cv::Point> input = {start, mid, end};
                LaneLine new_lane;
                new_lane.H_points = bezier(0.05, input);
                new_lane.type = LaneType::CURVE;
                lane_left = new_lane;
                stack_left.push(new_lane);
                lane_right.H_points.clear();
                lane_right.points.clear();
                lane_right.type = LaneType::INVALID;
            }

            // 左转：画面中最高的锥桶（y值最小）位于画面右侧（x>IMAGE_WIDTH/2）时进入下一阶段
            // 右转：画面中最高的锥桶（y值最小）位于画面左侧（x<IMAGE_WIDTH/2）时进入下一阶段
            quit_counter++;
            if (quit_counter > 15)
            {
                searchCones(ai_results);
                if (is_left)
                {
                    cv::Point heighest_cone = searchHeighestCone();
                    if (heighest_cone.x > IMAGE_WIDTH/2)
                    {
                        quit_counter = 0;
                        track_state = TrackState::RESCUE_GARAGING;
                    }
                }
                else
                {
                    cv::Point heighest_cone = searchHeighestCone();
                    if (heighest_cone.x < IMAGE_WIDTH/2)
                    {
                        quit_counter = 0;
                        track_state = TrackState::RESCUE_GARAGING;
                    }
                }
            }

            // 如果已满足停车条件提前停车
            // 检查锥桶的平均y值，满足某个阈值后进入STOP状态
            int average_y = 0;
            for (auto& cone : cones_left)
                average_y += cone.y;
            for (auto& cone : cones_right)
                average_y += cone.y;
            if (cones_left.size() + cones_right.size() > 0)
                average_y /= cones_left.size() + cones_right.size();
            else
                average_y = IMAGE_HEIGHT-1;
            if (average_y >= 80)
                track_state = TrackState::RESCUE_STOP;

            break;
        }

        // 根据当前锥桶位置进行微调
        case(TrackState::RESCUE_GARAGING):
        {
            cone_points.clear();
            searchCones(ai_results);
            cv::Point heighest_cone = searchHeighestCone();
            if (is_left)
            {
                for (auto& cone : cones_left)
                {
                    if (cone.x >= heighest_cone.x)
                        cone_points.push_back(cone);
                }
                for (auto& cone : cones_right)
                {
                    if (cone.x >= heighest_cone.x)
                        cone_points.push_back(cone);
                }
            }
            else
            {
                for (auto& cone : cones_left)
                {
                    if (cone.x <= heighest_cone.x)
                        cone_points.push_back(cone);
                }
                for (auto& cone : cones_right)
                {
                    if (cone.x <= heighest_cone.x)
                        cone_points.push_back(cone);
                }
            }

            if (cone_points.size() >= 3)
            {
                sortConesByX(cone_points);
                std::vector<cv::Point> input = {cone_points[0], cone_points[cone_points.size()/2], cone_points[cone_points.size()-1]};
                LaneLine new_lane;
                new_lane.H_points = bezier(0.05, input);
                new_lane.type = LaneType::CURVE;
                if (is_left)
                {
                    lane_right = new_lane;
                    stack_right.push(new_lane);
                    lane_left.H_points.clear();
                    lane_left.points.clear();
                    lane_left.type = LaneType::INVALID;
                }
                else
                {
                    lane_left = new_lane;
                    stack_left.push(new_lane);
                    lane_right.H_points.clear();
                    lane_right.points.clear();
                    lane_right.type = LaneType::INVALID;
                }
            }
            else if (cone_points.size() >= 2)
            {
                sortConesByX(cone_points);
                cv::Point mid((cone_points[0].x+cone_points[cone_points.size()-1].x)/2, (cone_points[0].y+cone_points[cone_points.size()-1].y)/2);
                std::vector<cv::Point> input = {cone_points[0], mid, cone_points[cone_points.size()-1]};
                LaneLine new_lane;
                new_lane.H_points = bezier(0.05, input);
                new_lane.type = LaneType::CURVE;
                if (is_left)
                {
                    lane_right = new_lane;
                    stack_right.push(new_lane);
                    lane_left.H_points.clear();
                    lane_left.points.clear();
                    lane_left.type = LaneType::INVALID;
                }
                else
                {
                    lane_left = new_lane;
                    stack_left.push(new_lane);
                    lane_right.H_points.clear();
                    lane_right.points.clear();
                    lane_right.type = LaneType::INVALID;
                }
            }
            else
                track_state = TrackState::RESCUE_STOP;

            // 检查锥桶的平均y值，满足某个阈值后进入STOP状态
            int average_y = 0;
            for (auto& cone : cones_left)
                average_y += cone.y;
            for (auto& cone : cones_right)
                average_y += cone.y;
            if (cones_left.size() + cones_right.size() > 0)
                average_y /= cones_left.size() + cones_right.size();
            else
                average_y = IMAGE_HEIGHT-1;
            if (average_y >= 80)
                track_state = TrackState::RESCUE_STOP;
                
            break;
        }

        case(TrackState::RESCUE_STOP):
        {
            quit_counter++;
            if (quit_counter >= 40)
            {
                track_state = TrackState::RESCUE_EXITING;
                quit_counter = 0;
            }
            break;
        }

        case(TrackState::RESCUE_EXITING):
        {
            if (is_left)
            {
                if (!stack_right.empty())
                {
                    LaneLine new_lane = stack_right.top();
                    lane_right = new_lane;
                    stack_right.pop();
                    lane_left.H_points.clear();
                    lane_left.points.clear();
                    lane_left.type = LaneType::INVALID;
                }
                else
                    track_state = TrackState::NORMAL;
            }
            else
            {
                if (!stack_left.empty())
                {
                    LaneLine new_lane = stack_left.top();
                    lane_left = new_lane;
                    stack_left.pop();
                    lane_right.H_points.clear();
                    lane_right.points.clear();
                    lane_right.type = LaneType::INVALID;
                }
                else
                    track_state = TrackState::NORMAL;
            }
            break;
        }

        default:
            break;
    }
}

void RescueDetector::searchCones(const std::vector<PredictResult>& ai_results)
{
    cones_left.clear();
    cones_right.clear();
    for (auto& item : ai_results) 
    {
        if (item.label == "cone")
        {
            if (item.x < IMAGE_WIDTH / 2)
                cones_left.push_back(cv::Point(item.x, item.y));
            else
                cones_right.push_back(cv::Point(item.x, item.y));
        }
    }
}

cv::Point RescueDetector::searchHeighestCone()
{
    cv::Point heighest_cone(0, IMAGE_HEIGHT-1);
    for (auto& cone : cones_left)
    {
        if (cone.y < heighest_cone.y)
            heighest_cone = cone;
    }
    for (auto& cone : cones_right)
    {
        if (cone.y < heighest_cone.y)
            heighest_cone = cone;
    }
    return heighest_cone;
}

void RescueDetector::sortConesByX(std::vector<cv::Point>& cone_points)
{
    std::sort(cone_points.begin(), cone_points.end(), [](const cv::Point& a, const cv::Point& b) {
        return a.x < b.x;
    });
}