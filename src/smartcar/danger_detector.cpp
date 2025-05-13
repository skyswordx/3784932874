#include <smartcar/danger_detector.hpp>

void DangerDetector::init()
{
    enter_counter = 0;
    left_counter = 0;
    right_counter = 0;
    quit_counter = 0;
    enter_threshold = 2;
    left_threshold = 5;
    right_threshold = 5;
    quit_threshold = 20;
}

// void DangerDetector::detect(const std::vector<PredictResult>& ai_results, const LaneLine& lane_left, const LaneLine& lane_right, TrackState& track_state)
// {
//     if (track_state == TrackState::NORMAL || track_state == TrackState::DANGER_LEFT || track_state == TrackState::DANGER_RIGHT)
//     {
//         // std::cout << "LEFT: " << left_counter << " RIGHT: " << right_counter << " QUIT: " << quit_counter << std::endl;
//         double nearest_item_x = -1.0;
//         double nearest_item_y = -1.0;
//         double nearest_edge_left_x = -1.0;
//         double nearest_edge_right_x = -1.0;
//         for (auto& item : ai_results)
//         {
//             if (item.label == "cone" || item.label == "block")
//             {
//                 double edge_left_x = -1.0;
//                 double edge_right_x = -1.0;
//                 for (auto& p : lane_left.points)
//                 {
//                     int py = static_cast<int>(p.y);
//                     if (py <= item.y + 5 && py >= item.y - 5)
//                     {
//                         edge_left_x = p.x;
//                         break;
//                     }
//                 }
//                 for (auto& p : lane_right.points)
//                 {
//                     int py = static_cast<int>(p.y);
//                     if (py <= item.y + 5 && py >= item.y - 5)
//                     {
//                         edge_right_x = p.x;
//                         break;
//                     }
//                 }

//                 std::cout << "LEFT: " << edge_left_x << " RIGHT: " << edge_right_x << " ITEM: " << item.x << std::endl;
//                 // 只处理在车道线有效距离范围内的障碍物
//                 // if (edge_left_x == -1.0 && edge_right_x == -1.0)
//                 //     continue;
//                 if (item.x >= 40 && item.x <= 280);
//                 else continue;
                
                
//                 // 判断是否是离小车最近的障碍物
//                 if (nearest_item_x == -1.0)
//                 {
//                     nearest_item_x = item.x;
//                     nearest_item_y = item.y;
//                     nearest_edge_left_x = edge_left_x;
//                     nearest_edge_right_x = edge_right_x;  
//                 }
//                 else
//                 {
//                     if (item.y > nearest_item_y)
//                     {
//                         nearest_item_x = item.x;
//                         nearest_item_y = item.y;
//                         nearest_edge_left_x = edge_left_x;
//                         nearest_edge_right_x = edge_right_x;  
//                     }
//                 }
                
//             }
//         }

//         // 判断是否检测到一个距离小车最近的道路内障碍物
//         if (nearest_item_x != -1.0)
//         {
//             // 判断障碍物靠近左侧车道线还是右侧车道线
//             if (fabs(nearest_edge_right_x - nearest_item_x) >= 80)
//             {
//                 left_counter = (left_counter >= left_threshold) ? left_threshold : (left_counter+1);
//                 right_counter = (right_counter <= 0) ? 0 : (right_counter-1);
//                 quit_counter = (quit_counter <= 0) ? 0 : (quit_counter-1);
//             }
                
//             else
//             {
//                 right_counter = (left_counter >= left_threshold) ? left_threshold : (left_counter+1);
//                 left_counter = (right_counter <= 0) ? 0 : (right_counter-1);
//                 quit_counter = (quit_counter <= 0) ? 0 : (quit_counter-1);
//             }
//         }
//         else
//         {
//             left_counter = (left_counter <= 0) ? 0 : (left_counter-1);
//             right_counter = (right_counter <= 0) ? 0 : (right_counter-1);
//             quit_counter = (quit_counter >= quit_threshold) ? quit_threshold : (quit_counter+1);
//         }

//         if (left_counter >= left_threshold)
//         {
//             track_state = TrackState::DANGER_LEFT;
//             left_counter = 0;
//             right_counter = 0;
//             quit_counter = 0;
//         }
//         else if (right_counter >= right_threshold)
//         {
//             track_state = TrackState::DANGER_RIGHT;
//             left_counter = 0;
//             right_counter = 0;
//             quit_counter = 0;
//         }
//         else if (quit_counter >= quit_threshold)
//         {
//             track_state = TrackState::NORMAL;
//             left_counter = 0;
//             right_counter = 0;
//             quit_counter = 0;
//         }
//     }
    
// }

void DangerDetector::detect(const std::vector<PredictResult>& ai_results, const LaneLine& lane_left, const LaneLine& lane_right, TrackState& track_state)
{
    switch (track_state)
    {
        // NORMAL状态下检测是否有危险区标志，满足条件进入危险区
        case TrackState::NORMAL:
        {
            for (auto& item : ai_results)
            {
                // 改动：把19届的bomb标志替换为 block 和 cone
                if (item.label == "block" || item.label == "cone")
                {
                    // std::cout << "here\n";
                    enter_counter++;
                    break;
                }
            }

            // 如果满足阈值条件，进入危险区状态
            if (enter_counter >= enter_threshold)
            {
                enter_counter = 0;
                quit_counter = 0;
                left_counter = 0;
                right_counter = 0;
                track_state = TrackState::DANGER;
            }

            break;
        }

        // DANGER状态下，寻找在左/右侧的障碍物
        // 如果在一段连续的时间内都找不到，就退出危险区
        case TrackState::DANGER:
        {
            int left_avg_x = calculateAverageX(lane_left);
            int right_avg_x = calculateAverageX(lane_right);
            int max_y = -1;
            bool is_item_left = false;

            for (auto& item : ai_results)
            {
                if ((item.label == "block" || item.label == "cone") && item.y > max_y)
                {
                    max_y = item.y;
                    if (fabs(item.x-left_avg_x) <= fabs(item.x-right_avg_x)) is_item_left = true;
                    else is_item_left = false;
                }
            }

            if (max_y != -1)
            {
                if (is_item_left)
                {
                    left_counter = (left_counter >= left_threshold) ? left_threshold : (left_counter+1);
                    right_counter = (right_counter <= 0) ? 0 : (right_counter-1);
                    quit_counter = 0;
                }
                else
                {
                    right_counter = (right_counter >= right_threshold) ? right_threshold : (right_counter+1);
                    left_counter = (left_counter <= 0) ? 0 : (left_counter-1);
                    quit_counter = 0;
                }
            }
            else
            {
                left_counter = (left_counter <= 0) ? 0 : (left_counter-1);
                right_counter = (right_counter <= 0) ? 0 : (right_counter-1);
                quit_counter = (quit_counter >= quit_threshold) ? quit_threshold : (quit_counter+1);
            }

            // 状态转换判断
            if (left_counter >= left_threshold)
            {
                enter_counter = 0;
                quit_counter = 0;
                left_counter = 0;
                right_counter = 0;
                track_state = TrackState::DANGER_LEFT;
            }
            else if (right_counter >= right_threshold)
            {
                enter_counter = 0;
                quit_counter = 0;
                left_counter = 0;
                right_counter = 0;
                track_state = TrackState::DANGER_RIGHT;
            }
            else if (quit_counter >= quit_threshold)
            {
                enter_counter = 0;
                quit_counter = 0;
                left_counter = 0;
                right_counter = 0;
                track_state = TrackState::NORMAL;
            }

            break;
        }

        case TrackState::DANGER_LEFT:
        {
            int left_avg_x = calculateAverageX(lane_left);
            int right_avg_x = calculateAverageX(lane_right);
            int max_y = -1;
            bool is_item_left = false;

            for (auto& item : ai_results)
            {
                if ((item.label == "block" || item.label == "cone") && item.y > max_y)
                {
                    max_y = item.y;
                    if (fabs(item.x-left_avg_x) <= fabs(item.x-right_avg_x)) is_item_left = true;
                    else is_item_left = false;
                }
            }

            if (max_y != -1)
            {
                if (is_item_left)
                {
                    left_counter = (left_counter >= left_threshold) ? left_threshold : (left_counter+1);
                    right_counter = (right_counter <= 0) ? 0 : (right_counter-1);
                    quit_counter = 0;
                }
                else
                {
                    right_counter = (right_counter >= right_threshold) ? right_threshold : (right_counter+1);
                    left_counter = (left_counter <= 0) ? 0 : (left_counter-1);
                    quit_counter = 0;
                }
            }
            else
            {
                left_counter = (left_counter <= 0) ? 0 : (left_counter-1);
                right_counter = (right_counter <= 0) ? 0 : (right_counter-1);
                quit_counter = (quit_counter >= quit_threshold) ? quit_threshold : (quit_counter+1);
            }

            // 状态转换判断
            if (left_counter >= left_threshold)
            {
                enter_counter = 0;
                quit_counter = 0;
                left_counter = 0;
                right_counter = 0;
                track_state = TrackState::DANGER_LEFT;
            }
            else if (right_counter >= right_threshold)
            {
                enter_counter = 0;
                quit_counter = 0;
                left_counter = 0;
                right_counter = 0;
                track_state = TrackState::DANGER_RIGHT;
            }
            else if (quit_counter >= quit_threshold)
            {
                enter_counter = 0;
                quit_counter = 0;
                left_counter = 0;
                right_counter = 0;
                track_state = TrackState::DANGER;
            }

            break;
        }

        case TrackState::DANGER_RIGHT:
        {
            int left_avg_x = calculateAverageX(lane_left);
            int right_avg_x = calculateAverageX(lane_right);
            int max_y = -1;
            bool is_item_left = false;

            for (auto& item : ai_results)
            {
                if ((item.label == "block" || item.label == "cone") && item.y > max_y)
                {
                    max_y = item.y;
                    if (fabs(item.x-left_avg_x) <= fabs(item.x-right_avg_x)) is_item_left = true;
                    else is_item_left = false;
                }
            }

            if (max_y != -1)
            {
                if (is_item_left)
                {
                    left_counter = (left_counter >= left_threshold) ? left_threshold : (left_counter+1);
                    right_counter = (right_counter <= 0) ? 0 : (right_counter-1);
                    quit_counter = 0;
                }
                else
                {
                    right_counter = (right_counter >= right_threshold) ? right_threshold : (right_counter+1);
                    left_counter = (left_counter <= 0) ? 0 : (left_counter-1);
                    quit_counter = 0;
                }
            }
            else
            {
                left_counter = (left_counter <= 0) ? 0 : (left_counter-1);
                right_counter = (right_counter <= 0) ? 0 : (right_counter-1);
                quit_counter = (quit_counter >= quit_threshold) ? quit_threshold : (quit_counter+1);
            }

            // 状态转换判断
            if (left_counter >= left_threshold)
            {
                enter_counter = 0;
                quit_counter = 0;
                left_counter = 0;
                right_counter = 0;
                track_state = TrackState::DANGER_LEFT;
            }
            else if (right_counter >= right_threshold)
            {
                enter_counter = 0;
                quit_counter = 0;
                left_counter = 0;
                right_counter = 0;
                track_state = TrackState::DANGER_RIGHT;
            }
            else if (quit_counter >= quit_threshold)
            {
                enter_counter = 0;
                quit_counter = 0;
                left_counter = 0;
                right_counter = 0;
                track_state = TrackState::DANGER;
            }
            break;
        }

        default:
            return;
    }
}

int DangerDetector::calculateAverageX(const LaneLine& lane)
{
    int sum = 0;
    for (auto& p : lane.points)
        sum += p.x;
    return sum / lane.points.size();
}