#include <smartcar/bridge_detector.hpp>

void BridgeDetector::init()
{
    enter_counter = 0;
    quit_counter = 0;
    enter_threshold = 7;
    quit_threshold = 100;
}

void BridgeDetector::detect(const std::vector<PredictResult>& ai_results, TrackState& track_state)
{
    if (track_state == TrackState::NORMAL)
    {
        bool is_bridge_detected = false;
        bool is_others_detected = false;
        for (auto& item : ai_results)
        {
            if (item.label == "bridge")
                is_bridge_detected = true;
            else
                is_others_detected = true;
        }
        if (is_bridge_detected && !is_others_detected) enter_counter++;
        else {
            enter_counter--;
            if (enter_counter < 0) enter_counter = 0;
        }

        if (enter_counter >= enter_threshold) track_state = TrackState::BRIDGE;
    }

    else if (track_state == TrackState::BRIDGE)
    {
        bool is_bridge_detected = false;
        for (auto& item : ai_results)
        {
            if (item.label == "bridge")
            {
                is_bridge_detected = true;
                break;
            }
        }
        if (!is_bridge_detected) quit_counter++;
        else {
            quit_counter--;
            if (quit_counter < 0) quit_counter = 0;
        }

        if (quit_counter >= quit_threshold) track_state = TrackState::NORMAL;
    }
}