#include <smartcar/crosswalk_detector.hpp>

void CrosswalkDetector::init()
{
    begin_counter = 0;
    prestop_counter = 0;
    stop_counter = 0;
    begin_threshold = 10;
    prestop_threshold = 10;
    stop_threshold = 10;
    crosswalk_y = 0;
    is_detected = false;
}

void CrosswalkDetector::detect(const std::vector<PredictResult>& ai_results, TrackState& track_state)
{
    is_detected = false;

    if (track_state == TrackState::BEGIN)
    {
        bool is_crosswalk_detected = false;
        for (auto& item : ai_results)
        {
            if (item.label == "crosswalk")
            {
                is_crosswalk_detected = true;
                crosswalk_y = item.y;
                is_detected = true;
                break;
            }
        }
        if (!is_crosswalk_detected) begin_counter++;
        else {
            begin_counter--;
            if (begin_counter < 0) begin_counter = 0;
        }

        if (begin_counter >= begin_threshold) track_state = TrackState::NORMAL;
    }

    else if (track_state == TrackState::NORMAL)
    {
        bool is_crosswalk_detected = false;
        for (auto& item : ai_results)
        {
            if (item.label == "crosswalk")
            {
                is_crosswalk_detected = true;
                crosswalk_y = item.y;
                is_detected = true;
                break;
            }
        }
        if (is_crosswalk_detected) prestop_counter++;
        else {
            prestop_counter--;
            if (prestop_counter < 0) prestop_counter = 0;
        }

        if (prestop_counter >= prestop_threshold) track_state = TrackState::PRESTOP;
    }

    else if (track_state == TrackState::PRESTOP)
    {
        bool is_crosswalk_detected = false;
        for (auto& item : ai_results)
        {
            if (item.label == "crosswalk")
            {
                is_crosswalk_detected = true;
                crosswalk_y = item.y;
                is_detected = true;
                break;
            }
        }
        if (!is_crosswalk_detected) stop_counter++;
        if (stop_counter >= stop_threshold) track_state = TrackState::STOP;
    }
}