#include <smartcar/charging_detector.hpp>

/**
 * 待处理：ai_results的label需要根据实际模型进行调整
 * APPROACHING 之后的状态切换需要配合 cv图像处理辅助下位机转向？ 
 */

void ChargingDetector::init()
{
    approaching_counter = 0;
    spot_selection_counter = 0;
    entering_turn_counter = 0;
    parking_counter = 0;
    charging_duration_counter = 0;
    reversing_counter = 0;
    exiting_counter = 0;

    approaching_threshold = 5;
    spot_selection_threshold = 10; // e.g., frames to analyze spots
    entering_turn_threshold = 30;
    parking_threshold = 20;
    charging_duration_threshold = 100; // e.g., 100 frames for charging
    reversing_threshold = 30;
    exiting_threshold = 20;
    // selected_spot_id = -1;
;
}

void ChargingDetector::detect(const std::vector<PredictResult>& ai_results, TrackState& track_state)
{
    // Placeholder: actual logic would use AI to find "charging_station_sign",
    // "parking_spot_lines", "occupied_spot_indicator (blue_car_model)"
    // and lane information (lane_left, lane_right) for precise parking.

    bool charging_sign_detected = false;
    bool empty_spot_found = false; // This would be set by more complex logic

    for (const auto& item : ai_results)
    {
        if (item.label == "battery") // Assume AI model provides this
        {
            charging_sign_detected = true;
        }
        if (item.label != "car") // Hypothetical
        {
            empty_spot_found = true;
        }
    }

    switch (track_state)
    {
        case TrackState::NORMAL:
            if (charging_sign_detected)
            {
                approaching_counter++;
                if (approaching_counter >= approaching_threshold)
                {
                    track_state = TrackState::CHARGING_ZONE_APPROACHING;
                    approaching_counter = 0;
                  
                }
            }
            else
            {
                approaching_counter = std::max(0, approaching_counter - 1);
            }
            break;

        case TrackState::CHARGING_ZONE_APPROACHING:
            // Logic to start looking for parking spots
            spot_selection_counter++;
            if (spot_selection_counter > 5 && empty_spot_found) // Placeholder: needs actual spot detection
            {
                track_state = TrackState::CHARGING_SPOT_SELECTION;
                spot_selection_counter = 0;
                

            } else if (spot_selection_counter > 50 && !empty_spot_found) { // Timeout if no spot found
              
                 track_state = TrackState::NORMAL; // Or some error state
                 spot_selection_counter = 0;
            }
            break;

        case TrackState::CHARGING_SPOT_SELECTION:
            // After a spot is selected (e.g., by an AI subroutine or geometric analysis)
            // For now, assume it's selected and proceed to turn.
            // selected_spot_id = some_logic_to_get_spot_id(ai_results, lane_left, lane_right);
            track_state = TrackState::CHARGING_ENTERING_TURN;
           
            break;

        case TrackState::CHARGING_ENTERING_TURN:
            entering_turn_counter++;
            if (entering_turn_counter >= entering_turn_threshold) // Placeholder for turn completion
            {
                track_state = TrackState::CHARGING_PARKING;
                entering_turn_counter = 0;
              
            }
            break;

        case TrackState::CHARGING_PARKING:
            parking_counter++;
            if (parking_counter >= parking_threshold) // Placeholder for parking completion
            {
                track_state = TrackState::CHARGING_CHARGING;
                parking_counter = 0;
          
            }
            break;

        case TrackState::CHARGING_CHARGING:
            charging_duration_counter++;
            if (charging_duration_counter >= charging_duration_threshold)
            {
                track_state = TrackState::CHARGING_REVERSING;
                charging_duration_counter = 0;
              
            }
            break;

        case TrackState::CHARGING_REVERSING:
            reversing_counter++;
            if (reversing_counter >= reversing_threshold) // Placeholder for reversing completion
            {
                track_state = TrackState::CHARGING_EXITING;
                reversing_counter = 0;
          
            }
            break;
        
        case TrackState::CHARGING_EXITING:
            exiting_counter++;
            if (exiting_counter >= exiting_threshold) // Placeholder
            {
                track_state = TrackState::NORMAL;
                exiting_counter = 0;
            
            }
            break;
        default:
            break;
    }
}