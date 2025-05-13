#include <smartcar/restaurant_detector.hpp>

/**
 * 待处理：ai_results的label需要根据实际模型进行调整
 * APPROACHING 之后的状态切换需要配合 cv图像处理辅助下位机45度转向？ 
 */

void RestaurantDetector::init()
{
    approaching_counter = 0;
    entering_turn_counter = 0;
    aligning_counter = 0;
    stopped_counter = 0;
    exiting_turn_counter = 0;

    approaching_threshold = 5;     // Example: 5 consecutive frames with sign
    entering_turn_threshold = 30;  // Example: Counter for turn duration/completion
    aligning_threshold = 20;       // Example: Counter for alignment
    stopped_duration_threshold = 50; // Example: Frames to stay stopped
    exiting_turn_threshold = 30;   // Example: Counter for exit turn

    // std::cout << "[RestaurantDetector] Initialized." << std::endl;
}

void RestaurantDetector::detect(const std::vector<PredictResult>& ai_results, TrackState& track_state)
{
    // Note: This is a very basic state machine.
    // Actual implementation would require more sophisticated logic for navigation,
    // angle checking, and potentially using lane line information.

    bool restaurant_sign_detected = false;
    // Hypothetical AI labels, adjust as per your model
    // bool food_pickup_zone_detected = false;
    // bool exit_path_clear = false;

    for (const auto& item : ai_results)
    {
        if (item.label == "burger") // Assume your AI model can detect this
        {
            restaurant_sign_detected = true;
        }
        // Add other relevant detections like "pickup_zone", "restaurant_entry_path" if available
    }

    switch (track_state)
    {
        case TrackState::NORMAL:
            if (restaurant_sign_detected)
            {
                approaching_counter++;
                if (approaching_counter >= approaching_threshold)
                {
                    track_state = TrackState::RESTAURANT_APPROACHING;
                    approaching_counter = 0; // Reset for next time or use different counters
   
                }
            }
            else
            {
                approaching_counter = std::max(0, approaching_counter - 1);
            }
            break;

        case TrackState::RESTAURANT_APPROACHING:
            // Logic to initiate the 45-degree turn
            // This might involve checking distance to sign, or a specific AI trigger
            // For now, simple counter after sign detection
            entering_turn_counter++;
            if (entering_turn_counter >= 10) // Placeholder for turn initiation logic
            {
                track_state = TrackState::RESTAURANT_ENTERING_TURN;
                entering_turn_counter = 0;
   
            }
            break;

        case TrackState::RESTAURANT_ENTERING_TURN:
            // Logic to monitor completion of the 45-degree turn and start aligning
            // This would involve IMU/odometry or visual cues
            aligning_counter++;
            if (aligning_counter >= entering_turn_threshold) // Placeholder for turn completion
            {
                track_state = TrackState::RESTAURANT_ALIGNING;
                aligning_counter = 0;
     
            }
            break;

        case TrackState::RESTAURANT_ALIGNING:
            // Logic for aligning on the diagonal path, possibly waiting for AI "pickup_zone_detected"
            stopped_counter++;
            if (stopped_counter >= aligning_threshold) // Placeholder for alignment completion
            {
                track_state = TrackState::RESTAURANT_STOPPED;
                stopped_counter = 0;
            
            }
            break;

        case TrackState::RESTAURANT_STOPPED:
            // Logic to wait for a certain duration
            exiting_turn_counter++;
            if (exiting_turn_counter >= stopped_duration_threshold)
            {
                track_state = TrackState::RESTAURANT_EXITING_TURN;
                exiting_turn_counter = 0;
          
            }
            break;

        case TrackState::RESTAURANT_EXITING_TURN:
            // Logic to execute and monitor the 45-degree exit turn
            approaching_counter++; // Re-using counter for simplicity
            if (approaching_counter >= exiting_turn_threshold) // Placeholder
            {
                track_state = TrackState::RESTAURANT_COMPLETED;
                approaching_counter = 0;
            
            }
            break;
        
        case TrackState::RESTAURANT_COMPLETED:
            // Transition back to NORMAL, perhaps after a short straight drive
            // This might be immediate or after a few frames.
            track_state = TrackState::NORMAL;
    
            break;

        default:
            // If in any other restaurant sub-state and restaurant_sign is lost for too long, consider resetting.
            // This part needs careful thought to avoid premature exits.
            break;
    }
}