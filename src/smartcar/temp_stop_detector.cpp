#include <smartcar/temp_stop_detector.hpp>


void TempStopDetector::init()
{
    approaching_counter = 0;
    entering_counter = 0;
    stopped_duration_counter = 0;
    resuming_counter = 0;

    approaching_threshold = 5;
    entering_threshold = 15;    // Time/frames to complete entry
    stopped_duration_threshold = 70; // Frames to stay stopped
    resuming_threshold = 10;   // Time/frames to resume normal driving
   
}

void TempStopDetector::detect(const std::vector<PredictResult>& ai_results, TrackState& track_state)
{
    // Placeholder: actual logic would use AI to find "temp_stop_sign" and "dashed_stop_lines"

    bool temp_stop_sign_detected = false;
    // bool dashed_lines_detected = false;

    for (const auto& item : ai_results)
    {
        if (item.label == "school" || item.label == "company") // Assume AI model provides this
        {
            temp_stop_sign_detected = true;
        }
        // if (item.label == "dashed_stop_lines") // Hypothetical
        // {
        //     dashed_lines_detected = true;
        // }
    }

    switch (track_state)
    {
        case TrackState::NORMAL:
            if (temp_stop_sign_detected)
            {
                approaching_counter++;
                if (approaching_counter >= approaching_threshold)
                {
                    track_state = TrackState::TEMP_STOP_APPROACHING;
                    approaching_counter = 0;
                 
                }
            }
            else
            {
                approaching_counter = std::max(0, approaching_counter - 1);
            }
            break;

        case TrackState::TEMP_STOP_APPROACHING:
            // Logic to start entering the stop area
            // This might involve detecting dashed lines or specific positioning
            entering_counter++;
            if (entering_counter >= 10 ) // Placeholder: if (dashed_lines_detected && suitable_position)
            {
                track_state = TrackState::TEMP_STOP_ENTERING;
                entering_counter = 0;
              
            }
            break;

        case TrackState::TEMP_STOP_ENTERING:
            // Logic to confirm successful entry into the stop zone
            stopped_duration_counter++;
            if (stopped_duration_counter >= entering_threshold) // Placeholder for entry completion
            {
                track_state = TrackState::TEMP_STOP_STOPPED;
                stopped_duration_counter = 0;
           
            }
            break;

        case TrackState::TEMP_STOP_STOPPED:
            // Wait for a specified duration
            resuming_counter++;
            if (resuming_counter >= stopped_duration_threshold)
            {
                track_state = TrackState::TEMP_STOP_RESUMING;
                resuming_counter = 0;
        
            }
            break;

        case TrackState::TEMP_STOP_RESUMING:
            // Logic to ensure car has resumed normal path following
            approaching_counter++; // Re-using for simplicity
            if (approaching_counter >= resuming_threshold) // Placeholder
            {
                track_state = TrackState::NORMAL;
                approaching_counter = 0;
      
            }
            break;
        default:
            break;
    }
}