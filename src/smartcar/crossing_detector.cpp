#include <smartcar/crossing_detector.hpp>

void CrossingDetector::init()
{
    counter = 0;
    last_left_corner_y = -1;
    last_right_corner_y = -1;
    crossing_left_out = false;
    crossing_right_out = false;
}

void CrossingDetector::detect(LaneLine& lane_left, LaneLine& lane_right, TrackState& track_state)
{
    int crossing_in_threshold = 4;
    int crossing_out_threshold = 6;

    if (track_state == TrackState::NORMAL)
    {
        // std::cout << counter << std::endl;
        if (lane_left.corner.valid && lane_left.corner.angle < 0 && lane_right.corner.valid && lane_right.corner.angle > 0)
            counter++;
        else
        {
            counter--;
            if (counter < 0) counter = 0;
        }

        if (counter >= crossing_in_threshold)
        {
            track_state = TrackState::CROSSING_IN;
            counter = 0;
            last_left_corner_y = -1;
            last_right_corner_y = -1;
        }
    }
    else if (track_state == TrackState::CROSSING_IN)
    {
        if (lane_left.corner.valid && last_left_corner_y != -1 && last_left_corner_y - lane_left.corner.pos.y >= 20)
            crossing_left_out = true;
        if (lane_right.corner.valid && last_right_corner_y != -1 && last_right_corner_y - lane_right.corner.pos.y >= 20)
            crossing_right_out = true;
        
        if (crossing_left_out || crossing_right_out)
        {
            track_state  = TrackState::CROSSING_OUT;
            crossing_left_out = false;
            crossing_right_out = false;
        }

        if (lane_left.corner.valid) last_left_corner_y = lane_left.corner.pos.y;
        if (lane_right.corner.valid) last_right_corner_y = lane_right.corner.pos.y;
    }
    else if (track_state == TrackState::CROSSING_OUT)
    {
        if (!lane_left.corner.valid && !lane_right.corner.valid)
            counter++;
        else
        {
            counter--;
            if (counter < 0) counter = 0;
        }

        if (counter >= crossing_out_threshold)
        {
            track_state = TrackState::NORMAL;
            counter = 0;
        }
    }

}
