#include <smartcar/ring_detector.hpp>

void RingDetector::init()
{
    counter = 0;
    counter_run = 0;
}

void RingDetector::detect(LaneLine& lane_left, LaneLine& lane_right, TrackState& track_state)
{    
    int ring_begin_threshold = 10;
    int ring_in_threshold = 15;
    int ring_run_threshold = 3;
    int ring_out_threshold_1 = 15;
    int ring_out_threshold_2 = 5;
    int ring_end_threshold = 5;
    int ring_to_normal_threshold = 8;

    if (track_state == TrackState::NORMAL)
    {
        if (!lane_left.corner.valid && lane_right.corner.valid && lane_right.corner.angle > 0)
            counter++;
        else
        {
            counter--;
            if (counter < 0) counter = 0;
        }

        if (counter >= ring_begin_threshold)
        {
            track_state = TrackState::RING_BEGIN;
            counter = 0;
        }
    }
    else if (track_state == TrackState::RING_BEGIN)
    {
        if (!lane_left.corner.valid && (lane_right.H_points.empty() || !lane_right.corner.valid))
            counter++;
        else
        {
            counter--;
            if (counter < 0) counter = 0;
        }

        if (counter >= ring_in_threshold) {
            track_state = TrackState::RING_IN;
            counter = 0;
        }
    }
    else if (track_state == TrackState::RING_IN)
    {
        if (lane_left.corner.valid && lane_left.corner.angle < 0 && !lane_right.corner.valid)
            counter++;
        else
        {
            counter--;
            if (counter < 0) counter = 0;
        }

        if (counter >= ring_run_threshold)
        {
            track_state = TrackState::RING_RUN;
            counter = 0;
        }
    }
    else if (track_state == TrackState::RING_RUN)
    {
        if (!lane_left.corner.valid && !lane_right.corner.valid)
            counter_run++;
        else
        {
            counter_run--;
            if (counter_run < 0) counter_run = 0;
        }

        if (counter_run >= ring_out_threshold_1)
        {
            if (lane_left.corner.valid && lane_left.corner.angle < 0 && !lane_right.corner.valid)
                counter ++;
            else
            {
                counter --;
                if (counter < 0) counter = 0;
            }
            if (counter >= ring_out_threshold_2) {
                track_state = TrackState::RING_OUT;
                counter = 0;
                counter_run = 0;
            }
        }
    }
    else if (track_state == TrackState::RING_OUT)
    {
        if (!lane_left.corner.valid)
            counter++;
        else
        {
            counter--;
            if (counter < 0) counter = 0;
        }

        if (counter >= ring_end_threshold)
        {
            track_state = TrackState::RING_END;
            counter = 0;
        }
    }
    else if (track_state == TrackState::RING_END)
    {
        if (!lane_left.corner.valid && !lane_right.corner.valid)
            counter++;
        else
        {
            counter--;
            if (counter < 0) counter = 0;
        }

        if (counter >= ring_to_normal_threshold)
        {
            track_state = TrackState::NORMAL;
            counter = 0;
        }
    }

}