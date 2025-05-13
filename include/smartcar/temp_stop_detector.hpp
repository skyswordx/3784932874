/**
 * @file temp_stop_detector.hpp
 * @brief 临时停车区识别类实现
 */

 #pragma once

 #include <smartcar/config.hpp>
 #include <smartcar/utils.hpp>
 
 /**
  * @brief 临时停车区识别类
  */
 class TempStopDetector
 {
   public:
     void init();
     void detect(const std::vector<PredictResult>& ai_results, TrackState& track_state);
 
   private:
     int approaching_counter;
     int entering_counter;
     int stopped_duration_counter;
     int resuming_counter;
 
     int approaching_threshold;
     int entering_threshold;
     int stopped_duration_threshold;
     int resuming_threshold;
 };