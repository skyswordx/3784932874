/**
 * @file charging_detector.hpp
 * @brief 充电区识别类实现
 */

 #pragma once

 #include <smartcar/config.hpp>
 #include <smartcar/utils.hpp>
 
 /**
  * @brief 充电区识别类
  */
 class ChargingDetector
 {
   public:
     void init();
     void detect(const std::vector<PredictResult>& ai_results, TrackState& track_state);
 
   private:
     int approaching_counter;
     int spot_selection_counter;
     int entering_turn_counter;
     int parking_counter;
     int charging_duration_counter;
     int reversing_counter;
     int exiting_counter;
 
     int approaching_threshold;
     int spot_selection_threshold;
     int entering_turn_threshold;
     int parking_threshold;
     int charging_duration_threshold;
     int reversing_threshold;
     int exiting_threshold;
 
     // Example: Store ID or coordinates of the selected parking spot
     // int selected_spot_id; 
 };