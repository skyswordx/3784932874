#include <smartcar/motion_data.hpp>

void MotionData::init(const Config &config)
{
    try
    {
        yaw = config.get<double>("data_init_yaw");
        motion_speed = config.get<double>("data_init_speed");
        stop_flag = false;
        bridge_flag = false;
        danger_flag = false;
        reverse_flag = false;
    }
    catch(const std::exception& e)
    {
        std::cerr << "[Data] An error occurred while reading config: " << e.what() << std::endl;
    }
    
}
