#ifdef ENABLE_SERIAL_MODULE
#include <smartcar/communicator.hpp>

void Communicator::init(const Config &config)
{
#ifdef __linux__
    // 导入串口设备路径
    std::string str_serial_port = config.get<std::string>("communicator_serial_port");
    const char *serial_port = str_serial_port.c_str();

    // 打开串口
    serial.Open(serial_port);
    if (!serial.IsOpen())
    {
        std::cerr << "Unable to open the serial port " << serial_port << std::endl;
        return;
    }

    // 配置串口参数
    serial.SetBaudRate(LibSerial::BaudRate::BAUD_460800);
    serial.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial.SetParity(LibSerial::Parity::PARITY_NONE);
    // serial.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
#endif

    yaw_str = config.get<float>("data_init_yaw");
    motion_speed_str = config.get<float>("data_init_speed");
    stop_flag_str = int(false);

    DataPacket packet = {'a', 'a', 'a', 'a', yaw_str, motion_speed_str, 'f', 'f', stop_flag_str, 'b'};
    char sent_data[sizeof(DataPacket)];
    std::memcpy(sent_data, &packet, sizeof(packet));
#ifdef __linux__
    serial.write(sent_data, sizeof(sent_data));
#endif
    std::cout << "[Communication] Ready!!!" << std::endl;
}

void Communicator::send_bytes(MotionData &data)
{
    // 更新缓冲区和计数器
    char input_tag = 'f';
    char input_info = 'f';
    if (data.stop_flag) input_info = 'z';
    if (data.bridge_flag) input_info = 'k';
    if (data.reverse_flag) input_info = 'g';
    if (data.is_curve) input_tag = 'n';
    if (data.danger_flag) input_tag = 'd';

    update_buffer(tag_buffer, input_tag, tag_count);

    // 计算出现最多的tag
    char tag = get_most_frequent(tag_count);
    char info = input_info;

    // 要发送的初始数据
    yaw_str = float(data.yaw);
    motion_speed_str = float(data.motion_speed);
    stop_flag_str = int(data.stop_flag);
    DataPacket packet = {'a', 'a', 'a', 'a', yaw_str, motion_speed_str, tag, info, stop_flag_str, 'b'};

    char sent_data[sizeof(DataPacket)];
    std::memcpy(sent_data, &packet, sizeof(packet));
    std::cout << sent_data[13] << std::endl;
    // 发送数据
#ifdef __linux__
    serial.write(sent_data, sizeof(sent_data));
#endif
    std::string text = "[Serial] yaw: " + std::to_string(packet.yaw) + "speed: " + std::to_string(packet.speed) + '\n';
    std::cout << text;
    
}
#endif

void Communicator::update_buffer(std::vector<char>& buffer, char value, std::unordered_map<char, int>& count)
{
    if (buffer.size() >= buffer_size) {
        char old_value = buffer.front();
        buffer.erase(buffer.begin());
        count[old_value]--;
        if (count[old_value] == 0) {
            count.erase(old_value);
        }
    }
    buffer.push_back(value);
    if (count.find(value) == count.end()) {
        count[value] = 1;
    } else {
        count[value]++;
    }
}

char Communicator::get_most_frequent(const std::unordered_map<char, int>& count) {
    return std::max_element(count.begin(), count.end(),
                            [](const std::pair<char, int>& a, const std::pair<char, int>& b) {
                                return a.second < b.second;
                            })->first;
}