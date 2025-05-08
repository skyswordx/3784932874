/**
 * @file main.cpp
 * @brief 智能车巡线程序入口
 *
 * @author lqf
 * @date 2024.3.3
 * @version 0.0
 */
#ifdef ENABLE_AI_MODULE
    #include <smartcar/ai_detector.hpp>
#endif

#ifdef ENABLE_SERIAL_MODULE
    #include <smartcar/communicator.hpp>
#endif

#include <smartcar/config.hpp>
#include <smartcar/camera.hpp>
#include <smartcar/motion_controller.hpp>
#include <smartcar/ring_detector.hpp>
#include <smartcar/motion_data.hpp>
#include <smartcar/utils.hpp>

#include <atomic>
#include <chrono>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <thread>
#include <condition_variable>

/**
 * @brief 智能车类
 *
 * 集成智能车巡线所有功能部件，线程管理，以及描述巡线算法主体逻辑
 */
class Car
{
  private:
    // Car类功能组件
    Config config;             ///< 配置文件读取部件
    Camera camera;             ///< 图像读取与处理部件
    MotionController motion;   ///< 车辆运动控制部件
    CrossingDetector crossing; ///< 十字检测部件
    RingDetector ring;         ///< 环路检测部件
    #ifdef ENABLE_AI_MODULE
        AIDetector ai;             ///< ai模型检测部件
    #endif
    #ifdef ENABLE_SERIAL_MODULE
        Communicator communicator; ///< 单片机通信部件
    #endif
    MotionData data;           ///< 运动数据存储部件

    bool display_enable;        ///< display线程启用使能
    bool dump_enable;           ///< dump线程启用使能

    // 并行相关变量
    std::mutex work2dump_mtx;                                    ///< 待输出图像队列互斥锁
    std::queue<std::pair<cv::Mat, cv::Mat>> work2dump_queue;     ///< 待输出图像队列，存储work线程的处理结果，用于视频存储
    std::mutex display_mtx;                                      ///< 存储display线程显示图像的互斥锁
    std::pair<cv::Mat, cv::Mat> display_frame;                   ///< 存储display线程显示图像
    std::mutex input_data_mtx;
    std::condition_variable input_data_cond;                     ///< 输入数据访问条件变量                                        ///< 输入数据访问互斥锁
    std::shared_ptr<InputData> input_data;           

    std::atomic<bool> exit_flag;                       ///< 退出信号

  public:
    /**
     * @brief 初始化函数
     *
     * 初始各个部件，加载配置等
     *
     * @author lqf
     */
    void init()
    {
        config.load("../config/config.json");
        camera.init(config);
        motion.init(config);
        crossing.init();
        ring.init();
        data.init(config);
        #ifdef ENABLE_AI_MODULE
            ai.init();
        #endif
        #ifdef ENABLE_SERIAL_MODULE
            communicator.init(config);
        #endif

        exit_flag = false;

        // 从Config中读取参数
        try
        {
            display_enable = config.get<bool>("display_enable");
            dump_enable = config.get<bool>("dump_enable");
        }
        catch (const std::exception &e)
        {
            std::cerr << "[Car] An error occurred while reading config: " << e.what() << std::endl;
            return;
        }

        std::cout << "[Car] IMAGE_WIDTH: " << IMAGE_WIDTH << std::endl;
        std::cout << "[Car] IMAGE_HEIGHT: " << IMAGE_HEIGHT << std::endl;
     }

    /**
     * @brief 巡线计算线程
     *
     * 该线程对输入图像进行相关计算，最终计算出车辆的运动数据
     */
    void work()
    {
        int count = 0;

        // work处理计时(fps)
        auto start_time = std::chrono::high_resolution_clock::now();
        int fps_count = 0;

        while (!exit_flag)
        {
            // 读取输入数据（图像、ai检测结果）
            std::shared_ptr<InputData> current_data;
            {
                std::unique_lock<std::mutex> lock(input_data_mtx);
                input_data_cond.wait(lock, [&]{ return input_data != nullptr; });
                current_data = input_data;
                input_data = nullptr;
            }
            cv::Mat src_frame = current_data->input_image;
            
            // 图像预处理
            cv::Mat gray_frame = camera.convertToGray(src_frame);
            camera.preprocess(gray_frame);       

            // 巡线
            motion.process(gray_frame, current_data->input_ai_result, data);

            #ifdef ENABLE_SERIAL_MODULE
            communicator.send_bytes(data);
            #endif

            cv::Mat pt_frame;
            if (display_enable || dump_enable)
            {
                // cv::Mat lane_frame = motion.display(gray_frame); // 生成车道线提取结果图像（该操作可放入调试线程执行）
                pt_frame = motion.getPerspectiveTransformedImage(src_frame);
                motion.drawBilateralLaneLineBeforePT(src_frame);
                motion.drawBilateralLaneLine(pt_frame);
                motion.drawMidLaneLine(pt_frame);
                motion.drawCorners(pt_frame);
                motion.drawTrackState(pt_frame);
                motion.drawLookahead(pt_frame);
                motion.drawValidCornerLine(pt_frame);
                motion.drawVerticalLine(pt_frame);
                #ifdef ENABLE_AI_MODULE
                ai.drawBox(src_frame, current_data->input_ai_result);
                #endif
            }
            // motion.printTrackState();

            // 临界区，将用于调试输出的图像放入队列，供display线程使用
            if (dump_enable)
            {
                std::lock_guard<std::mutex> lock(work2dump_mtx); // 申请锁
                work2dump_queue.emplace(src_frame, pt_frame);
            }
            if (display_enable)
            {
                std::lock_guard<std::mutex> lock(display_mtx);
                display_frame = {src_frame, pt_frame};
            }
            count++;

            // 计算处理用时（fps）
            fps_count++;
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_time = end_time - start_time;
            if (elapsed_time.count() >= 1.0)
            {
                double fps = (double)fps_count / elapsed_time.count();
                std::cout << "[Work Thread] FPS: " << fps << std::endl;
                start_time = std::chrono::high_resolution_clock::now();
                fps_count = 0;
            }

        }

        std::cout << "[Work Thread] " << count << " images" << std::endl;
    }

    /**
     * @brief 调试输出线程
     *
     * 用于将work线程的处理结果以图像形式输出，方便调试
     *
     * @author lqf
     */
    void display()
    {
        int count = 0;
        while (true)
        {
            std::pair<cv::Mat, cv::Mat> images;
            // 临界区，从共享图像变量中读取待输出图像
            {
                std::lock_guard<std::mutex> lock(display_mtx);
                images = display_frame;
                if (display_frame.first.empty() || display_frame.second.empty())
                {
                    if (exit_flag) break;
                    else continue;
                }
                display_frame = {};
            }

            // 显示图像
            cv::imshow("src", images.first);
            cv::imshow("lane", images.second);

            // 按'ESC'退出视频
            char c = static_cast<char>(cv::waitKey(1));
            if (c == 27)
            {
                std::cout << "[Display Thread] Quit\n";
                exit_flag = true;
                break;
            }
            else if (c == 's' || c == 'S') // 按 's' 键保存图片
            {
                std::string filename = "frame_" + std::to_string(count) + ".jpg";
                cv::imwrite(filename, images.first);
                std::cout << "[Display Thread] Frame saved as " << filename << std::endl;
            }

            count++;
        }

        std::cout << "[Display Thread] " << count << " images" << std::endl;
        cv::destroyAllWindows();
    }

    /**
     * @brief 巡线结果输出线程
     *
     * 该线程将巡线结果输出为视频文件
     *
     * @author ycr
     */
    void dump()
    {
        int count = 0;
        std::pair<cv::Mat, cv::Mat> imgs;

        while (true)
        {
            {
                std::lock_guard<std::mutex> lock(work2dump_mtx);
                if (!work2dump_queue.empty())
                {
                    imgs = work2dump_queue.front();
                    break;
                }
            }
        }

        cv::Mat combined;
        // cv::cvtColor(imgs.second, imgs.second, cv::COLOR_GRAY2BGR);
        cv::hconcat(imgs.first, imgs.second, combined);

        cv::VideoWriter video("../output.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 60,
                              cv::Size(combined.cols, combined.rows));

        while (true)
        {
            // 临界区，从图像队列中读取待输出图像
            {
                std::lock_guard<std::mutex> lock(work2dump_mtx);

                if (work2dump_queue.empty())
                {
                    if (exit_flag)
                        break;
                    else
                        continue;
                }

                imgs = work2dump_queue.front();
                work2dump_queue.pop();
                count++;
            }
            // cv::cvtColor(imgs.second, imgs.second, cv::COLOR_GRAY2BGR);
            cv::hconcat(imgs.first, imgs.second, combined);
            cv::putText(combined, std::to_string(count), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar(0, 255, 0), 2);
            video.write(combined);

            // 如果当前display线程没有启动，则创建一个黑色opencv窗口用于waikkey函数的使用
            if (!display_enable) {
                cv::Mat black_image = cv::Mat::zeros(50, 50, CV_8U);
                cv::imshow("dump", black_image);
                // 按'ESC'退出视频
                char c = (char)cv::waitKey(10);
                if (c == 27)
                {
                    exit_flag = true;
                    std::cout << "[Dump Thread] Exit signal is received ..." << std::endl;
                    continue;
                }
            }
        }

        std::cout << "[Dump Thread] " << count << " images" << std::endl;
        video.release();
    }

    /**
     * @brief ai推理线程
     *
     * 调用paddle api对输入图像进行目标检测
     */
    void infer()
    {
        int count = 0;
        while (!exit_flag)
        {
            std::shared_ptr<InputData> current_data = std::make_shared<InputData>();

            // 读入待处理图像
            if (!camera.read(current_data->input_image))
            {
                exit_flag = true;
                std::cout << "[Infer Thread] No frame captured from the video source" << std::endl;
                break;
            }
                       
            #ifdef ENABLE_AI_MODULE
            // 进行ai推理
            current_data->input_ai_result = ai.inference(current_data->input_image);
            #endif

            // 更新ai结果
            {
                std::unique_lock<std::mutex> lock(input_data_mtx);
                input_data = current_data;
                input_data_cond.notify_all();
            }
            
            count++;
        }

        std::cout << "[Infer Thread] " << count << " images" << std::endl;
    }

    /**
     * @brief 启动巡线程序
     *
     * @author lqf
     */
    void run()
    {
        // 启动线程
        std::thread work_thread([this] { work(); });

        std::unique_ptr<std::thread> dump_thread;
        if (dump_enable) {
            dump_thread = std::make_unique<std::thread>([this] { dump(); });
        }

        std::unique_ptr<std::thread> display_thread;
        if (display_enable) {
            display_thread = std::make_unique<std::thread>([this] { display(); });
        }

        std::thread infer_thread([this] { infer(); });

        work_thread.join();
        if (dump_thread) dump_thread->join();
        if (display_thread) display_thread->join();
        infer_thread.join();
    }

    /**
     * @brief 启动巡线程序（串行版）
     */
    void serialRun()
    {
        std::thread work_thread([this] { serialWork(); });
        work_thread.join();
    }



    /**
     * @brief 串行巡线线程
    */
    void serialWork()
    {
        int count = 0;
        auto start_time = std::chrono::high_resolution_clock::now();
        int fps_count = 0;

        while (!exit_flag)
        {
            std::shared_ptr<InputData> current_data = std::make_shared<InputData>();

            if (!camera.read(current_data->input_image))
            {
                exit_flag = true;
                std::cout << "[SerialWork Thread] No frame captured from the video source" << std::endl;
                break;
            }

            #ifdef ENABLE_AI_MODULE
            auto ai_start_time = std::chrono::high_resolution_clock::now();
            current_data->input_ai_result = ai.inference(current_data->input_image);
            auto ai_end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> ai_duration = ai_end_time - ai_start_time;
            std::cout << "[AI Inference] Time: " << ai_duration.count() << " seconds" << std::endl;
            #endif

            cv::Mat src_frame = current_data->input_image;
            
            // 图像预处理
            cv::Mat gray_frame = camera.convertToGray(src_frame);
            camera.preprocess(gray_frame);       

            // 巡线
            motion.process(gray_frame, current_data->input_ai_result, data);

            #ifdef ENABLE_SERIAL_MODULE
            communicator.send_bytes(data);
            #endif

            cv::Mat pt_frame;
            if (display_enable || dump_enable)
            {
                // cv::Mat lane_frame = motion.display(gray_frame); // 生成车道线提取结果图像（该操作可放入调试线程执行）
                pt_frame = motion.getPerspectiveTransformedImage(src_frame);
                motion.drawBilateralLaneLineBeforePT(src_frame);
                motion.drawBilateralLaneLine(pt_frame);
                motion.drawMidLaneLine(pt_frame);
                motion.drawCorners(pt_frame);
                motion.drawTrackState(pt_frame);
                motion.drawLookahead(pt_frame);
                #ifdef ENABLE_AI_MODULE
                ai.drawBox(src_frame, current_data->input_ai_result);
                #endif
            }

            // 临界区，将用于调试输出的图像放入队列，供display线程使用
            if (dump_enable)
            {
                std::lock_guard<std::mutex> lock(work2dump_mtx); // 申请锁
                work2dump_queue.emplace(src_frame, pt_frame);
            }
            if (display_enable)
            {
                std::lock_guard<std::mutex> lock(display_mtx);
                display_frame = {src_frame, pt_frame};
            }
            count++;

            // 计算处理用时（fps）
            fps_count++;
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_time = end_time - start_time;
            if (elapsed_time.count() >= 1.0)
            {
                double fps = (double)fps_count / elapsed_time.count();
                std::cout << "[SerialWork Thread] FPS: " << fps << std::endl;
                start_time = std::chrono::high_resolution_clock::now();
                fps_count = 0;
            }

        }

        std::cout << "[SerialWork Thread] " << count << " images" << std::endl;
    }
};

/**
 * @brief 程序入口
 */
int main()
{
    Car car;
    car.init();
    car.run();

    // system("pause");
    return 0;
}
