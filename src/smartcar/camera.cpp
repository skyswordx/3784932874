#include <smartcar/camera.hpp>

Camera::~Camera()
{
    cap.release();
}

void Camera::init(const Config& config)
{
    // 从Config中读取参数
    try
    {
        video_src = config.get<std::string>("camera_video_src");
        video_width = config.get<int>("camera_video_width");
        video_height = config.get<int>("camera_video_height");
        resize_height = config.get<int>("camera_resize_height");
        resize_width = config.get<int>("camera_resize_width");
        alpha = config.get<double>("camera_alpha");
        beta = config.get<double>("camera_beta");
        min_gray_enable = config.get<bool>("camera_min_gray_enable");
        correct_enable = config.get<bool>("camera_correct_enable");
        calibration_file_path = config.get<std::string>("camera_calibration_file");
        IMAGE_HEIGHT = resize_height;
        IMAGE_WIDTH = resize_width;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Camera] An error occurred while reading config: " << e.what() << std::endl;
        return;
    }

    // 读取视频文件or摄像头
    if (!cap.open(video_src))
    {
        std::cerr << "[Camera] Error opening video stream or file" << std::endl;
    }
    
    // 设定读入图像宽高
    try
    {
        cap.set(cv::CAP_PROP_FRAME_WIDTH, video_width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, video_height);
        bool success = cap.set(cv::CAP_PROP_AUTO_WB, 0);
        if (!success) {
        std::cerr << "Error in closing AUTO_WB" << std::endl;
        } else {
            std::cout << "Success in closing AUTO_WB" << std::endl;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << "[Camera] An error occurred while setting the size of video: " << e.what() << std::endl;
    }

    // 图像去畸参数初始化
    if (correct_enable)
    {
        cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); 
        distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
        cv::FileStorage file;
        if (file.open(calibration_file_path, cv::FileStorage::READ)) // 读取本地保存的标定文件
        {
            file["cameraMatrix"] >> cameraMatrix;
            file["distCoeffs"] >> distCoeffs;
            std::cout << "[Camera] ready!!!" << std::endl;
        }
        else
        {
            std::cout << "[Camera] Loading calibration info failed!" << std::endl;
            correct_enable = false;
        }
    }
    
}

bool Camera::read(cv::Mat& image)
{

    if (cap.read(image))
    {
        cv::resize(image, image, cv::Size(resize_width, resize_height));
        return true;
    }
    else
        return false;
}

cv::Mat Camera::convertToGray(const cv::Mat& image)
{
    cv::Mat gray_image;

    if (min_gray_enable)
    {
    // 灰度图：将源图像分解为三个单通道图像，找到每个像素的最小值
        std::vector<cv::Mat> channels;
        cv::split(image, channels);
        cv::min(channels[0], channels[1], gray_image);
        cv::min(gray_image, channels[2], gray_image);
    }
    else
    {
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    }

    return gray_image;
}

cv::Mat Camera::correct(const cv::Mat& image)
{
    if(correct_enable)
    {
        cv::Size sizeImage; // 图像的尺寸
        sizeImage.width = image.cols;
        sizeImage.height = image.rows;

        cv::Mat mapx = cv::Mat(sizeImage, CV_32FC1);	// 经过矫正后的X坐标重映射参数
        cv::Mat mapy = cv::Mat(sizeImage, CV_32FC1);	// 经过矫正后的Y坐标重映射参数
        cv::Mat rotMatrix = cv::Mat::eye(3, 3, CV_32F); // 内参矩阵与畸变矩阵之间的旋转矩阵

        // 进行图像矫正
        cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, rotMatrix, cameraMatrix, sizeImage, CV_32FC1, mapx, mapy);
        cv::Mat imageCorrect = image.clone();
        cv::remap(image, imageCorrect, mapx, mapy, cv::INTER_LINEAR);
        return imageCorrect;
    }
    else
    {
        return image;
    }
}

void Camera::preprocess(cv::Mat& image)
{
    cv::GaussianBlur(image, image, cv::Size(5, 5), 0);
    image.convertTo(image, -1, alpha, beta);
}