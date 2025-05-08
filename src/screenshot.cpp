// #include <opencv2/opencv.hpp>
// #include <iostream>

// using namespace cv;
// using namespace std;

// // 回调函数，用于处理鼠标事件
// void onMouse(int event, int x, int y, int flags, void* userdata) {
//     if (event == EVENT_LBUTTONDOWN) { // 如果检测到鼠标左键按下事件
//         Point* pt = (Point*)userdata; // 将userdata强制转换为Point指针
//         pt->x = x; // 将鼠标点击的x坐标保存到pt指向的对象中
//         pt->y = y; // 将鼠标点击的y坐标保存到pt指向的对象中
//     }
// }

// int main() {
//     // 读取图片
//     Mat image = imread("../res/frame_2420.jpg");
    
//     // 检查图片是否成功读取
//     if (image.empty()) {
//         cout << "Error: Cannot read image file." << endl;
//         return -1;
//     }
    
//     // 创建窗口并显示图片
//     namedWindow("Image", WINDOW_NORMAL);
//     imshow("Image", image);
    
//     // 创建一个Point对象来保存用户点击的坐标
//     Point clickedPoint(-1, -1);
    
//     // 设置鼠标事件回调函数
//     setMouseCallback("Image", onMouse, (void*)&clickedPoint);
    
//     // 循环检测是否有点击事件发生
//     while (true) {
//         // 如果用户点击了鼠标左键，则显示点击的坐标
//         if (clickedPoint.x != -1 && clickedPoint.y != -1) {
//             cout << "Clicked point coordinates: (" << clickedPoint.x << ", " << clickedPoint.y << ")" << endl;
//             // 重置clickedPoint对象，以便继续检测下一个点击事件
//             clickedPoint.x = -1;
//             clickedPoint.y = -1;
//         }
        
//         // 等待一段时间，检测用户点击事件
//         if (waitKey(10) == 27) // 如果用户按下ESC键，则退出循环
//             break;
//     }
    
//     // 关闭窗口
//     destroyAllWindows();
    
//     return 0;
// }

#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 读取图片
    cv::Mat image = cv::imread("../res/frame_2420.jpg");  // 替换为你的图片路径
    if (image.empty()) {
        std::cout << "Error loading the image" << std::endl;
        return -1;
    }

    // 显示原始图片
    cv::imshow("Original Image", image);

    // 设置源点和目标点
    std::vector<cv::Point2f> srcPoints, dstPoints;

    // 源点 - 这些点应根据实际情况调整
    srcPoints.push_back(cv::Point2f(86, 55));    // 左上角
    srcPoints.push_back(cv::Point2f(234, 55));  // 右上角
    srcPoints.push_back(cv::Point2f(47, 117));  // 右下角
    srcPoints.push_back(cv::Point2f(273, 117));  // 左下角

    // 目标点 - 透视变换的目的地点，通常是一个矩形或正方形
    dstPoints.push_back(cv::Point2f(114, 55 + 70));
    dstPoints.push_back(cv::Point2f(206, 55 + 70));
    dstPoints.push_back(cv::Point2f(110, 117 + 70));
    dstPoints.push_back(cv::Point2f(210, 117 + 70));

    // 计算透视变换矩阵
    cv::Mat perspectiveMatrix = cv::getPerspectiveTransform(srcPoints, dstPoints);

    // 创建输出图像
    cv::Mat transformedImage;
    cv::Size size(image.cols, image.rows);  // 可以调整输出大小

    // 应用透视变化
    cv::warpPerspective(image, transformedImage, perspectiveMatrix, size);

    // 显示变换后的图片
    cv::imshow("Transformed Image", transformedImage);

    // 等待用户按键
    cv::waitKey(0);

    return 0;
}

