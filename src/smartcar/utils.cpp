#include <smartcar/utils.hpp>

int IMAGE_HEIGHT = 0;
int IMAGE_WIDTH = 0;

uchar calAdaptiveThreshold(const cv::Mat &gray_image, int x, int y, int block_size, int bias)
{
    int half_block_size = block_size / 2;
    int start_x = std::max(0, x - half_block_size);
    int start_y = std::max(0, y - half_block_size);
    int end_x = std::min(gray_image.cols - 1, x + half_block_size);
    int end_y = std::min(gray_image.rows - 1, y + half_block_size);

    // 计算阈值（正方形范围内所有点像素值的均值）
    float sum = 0;
    int count = 0;
    for (int i = start_y; i <= end_y; ++i)
    {
        for (int j = start_x; j <= end_x; ++j)
        {
            sum += gray_image.at<uchar>(i, j);
            count++;
        }
    }
    double mean = sum / count;

    // 减去经验误差值，保证当采样范围内的所有像素点差别不大时不会强行求出一个阈值
    uchar threshold = static_cast<uchar>(mean - bias);
    // 大于阈值二值化为255，否则阈值化为0
    return gray_image.at<uchar>(y, x) > threshold ? 255 : 0;
}

bool calColorDeltaThreshold(const cv::Mat &gray_image, const cv::Point2f &start, const cv::Point2f &end, int delta)
{
    // Bresenham
    cv::LineIterator it(gray_image, start, end, 8);
    uchar color;
    uchar prev_color = gray_image.at<uchar>(start);

    for (int i = 0; i < it.count; i++, ++it)
    {
        color = gray_image.at<uchar>(it.pos());
        if (std::abs(color - prev_color) > delta)
        {
            return false;
        }
        prev_color = color;
    }
    return true;
}

double dotProduct(const cv::Point2f& v1, const cv::Point2f& v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}

double crossProduct(const cv::Point2f& v1, const cv::Point2f& v2)
{
    return v1.x * v2.y - v1.y * v2.x;
}

double norm(const cv::Point2f& v)
{
    return sqrt(v.x * v.x + v.y * v.y);
}

int factorial(int x)
{
    int f = 1;
    for (int i = 1; i <= x; i++)
    {
        f *= i;
    }
    return f;
}

std::vector<cv::Point> bezier(double dt, const std::vector<cv::Point>& input)
{
    std::vector<cv::Point> output;

    double t = 0;
    while (t <= 1)
    {
        cv::Point p;
        double x_sum = 0.0;
        double y_sum = 0.0;
        int i = 0;
        int n = input.size() - 1;
        while (i <= n)
        {
            double k =
                factorial(n) / (factorial(i) * factorial(n - i)) * pow(t, i) * pow(1 - t, n - i);
            x_sum += k * input[i].x;
            y_sum += k * input[i].y;
            i++;
        }
        p.x = x_sum;
        p.y = y_sum;
        output.push_back(p);
        t += dt;
    }
    return output;
}