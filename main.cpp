#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    // ? Do not need to be recursive
    // Use a queue is enough, and it will be faster without calling stack overhead
    std::queue<cv::Point2f> q;
    for (auto &point : control_points) 
    {
        q.push(point);
    }
    while (q.size() > 1) 
    {
        int size = q.size()-1;
        while (size-->0){
            auto& p0 = q.front();
            q.pop();
            auto&  p1 = q.front();
            auto np = (1-t)*p0 + t*p1;
            q.push(np);
        }
        q.pop();
    }
    return q.front();

}

template<typename T>
T lerp(T a, T b, float t) {
    return (1-t)*a + t*b;
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.0001) 
    {
        auto point = recursive_bezier(control_points, t);
        // implement the anti-aliasing of the curve
        // window.at<cv::Vec3b>(point.y, point.x)[1] = 255;

        // get topleft, topright, bottomleft, bottomright, the floor value should be compared with the height or width of the window
        // depending on the location of the point inside a pixel(less or no less than 0.5), the ajacent pixels should be different
        auto x = std::floor(point.x), y = std::floor(point.y);
        auto dx = point.x - x, dy = point.y - y;
        auto min_x = std::max(0.0f, dx<0.5f?x-1:x), max_x = std::min(window.cols-1.0f, dx<0.5f?x:x+1);
        auto min_y = std::max(0.0f, dy<0.5f?y-1:y), max_y = std::min(window.rows-1.0f, dy<0.5f?y:y+1);
        // auto min_x = x, max_x = std::ceil(x);
        // auto min_y = y, max_y = std::ceil(y);
        auto distance = std::numeric_limits<float>::max();
        for(auto tx : {min_x, max_x}){
            for(auto ty : {min_y, max_y}){
                auto dtx = tx+0.5f - point.x, dty = ty+0.5f - point.y;
                auto d = std::sqrt(dtx*dtx + dty*dty);
                distance = std::min(distance, d);
            }
        }

        // interpolate each pixel's value by lerp
        for(auto tx : {min_x, max_x}){
            for (auto ty : {min_y, max_y}){
                auto dtx = tx+0.5f - point.x, dty = ty+0.5f - point.y;
                auto d = std::sqrt(dtx*dtx + dty*dty);
                auto t = distance/d;
                // ! IMPORTANT: each pixel may be visited multiple times, thus we need to use max to get the max value
                window.at<cv::Vec3b>(ty, tx)[1] = std::max(255*t, (float)window.at<cv::Vec3b>(ty, tx)[1]);
            }
        }
    }

}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
