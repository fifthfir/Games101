#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 8)
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
        auto point = std::pow(1 - t, 3) * p_0
                + 3 * t * std::pow(1 - t, 2) * p_1
                + 3 * std::pow(t, 2) * (1 - t) * p_2
                + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    std::size_t count = control_points.size();

    if (count == 1) {
//        std::cout << "end recursive" << std::endl;
        return control_points[0];
    }

    std::vector<cv::Point2f> newPoints;

    for (int i = 0; i < count - 1; i++) {
        auto pointDif = control_points[i + 1] - control_points[i];
        auto point = control_points[i] + pointDif * t;
        newPoints.push_back(point);
    }

    return recursive_bezier(newPoints, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    int n = 1;  // pixel num to this point
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
    for (double t = 0.0; t <= 1.0; t += 0.001) {
        // recursive Bezier algorithm.
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;

        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                float maxDis = std::sqrt(2.0 * std::pow(n, 2.0));
                int x = point.x + i;
                int y = point.y + j;
                if (x >= 0 && x <= 700 && y <= 700 && y >= 0) {
                    float dis = std::sqrt(std::pow(i, 2.0) + std::pow(j, 2.0));
                    int color = 255.0 * (1.0 - (dis / maxDis));
                    window.at<cv::Vec3b>(y, x)[1] = std::max((int)window.at<cv::Vec3b>(y, x)[1],color);
                }
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
//            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve2.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
