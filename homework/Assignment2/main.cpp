// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"
#include <eigen3/Eigen/Eigen>


constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f mo = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

//    double radia = MY_PI * rotation_angle / 180.0;
//
//    mo << cos(radia), -sin(radia), 0.0, 0.0,
//            sin(radia), cos(radia), 0.0, 0.0,
//            0.0, 0.0, 1.0, 0.0,
//            0.0, 0.0, 0.0, 1.0;

    return mo;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Copy-paste your implementation from the previous assignment.

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    Eigen::Matrix4f paraToOrtho = Eigen::Matrix4f::Identity();
    paraToOrtho << zNear, 0, 0, 0,
            0, zNear, 0, 0,
            0, 0, (zNear + zFar), -zNear * zFar,
            0, 0, 1, 0;

    float radianFov = MY_PI * eye_fov / 180.0;
    float r = -aspect_ratio * zNear * tan(radianFov / 2);
    float l = -r;
    float b = zNear * tan(radianFov / 2);
    float t = -b;

    Eigen::Matrix4f orthToZero = Eigen::Matrix4f::Identity();
    orthToZero << 2 / (r - l), 0, 0, 0,
            0, 2 / (t - b), 0, 0,
            0, 0, 2 / (zNear - zFar), -(zNear + zFar) / 2,
            0, 0, 0, 1;

    return orthToZero * paraToOrtho;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    axis.normalize();

    float radianAngle = MY_PI * angle / 180.0;

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f aaT = axis * axis.transpose();
    Eigen::Matrix3f N;
    N << 0, -axis.z(), axis.y(),
            axis.z(), 0, -axis.x(),
            -axis.y(), axis.x(), 0;

    Eigen::Matrix3f rotation = cos(radianAngle) * I + (1 - cos(radianAngle)) * aaT + sin(radianAngle) * N;
    Eigen::Matrix4f res = Eigen::Matrix4f::Identity();
    res.block<3, 3>(0, 0) = rotation;

    return res;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    float x = 0;
    float y = 0;
    float z = -10;

    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0,0,5};


    std::vector<Eigen::Vector3f> pos
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    std::vector<Eigen::Vector3f> cols
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    Eigen::Vector3f axis = {5, 0.5, 3}; // rotation axis

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

//        r.set_model(get_rotation(axis, angle));
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27) {
//        Eigen::Vector3f eye_pos = {x, y, z};
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

//        r.set_model(get_rotation(axis, angle));
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

//        if (key == 'a') {
//            angle += 10;
//        }
//        else if (key == 'd') {
//            angle -= 10;
//        }
//        else if (key == 'j') {
//            x -= 1;
//        }
//        else if (key == 'l') {
//            x += 1;
//        }
//        else if (key == 'i') {
//            y += 1;
//        }
//        else if (key == 'k') {
//            y -= 1;
//        }
//        else if (key == 'n') {
//            z -= 1;
//        }
//        else if (key == 'm') {
//            z += 1;
//        }
    }

    return 0;
}
// clang-format on