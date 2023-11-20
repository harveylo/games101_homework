#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

// Angle is in radians or degrees?
// I will assume it is in radians here.
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    // Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    Eigen::Matrix4f rotate;
    rotate << std::cos(rotation_angle/180.0*MY_PI), -std::sin(rotation_angle/180.0*MY_PI), 0, 0,
              std::sin(rotation_angle/180.0*MY_PI), std::cos(rotation_angle/180.0*MY_PI), 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

    return rotate;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection; 
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float t = std::tan(eye_fov/2.0/180.0*MY_PI) * std::abs(zNear);
    float r = t * aspect_ratio;
    float l = -r;
    float b = -t;

    Eigen::Matrix4f persp2ortho;
    persp2ortho << zNear,0,0,0,
                    0,zNear,0,0,
                    0,0,zNear+zFar,-zNear*zFar,
                    0,0,1,0;
    
    Eigen::Matrix4f shift;
    Eigen::Matrix4f  scale;
    scale << 2.0/(r-l),0,0,0,
            0,2.0/(t-b),0,0,
            0,0,2.0/(zNear-zFar),0,
            0,0,0,1;
    shift << 1.0,.0,.0,-(r+l)/2.0,
            .0,1.0,.0,-(b+t)/2.0,
            .0,.0,1.0,-(zNear+zFar)/2.0,
            .0,.0,.0,1.0;
    Eigen::Matrix4f ortho = scale*shift;

    projection = ortho * persp2ortho;
    
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cerr << "\rframe count: " << frame_count++<<std::flush;

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
