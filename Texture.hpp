//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto u00 = std::floor(u_img), v00 = std::floor(v_img);
        auto u01 = std::floor(u_img), v01 = std::ceil(v_img);
        auto u10 = std::ceil(u_img), v10 = std::floor(v_img);
        auto u11 = std::ceil(u_img), v11 = std::ceil(v_img);
        auto color00 = image_data.at<cv::Vec3b>(v00, u00);
        auto color01 = image_data.at<cv::Vec3b>(v01, u01);
        auto color10 = image_data.at<cv::Vec3b>(v10, u10);
        auto color11 = image_data.at<cv::Vec3b>(v11, u11);
        auto color0 = color00 + (color10 - color00) * (u_img - u00);
        auto color1 = color01 + (color11 - color01) * (u_img - u01);
        auto color = color0 + (color1 - color0) * (v_img - v00);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
