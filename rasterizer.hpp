//
// Created by goksu on 4/6/19.
//

#pragma once

#include "Triangle.hpp"
#include <algorithm>
#include <eigen3/Eigen/Eigen>
using namespace Eigen;

namespace rst {

// This enum class indicates the type of a buffer
enum class Buffers
{
    Color = 1,
    Depth = 2
};

inline Buffers operator|(Buffers a, Buffers b)
{
    return Buffers((int)a | (int)b);
}

inline Buffers operator&(Buffers a, Buffers b)
{
    return Buffers((int)a & (int)b);
}

// This enum class indicates the type of primitive to be rendered
enum class Primitive
{
    Line,
    Triangle
};

/*
 * For the curious : The draw function takes two buffer id's as its arguments.
 * These two structs make sure that if you mix up with their orders, the
 * compiler won't compile it. Aka : Type safety
 * */
struct pos_buf_id
{
    int pos_id = 0;
};

struct ind_buf_id
{
    int ind_id = 0;
};

class rasterizer
{
  public:
    // The init function in the rasterizer, setting width and height.
    rasterizer(int w, int h);
    pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);
    ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);

    void set_model(const Eigen::Matrix4f& m);
    void set_view(const Eigen::Matrix4f& v);
    void set_projection(const Eigen::Matrix4f& p);

    void set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color);

    void clear(Buffers buff);

    void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, Primitive type);

    std::vector<Eigen::Vector3f>& frame_buffer() { return frame_buf; }

  private:
    void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);
    void rasterize_wireframe(const Triangle& t);

  private:
    // Model Transformation Matrix
    Eigen::Matrix4f model;
    // View Transformation Matrix
    Eigen::Matrix4f view;
    // Projection Transformation Matrix
    Eigen::Matrix4f projection;

    // stores all positions of vertices
    std::map<int, std::vector<Eigen::Vector3f>> pos_buf;
    // stores every triangle's three indices of vertices
    // for example, if an element of ind_buf is {0, 1, 2}, it means that
    // the corresponding triangle is made up of the vertices stored in pos_buf with indices 0, 1, 2
    std::map<int, std::vector<Eigen::Vector3i>> ind_buf;

    std::vector<Eigen::Vector3f> frame_buf;
    std::vector<float> depth_buf;
    int get_index(int x, int y);

    int width, height;

    int next_id = 0;
    int get_next_id() { return next_id++; }
};
} // namespace rst
