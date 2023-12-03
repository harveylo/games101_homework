//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f>& normals)
{
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

    return {id};
}


// Bresenham's line drawing algorithm
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    if(dy1<=dx1)
    {
        if(dx>=0)
        {
            x=x1;
            y=y1;
            xe=x2;
        }
        else
        {
            x=x2;
            y=y2;
            xe=x1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;x<xe;i++)
        {
            x=x+1;
            if(px<0)
            {
                px=px+2*dy1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    y=y+1;
                }
                else
                {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
    else
    {
        if(dy>=0)
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(float x, float y, const Vector4f* _v){
    Vector3f v[3];
    for(int i=0;i<3;i++)
        v[i] = {_v[i].x(),_v[i].y(), 1.0};
    Vector3f f0,f1,f2;
    f0 = v[1].cross(v[0]);
    f1 = v[2].cross(v[1]);
    f2 = v[0].cross(v[2]);
    Vector3f p(x,y,1.);
    if((p.dot(f0)*f0.dot(v[2])>0) && (p.dot(f1)*f1.dot(v[0])>0) && (p.dot(f2)*f2.dot(v[1])>0))
        return true;
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v){
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(std::vector<Triangle *> &TriangleList) {

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (const auto& t:TriangleList)
    {
        Triangle newtri = *t;

        std::array<Eigen::Vector4f, 3> mm {
                (view * model * t->v[0]),
                (view * model * t->v[1]),
                (view * model * t->v[2])
        };

        std::array<Eigen::Vector3f, 3> viewspace_pos;

        std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) {
            // this in fact calls v.head<3>(), a template function
            // but if v.head<3>() may be interpreted as '((v.head<3)>())', a boolean expression
            // thus, to eliminate this ambiguity, C++ introduce the usage of 'template' keyword in such case to 
            // explicitly indicate that, a template function is called here.
            // ! do not regard the following statement as a variable definition with the type of (v.template) and value of head<3>()
            return v. template head<3>();
            // Model and view transformation wont invoke the homogeneous coordinates' variation, 
            // thus normalization in this stage is not needed
        });

        Eigen::Vector4f v[] = {
                mvp * t->v[0],
                mvp * t->v[1],
                mvp * t->v[2]
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec.x()/=vec.w();
            vec.y()/=vec.w();
            vec.z()/=vec.w();
        }

        Eigen::Matrix4f inv_trans = (view * model).inverse().transpose();
        Eigen::Vector4f n[] = {
                inv_trans * to_vec4(t->normal[0], 0.0f),
                inv_trans * to_vec4(t->normal[1], 0.0f),
                inv_trans * to_vec4(t->normal[2], 0.0f)
        };

        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = -vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            //screen space coordinates
            newtri.setVertex(i, v[i]);
        }

        for (int i = 0; i < 3; ++i)
        {
            //view space normal
            newtri.setNormal(i, n[i].head<3>());
        }

        newtri.setColor(0, 148,121.0,92.0);
        newtri.setColor(1, 148,121.0,92.0);
        newtri.setColor(2, 148,121.0,92.0);

        // Also pass view space vertice position
        rasterize_triangle(newtri, viewspace_pos);
    }
}

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight)
{
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}


template<typename T>
inline T barycentric_interp(float alpha, float beta, float gamma, const T& v0, const T& v1, const T& v2){
    return alpha * v0 + beta * v1 + gamma * v2;
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos) 
{
    auto v = t.toVector4();
    
    // Find out the bounding box of current triangle.
    auto top = (int)std::min(std::ceil(std::max(v[0].y(), std::max(v[1].y(), v[2].y()))), (float)height-1);
    auto bottom = (int)std::max(std::floor(std::min(v[0].y(), std::min(v[1].y(), v[2].y()))), 0.0f);
    auto left = (int)std::max(std::floor(std::min(v[0].x(), std::min(v[1].x(), v[2].x()))), 0.0f);
    auto right = (int)std::min(std::ceil(std::max(v[0].x(), std::max(v[1].x(), v[2].x()))), (float)width-1);
    // iterate through the pixel and find if the current pixel is inside the triangle

    for(int x = left; x <= right; x++){
        for(int y = bottom; y <= top; y++){
            //~ with MSAA
            // auto value = Eigen::Vector3f(0,0,0);
            // auto depth = std::numeric_limits<float>::infinity();
            // for(float i = 0.25;i<1.0;i+=0.5){
            //     for(float j = 0.25;j<1.0;j+=0.5){
            //         auto px = x + i;
            //         auto py = y + j;
            //         if(!insideTriangle(px, py, t.v)) continue;

            //         // calculate the barycentric coordinates
            //         auto [alpha, beta, gamma] = computeBarycentric2D(px, py, t.v);

            //         // calculate the interpolated depth(z)
            //         auto w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            //         auto z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            //         z_interpolated *= w_reciprocal;
            //         depth = std::min(z_interpolated, depth);

            //         // calculate the interpolated color
            //         auto colorp = barycentric_interp(alpha,beta,gamma,t.color[0],t.color[1],t.color[2]);

            //         // calculate the interpolated normal
            //         Eigen::Vector3f normalp = barycentric_interp(alpha,beta,gamma,t.normal[0],t.normal[1],t.normal[2]).normalized();

            //         // calculate the interpolated uv coordinates
            //         auto up = barycentric_interp(alpha, beta, gamma, t.tex_coords[0][0], t.tex_coords[1][0], t.tex_coords[2][0]);
            //         auto vp = barycentric_interp(alpha, beta, gamma, t.tex_coords[0][1], t.tex_coords[1][1], t.tex_coords[2][1]);

            //         // calculate the interpolated shading color
            //         // interpolated shading coords
            //         Eigen::Vector3f shading_coordsp = barycentric_interp(alpha, beta, gamma, view_pos[0], view_pos[1], view_pos[2]);
            //         fragment_shader_payload payload(colorp, normalp.normalized(),{up,vp},texture? &*texture : nullptr);
            
            //         auto pixel_color = fragment_shader(payload);

            //         value += pixel_color;
            //     }
            // }
            // value /= 4;
            // if(value.x()+value.y()+value.z()==0) continue;
            // auto index = get_index(x, y);
            // if(depth_buf[index] < depth) continue;
            // depth_buf[index] = depth;
            // set_pixel({x, y}, value);

            //~ without MSAA

            auto index = get_index(x, y);
            // calculate the barycentric coordinates
            if (!insideTriangle(x+0.5, y+0.5, t.v)) continue;
            auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);

            // calculate the interpolated depth(z)
            auto w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            // auto zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            auto zp = barycentric_interp(alpha, beta, gamma, v[0].z()/v[0].w(), v[1].z()/v[1].w(), v[2].z()/v[2].w());
            zp *= w_reciprocal;

            // calculate the interpolated base color
            auto colorp = barycentric_interp(alpha,beta,gamma,t.color[0],t.color[1],t.color[2]);

            // calculate the interpolated normal
            Eigen::Vector3f normalp = barycentric_interp(alpha,beta,gamma,t.normal[0],t.normal[1],t.normal[2]).normalized();

            // calculate the interpolated uv coordinates
            auto up = barycentric_interp(alpha, beta, gamma, t.tex_coords[0][0], t.tex_coords[1][0], t.tex_coords[2][0]);
            auto vp = barycentric_interp(alpha, beta, gamma, t.tex_coords[0][1], t.tex_coords[1][1], t.tex_coords[2][1]);

            // get the final shading color from texture
            // interpolated shading coords
            Eigen::Vector3f shading_coordsp = barycentric_interp(alpha, beta, gamma, view_pos[0], view_pos[1], view_pos[2]);
            fragment_shader_payload payload(colorp, normalp.normalized(),{up,vp},texture? &*texture : nullptr);
            
            // set the pixel color
            auto pixel_color = fragment_shader(payload);

            if (depth_buf[index] < zp) continue;
            depth_buf[index] = zp;
            set_pixel({x, y}, pixel_color);
        }
    }
    // TODO: From your HW3, get the triangle rasterization code.
    // TODO: Inside your rasterization loop:
    //    * v[i].w() is the vertex view space depth value z.
    //    * Z is interpolated view space depth for the current pixel
    //    * zp is depth between zNear and zFar, used for z-buffer

    // float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // zp *= Z;

    // TODO: Interpolate the attributes:
    // auto interpolated_color
    // auto interpolated_normal
    // auto interpolated_texcoords
    // auto interpolated_shadingcoords

    // Use: fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
    // Use: payload.view_pos = interpolated_shadingcoords;
    // Use: Instead of passing the triangle's color directly to the frame buffer, pass the color to the shaders first to get the final color;
    // Use: auto pixel_color = fragment_shader(payload);

 
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

    texture = std::nullopt;
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-y)*width + x;
}

void rst::rasterizer::set_pixel(const Vector2i &point, const Eigen::Vector3f &color)
{
    //old index: auto ind = point.y() + point.x() * width;
    int ind = (height-point.y())*width + point.x();
    frame_buf[ind] = color;
}

void rst::rasterizer::set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader)
{
    vertex_shader = vert_shader;
}

void rst::rasterizer::set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader)
{
    fragment_shader = frag_shader;
}

