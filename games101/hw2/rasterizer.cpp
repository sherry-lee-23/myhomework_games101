// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include <map>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include<Eigen/Dense>


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

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f p1(static_cast<float>(x), static_cast<float>(y), 1.0f);
    float n[3] = {0};
    for (int i = 0; i < 3; i++) {
        Vector3f p0 = _v[i];
        Vector3f p2 = _v[(i + 1) % 3];
        n[i] = (p2 - p1).cross(p0 - p1).z();
    }
    if (n[0] <= 0 && n[1] <= 0 && n[2] <= 0) return true;
	if (n[0] >= 0 && n[1] >= 0 && n[2] >= 0) return true;
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }

    // downsampling
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            Eigen::Vector3f color = { 0., 0., 0. };
            float infinity = std::numeric_limits<float>::infinity();
            for (float j = start_point; j < 1.0; j += pixel_size_sm) {
                for (float i = start_point; i < 1.0; i += pixel_size_sm) {
                    int index = get_index_ssaa(x, y, i, j);
                    color += frame_buf_ssaa[index];
                }
            }
            Eigen::Vector3f p;
            p << x, y, 0.;
            set_pixel(p, color / (ssaa_h * ssaa_w));
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    float min_x = width;
	float max_x = 0;
	float min_y = height;
	float max_y = 0;

    // find out the bounding box of current triangle
    for (int i = 0; i < 3; i++) {
		min_x = std::min(min_x, v[i].x());
        max_x = std::max(max_x, v[i].x());
        min_y = std::min(min_y, v[i].y());
        max_y = std::max(max_y, v[i].y());
    }

    // iterate through the pixel
    for (int y = min_y; y < max_y; y++) {
        for (int x = min_x; x < max_x; x++) {
			//iterate through the sampling point
            for (float j = start_point; j < 1.0; j += pixel_size_sm) {
                for (float i = start_point; i < 1.0; i += pixel_size_sm) {
                    //find if the current point is inside the triangle
                    if (insideTriangle(x + i, y + j, t.v)) {
                        //if so, use the following code to get the interpolated 
                        auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        //set the current pixel(use the set_pixel function)
                        //to the color of the triangle (use getColor function)
                        //if it should be painted
                        int index = get_index_ssaa(x, y, i, j);
                        if (z_interpolated < depth_buf_ssaa[index]) {
                            frame_buf_ssaa[index] = t.getColor();
                            depth_buf_ssaa[index] = z_interpolated;
                        }
                    }
                }
            }
        }
    }

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
		std::fill(frame_buf_ssaa.begin(), frame_buf_ssaa.end(), Eigen::Vector3f{ 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
		std::fill(depth_buf_ssaa.begin(), depth_buf_ssaa.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
	frame_buf_ssaa.resize(w * h * ssaa_h * ssaa_w);
	depth_buf_ssaa.resize(w * h * ssaa_h * ssaa_w);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}


//for float args
int rst::rasterizer::get_index_ssaa(int x, int y, float i, float j) {
	int ssaa_height = height * ssaa_h;
	int ssaa_width = width * ssaa_w;
    
    i = int((i - start_point) / pixel_size_sm);
	j = int((j - start_point) / pixel_size_sm);

	return (ssaa_height - 1 - (y * ssaa_h + j)) * ssaa_width + x * ssaa_w + i;

}