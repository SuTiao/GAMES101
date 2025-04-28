// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

static const int sampleTimes = 1;
static const int totalSamplePoint = sampleTimes * sampleTimes;

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


static bool _insideTriangle(Vector3f P, const Vector3f* _v)
{   
    
    Vector3f AB = _v[1] - _v[0];
    Vector3f AP = P - _v[0];
    auto across1 = AB.cross(AP);

    Vector3f BC = _v[2] - _v[1];
    Vector3f BP = P - _v[1];
    auto across2 = BC.cross(BP);

    Vector3f CA = _v[0] - _v[2];
    Vector3f CP = P - _v[2];
    auto across3 = CA.cross(CP);

    if(( across1.z() > 0 && across2.z() > 0 && across3.z() > 0 ) ||
     ( across1.z() < 0 && across2.z() < 0 && across3.z() < 0 ))
        return true;
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
}

static float insideTriangle(float x,float y,const Vector3f* _v){
    Vector3f P = Eigen::Vector3f(x, y, 1.0f);
    
    float step = 1.0f / sampleTimes * 0.5f;
    float lefttopx = P.x() - 0.5f;
    float lefttopy = P.y() + 0.5f;

    float insideTrianglePoints = 0;

    for(int i = 1;i <= sampleTimes;i++){
        for(int j = 1;j <= sampleTimes;j++){
            Vector3f SamplePoint = Eigen::Vector3f(lefttopx + step * i, lefttopy - step * j, 1.0f);
            if(_insideTriangle(SamplePoint,_v))
                insideTrianglePoints++;
        }
    }
    return insideTrianglePoints == 0 ? 0 : insideTrianglePoints / totalSamplePoint;
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    float minx = FLT_MAX;
    float maxx = FLT_MIN;
    float miny = FLT_MAX;
    float maxy = FLT_MIN;
    for(const auto & vert : v)
    {
        minx = std::min(minx, vert.x());
        maxx = std::max(maxx, vert.x());
        miny = std::min(miny, vert.y());
        maxy = std::max(maxy, vert.y());
    }

    for(int x = floor(minx); x < ceil(maxx);++x){
        for(int y = floor(miny); y < ceil(maxy);++y){
            float grid_width = 1.0f / sampleTimes;
            float grid_p_offset = grid_width * 0.5f;
            float lefttopx = x - 0.5f;
            float lefttopy = y - 0.5f;
            bool should_set_color = false;
            for(int i = 0;i < sampleTimes;i++){
                for(int j = 0;j < sampleTimes;j++){
                    float samplex = lefttopx + grid_width * i + grid_p_offset;
                    float sampley = lefttopy + grid_width * j + grid_p_offset;
                    if(insideTriangle(samplex, sampley, t.v)){
                        auto [alpha, beta, gamma] = computeBarycentric2D(samplex, sampley, t.v);
                        float w_reciprocal = 1.0f / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        int sample_ind = get_index(x, y) + i * sampleTimes + j;
                        float sample_z = sample_depth_buf[sample_ind];
                        if(z_interpolated < sample_z){
                            sample_depth_buf[sample_ind] = z_interpolated;
                            sample_frame_buf[sample_ind] = t.getColor() / totalSamplePoint;
                            should_set_color = true;
                        }
                    }
                }
            }
            int ind = get_index(x, y);
            Vector3f final_color = {0, 0, 0};
            for(int i = ind; i < ind + totalSamplePoint; i++){
                final_color += sample_frame_buf[i];
            }
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point, final_color);
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
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
    if ((buff & rst::Buffers::SampleColor) == rst::Buffers::SampleColor)
    {
        std::fill(sample_frame_buf.begin(), sample_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::SampleDepth) == rst::Buffers::SampleDepth)
    {
        std::fill(sample_depth_buf.begin(), sample_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    sample_frame_buf.resize(w * h * totalSamplePoint);
    sample_depth_buf.resize(w * h * totalSamplePoint);
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




// clang-format on