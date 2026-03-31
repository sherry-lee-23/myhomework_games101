# homework2

## 1.透视投影矩阵
```cpp
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Zero();

    Eigen::Matrix4f m;
    m << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, (zNear + zFar), -zNear * zFar,
        0, 0, -1, 0;

    float half = (eye_fov / 2.0) * MY_PI / 180.0;
    float top = tan(half) * zNear;
    float bottom = -top;
    float right = aspect_ratio * top;
    float left = -right;

    Eigen::Matrix4f n, p;
    n << 2 / (right - left), 0, 0, 0,
        0, 2 / (top - bottom), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;
    p << 1, 0, 0, -(right + left) / 2.0,
        0, 1, 0, -(top + bottom) / 2.0,
        0, 0, 1, -(zNear + zFar) / 2.0,
        0, 0, 0, 1;

    projection = n * p * m;


    return projection;
}
```
步骤：
1. 构造透视投影矩阵m
2. 计算视锥体的边界值top、bottom、left、right
3. 构造缩放矩阵n和位移矩阵p，将视锥体映射到标准立方体内
4. 最终的投影矩阵为n * p * m

注(与作业1的差别)：  
z轴上的变换与作业1不同，作业1中z轴的变换为-(zNear + zFar)，而在作业2中为(zNear + zFar)，使图像正向

## 2.判断一个点是否在三角形内部
```cpp
static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
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
```
步骤：
1. 将点p转换为齐次坐标(x,y,1)
2. 提取向量
3. 计算叉积，判断点p与三角形的三个边的关系

注：
1. 注意三角形的定义，_v[0],_v[1],_v[2]分别为三角形的三个顶点
2. 叉乘时，三边与点p的关系可以通过判断叉积的符号来确定，如果三个叉积的符号相同，则点p在三角形内部，否则在外部
3. 注意三条边向量方向的一致性

## 3.光栅化深度测试和颜色插值
光栅化是将几何图形转换为像素的过程。在本作业中，我们需要实现一个简单的光栅化器，将三角形转换为像素，并进行深度测试和颜色插值。
```cpp
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    float min_x = width;
	float max_x = 0;
	float min_y = height;
	float max_y = 0;

    for (int i = 0; i < 3; i++) {
		min_x = std::min(min_x, v[i].x());
        max_x = std::max(max_x, v[i].x());
        min_y = std::min(min_y, v[i].y());
        max_y = std::max(max_y, v[i].y());
    }

    for (int y = min_y; y < max_y; y++) {
        for (int x = min_x; x < max_x; x++) {
            if (insideTriangle(x + 0.5, y + 0.5, t.v)) {
                auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                
				int index = get_index(x, y);
                if(z_interpolated < depth_buf[index]){
                    Eigen::Vector3f p;
					p << x, y, z_interpolated;
                    set_pixel(p, t.getColor());
					depth_buf[index] = z_interpolated;
				}
            }
        }
    }

}
```
步骤：
1. 计算三角形的边界框，确定需要遍历的像素范围 bounding box  
   遍历三个顶点，找到x，y的最小值和最大值，作为边界框的范围
2. 遍历边界框内的每个像素，判断像素中心是否在三角形内部(x + 0.5,y + 0.5, t.v)
3. 如果在三角形内部，计算该像素的重心坐标(alpha, beta, gamma)，并进行深度插值
4. 进行深度测试，如果当前像素的深度值小于缓冲区中的值，则更新像素颜色和深度缓冲区

注：
1. 判断像素中心是否在三角形内部时，使用(x + 0.5, y + 0.5)来表示像素中心的坐标
2. 深度插值时，需要考虑齐次坐标的影响，因此需要先计算重心坐标的倒数，然后再进行插值


# Triangle and Rasterizer
## Triangle

### 1. Triangle Class
1. v[3]: 三角形的三个顶点坐标
2. color[3]: 三角形三个顶点的颜色
3. normal[3]: 三角形三个顶点的法线向量
4. tex_coords[3]: 三角形三个顶点的纹理坐标

### 2.方法
1. setVertex(int ind,Vector3f ver) : 设置三角形第ind个顶点的坐标
2. setNormal(int ind,Vector3f n) : 设置三角形第ind个顶点的法线向量
3. setColor(int ind,Vector3f c) : 设置三角形第ind个顶点的颜色
4. getColor() : 获取三角形的颜色（取第一个顶点的颜色）
5. setTexCoord(int ind, Vector2f tex) : 设置三角形第ind个顶点的纹理坐标
6. toVector4() : 将三角形的顶点坐标转换为齐次坐标（Vector4f）

## Rasterizer
### 1. namespace rst
1. enum class Buffers { Color = 1, Depth = 2 }; : 定义缓冲区类型，颜色缓冲区和深度缓冲区
2. inline Buffers operator|(Buffers a, Buffers b) : 定义位运算符，用于组合缓冲区类型
3. inline Buffers operator&(Buffers a, Buffers b) : 定义位运算符，用于判断缓冲区类型
4. enum class Primitive { Line, Triangle }; : 定义图元类型，线段和三角形
5. struct pos_buf_id, ind_buf_id, col_buf_id : 定义位置缓冲区ID、索引缓冲区ID和颜色缓冲区ID的结构体 
6. class rasterizer : 定义光栅化器类
   - width, height: 光栅化器的宽度和高度
   - frame_buf: 颜色缓冲区，存储每个像素的颜色值
   - depth_buf: 深度缓冲区，存储每个像素的深度值
   - set_pixel(const Vector3f& point, const Eigen::Vector3f& color) : 设置像素点的颜色
   - clear(Buffers buff) : 清除指定类型的缓冲区
   - get_index(int x, int y) : 获取像素点在缓冲区中的索引
   - rasterize_triangle(const Triangle& t) : 光栅化三角形的方法，包含边界框计算、像素遍历、深度测试和颜色插值等步骤

### class rasterizer
1. model, view, projection: 模型矩阵、视图矩阵和投影矩阵
2. map: pos_buf, ind_buf, col_buf: 存储位置缓冲区、索引缓冲区和颜色缓冲区的映射
3. frame_buf: 颜色缓冲区，存储每个像素的颜色值
4. depth_buf: 深度缓冲区，存储每个像素的深度值
5. width, height: 光栅化器的宽度和高度
6. next_id: 下一个缓冲区ID，用于分配新的缓冲区ID
7. frame_buf_ssaa: 超采样抗锯齿的颜色缓冲区
8. depth_buf_ssaa: 超采样抗锯齿的深度缓冲区
9. ssaa_w,ssaa_h: 超采样抗锯齿的宽度和高度
10. pixel_size_sm: 超采样抗锯齿的像素大小
11. start_point: 超采样抗锯齿的起始点坐标

### 方法
1. rasterizer(int w, int h) : 构造函数，初始化光栅化器的宽度和高度，并分配颜色缓冲区和深度缓冲区
2. loader：加载位置缓冲区、索引缓冲区和颜色缓冲区的方法，返回对应的缓冲区ID
3. setter：设置模型矩阵、视图矩阵和投影矩阵的方法
4. set_pixel(const Vector3f& point, const Eigen::Vector3f& color) : 设置像素点的颜色
5. clear(Buffers buff) : 清除指定类型的缓冲区
6. draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type) : 绘制图元的方法，根据图元类型调用相应的绘制函数
7. getter: index, index_ssaa, next_id 得到index和下一个缓冲区ID的方法
8. rasterize_triangle(const Triangle& t) : 光栅化三角形的方法，包含边界框计算、像素遍历、深度测试和颜色插值等步骤
9. draw_line(Vector3f begin, Vector3f end) : 绘制线段的方法，使用Bresenham算法实现
