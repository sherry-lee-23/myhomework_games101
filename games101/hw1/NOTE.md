# homework1

## 1. 模型变换矩阵
```cpp
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    float angle = rotation_angle * MY_PI / 180.0;
    model << cos(angle), -sin(angle), 0, 0,
              sin(angle), cos(angle), 0, 0,
              0, 0, 1, 0,
		      0, 0, 0, 1;

    return model;
}
```
步骤：
1. 将旋转角度转换为弧度   float angle = rotation_angle * MY_PI / 180.0;
2. 构造旋转矩阵(绕z轴)

注意：
1. 旋转矩阵的构造需要注意角度的单位，通常需要将角度转换为弧度。
2. 绕y轴旋转时，与绕x或z不同，需要取负




## 2. 透视投影矩阵
```cpp
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
    float zNear, float zFar)
{   
    Eigen::Matrix4f projection = Eigen::Matrix4f::Zero();

    Eigen::Matrix4f m;
    m << -zNear,0, 0, 0,
        0, -zNear, 0, 0,
        0, 0, -(zNear + zFar),  zNear * -zFar,
		0, 0, -1, 0;
     
	float half = (eye_fov / 2.0) * MY_PI / 180.0;
	float top = tan(half) * zNear;
	float bottom = -top;
	float right = aspect_ratio * top;
	float left = -right;

    Eigen::Matrix4f n, p;
    n<< 2 / (right - left), 0, 0, 0,
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
参数：  
eye_fov    张角大小_  
aspect_ratio_   宽高比  
znear    近平面  
zfar     远平面    

步骤：
1.构造m矩阵，进行透视变换，将近平面映射到z=-znear的平面上
2.计算top、bottom、left、right，作为近平面的边界
3.构造n矩阵，进行缩放，使得近平面映射到[-1,1]的范围内
4.构造p矩阵，进行平移，使得近平面中心映射到原点

注意：
1. znear和zfar应该是正数，表示距离摄像机的距离，而不是坐标值。但传入的参数是负数，所以在计算时需要取负值。
2. 透视投影矩阵的构造需要注意坐标系的定义和变换顺序，通常是先进行透视变换，再进行缩放和平移。
3. 透视投影矩阵的构造需要注意角度的单位，通常需要将角度转换为弧度。