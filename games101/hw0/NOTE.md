# homework0
点的旋转平移在齐次坐标下的旋转和平移：
```cpp
#define _USE_MATH_DEFINES
#include <iostream>
#include <Eigen/Dense> 
#include<cmath>
using namespace std;
using namespace Eigen;

int main()
{
    Vector3d point(2, 1, 1);
    Matrix3d R45;
    double angle = 45.0 * M_PI / 180.0;
    R45 << cos(angle), -sin(angle), 1,
        sin(angle), cos(angle), 2,
        0, 0, 1;

    cout << R45 * point << endl;
    return 0;
}
```
注：  
1. 旋转矩阵的构造：  
   - 旋转角度需要转换为弧度，使用 `angle = 45.0 * M_PI / 180.0` 来实现。  
   - 旋转矩阵 `R45` 的构造按照二维旋转矩阵的形式进行，同时添加了平移部分。
2. 矩阵的构造：  
   - 使用 `<<` 运算符来初始化矩阵，按照行优先的顺序输入元素。
   - d表示double类型，确保计算的精度。
   - f表示float类型，适用于需要节省内存的情况。
   - i表示整数类型，适用于计数等不需要小数的情况。
3.矩阵乘法：  
   - 使用 `R45 * point` 来实现旋转和平移的变换，结果是一个新的点的坐标。
   - 矩阵相乘时，确保矩阵的维度匹配，以及类型的一致性。

