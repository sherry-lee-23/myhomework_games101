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