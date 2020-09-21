#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>

using namespace cv;
using namespace std;

void getQuaternion(Mat R, float Q[])
{
    // index: w, x, y, z
    float trace = R.at<float>(0,0) + R.at<float>(1,1) + R.at<float>(2,2);
 
    if (trace > 0.0) 
    {
        float s = sqrt(trace + 1.0);
        Q[3] = (s * 0.5);
        s = 0.5 / s;
        Q[0] = ((R.at<float>(2,1) - R.at<float>(1,2)) * s);
        Q[1] = ((R.at<float>(0,2) - R.at<float>(2,0)) * s);
        Q[2] = ((R.at<float>(1,0) - R.at<float>(0,1)) * s);
    } 
    
    else 
    {
        int i = R.at<float>(0,0) < R.at<float>(1,1) ? (R.at<float>(1,1) < R.at<float>(2,2) ? 2 : 1) : (R.at<float>(0,0) < R.at<float>(2,2) ? 2 : 0); 
        int j = (i + 1) % 3;  
        int k = (i + 2) % 3;

        float s = sqrt(R.at<float>(i, i) - R.at<float>(j,j) - R.at<float>(k,k) + 1.0);
        Q[i] = s * 0.5;
        s = 0.5 / s;

        Q[3] = (R.at<float>(k,j) - R.at<float>(j,k)) * s;
        Q[j] = (R.at<float>(j,i) + R.at<float>(i,j)) * s;
        Q[k] = (R.at<float>(k,i) + R.at<float>(i,k)) * s;
    }
}


 void ToEulerAngles(float q[], vector<float> &euler) {
    // euler: x, y, z

    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q[3] * q[0] + q[1] * q[2]);
    float cosr_cosp = 1 - 2 * (q[0] * q[0] + q[1] * q[1]);
    euler[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2 * (q[3] * q[1] - q[2] * q[0]);
    if (std::abs(sinp) >= 1)
        euler[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1]);
    float cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
    euler[2] = std::atan2(siny_cosp, cosy_cosp);
}

void RotMat2Euler(Mat R, vector<float> &euler) {
    float q[4];
    getQuaternion(R, q);
    ToEulerAngles(q, euler);
}


// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
    
    return  norm(I, shouldBeIdentity) < 1e-6;
    
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{
    // euler order: y->x->z

    assert(isRotationMatrix(R));
    
    float sy = sqrt(R.at<float>(0,0) * R.at<float>(0,0) +  R.at<float>(1,0) * R.at<float>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if(R.at<float>(2,1) < 1.0) {
        if(R.at<float>(2,1) > -1.0) {
            x = asin(R.at<float>(2,1));
            z = atan2(-R.at<float>(0,1), R.at<float>(1,1));
            y = atan2(-R.at<float>(2,0), R.at<float>(2,2));
        }
        else {
            x = -3.14159/2.0;
            z = -atan2(R.at<float>(0,2), R.at<float>(0,0));
            y = 0;
        }
    }
    else {
        x = 3.14159/2.0;
        z = atan2(R.at<float>(0,2), R.at<float>(0,0));
        y = 0;
    }

    return Vec3f(x, y, z);
    
    
    
}

Mat rotMat(int axis, float rad) {
    // axis: 
    // x = 1, y = 2, z = 3

    Mat rot;
    if(axis == 1) {
        rot=(cv::Mat_<float>(3,3)<<1,0,0,
                                   0,cos(rad),-sin(rad),
                                   0,sin(rad),cos(rad));
    }
    else if(axis == 2) {
        rot=(cv::Mat_<float>(3,3)<<cos(rad),0,sin(rad),
                                   0,1,0,
                                   -sin(rad),0,cos(rad));
    }
    else if(axis == 3) {
        rot=(cv::Mat_<float>(3,3)<<cos(rad),-sin(rad),0,
                                   sin(rad),cos(rad),0,
                                   0,0,1);
    }
    else {
        cout << "wrong axis input!" << endl;
        return rot;
    }

    return rot;
}