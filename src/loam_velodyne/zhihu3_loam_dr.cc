
#include <cmath>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

void matrixToEulerYXZ(Eigen::Matrix3d& m, Eigen::Vector3d &out)
{
    //对照WIKIPEDIA的表运算（YXZ外旋）
    double z = atan2(m(1,0), m(1,1));

    double cos_x = m(1,1) / cos(z);
    double x = atan2(-m(1,2), cos_x);

    double y = atan2(m(0,2), m(2,2));

    out[0] = x;
    out[1] = y;
    out[2] = z;
}

void matrixToRtYXZ(Eigen::Matrix4d& matrix, Eigen::Vector3d &euler_r, Eigen::Vector3d &euler_t)
{
    Eigen::Matrix3d matrix_r = matrix.block(0,0,3,3);

    euler_t[0] = matrix(0,3);
    euler_t[1] = matrix(1,3);
    euler_t[2] = matrix(2,3);

    matrixToEulerYXZ(matrix_r, euler_r);
}

void eulerToMatrixYXZ(Eigen::Matrix3d& m, Eigen::Vector3d &euler)
{
    m = Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
}

void rtToMatrixYXZ(Eigen::Vector3d &r, Eigen::Vector3d &t, Eigen::Matrix4d& m)
{
    Eigen::Matrix3d m_r;
    eulerToMatrixYXZ(m_r, r);

    m.block(0,0,3,3) = m_r;
    m(0,3) = t[0];
    m(1,3) = t[1];
    m(2,3) = t[2];

    m(3, 0) = 0;
    m(3, 1) = 0;
    m(3, 2) = 0;
    m(3, 3) = 1;
}

//全局变量

//前一帧的ODOM pose， r是欧拉角，t是平移向量， 请自行维护
Eigen::Vector3d odometry_pre_r_；
Eigen::Vector3d odometry_pre_t_；

//当前的ODOM pose， r是欧拉角，t是平移向量， 请自行维护
Eigen::Vector3d odometry_r_；
Eigen::Vector3d odometry_t_；

//前一帧的pose， r是欧拉角，t是平移向量， 请自行维护
Eigen::Vector3d pose_pre_r_；
Eigen::Vector3d pose_pre_r_；

//基于predict得到transformToOptimize_, 作为优化的初始值
void predictTransformByOdom(Eigen::Vector3d &dst_r, Eigen::Vector3d &dst_t)
{
	Eigen::Matrix4d odom_m1, odom_m2, odom_trans;

    //LOAM中的变换都是欧拉角表示， YXZ顺序外旋的欧拉角

    //将前一帧的ODOM（R， T）转换为4*4变换矩阵形式
	rtToMatrixYXZ(odometry_pre_r_, odometry_pre_t_, odom_m1);
    
    //将当前帧的ODOM（R， T）转换为4*4变换矩阵形式
	rtToMatrixYXZ(odometry_r_, odometry_t_, odom_m2);

    //求两帧之间的变换矩阵， 当前帧变换到前一帧
    //W1 * 12 = W2 ->12 = W1逆*W2
    odom_trans = odom_m1.inverse()*odom_m2;

    //将前一帧的pose（R，T）转换为4*4变换矩阵形式
	Eigen::Matrix4d pose;
	rtToMatrixYXZ(pose_pre_r_, pose_pre_t_, pose);

    //将ODOM得到的变换矩阵作用于前一帧POSE得到当前帧的POSE估计
	//W2 = W1*12
	pose = pose*odom_trans;

    //4*4矩阵格式的POSE转换为（R，T）格式的POSE
	matrixToRtYXZ(pose, dst_r, dst_t);

    //now，dst_r， dst_t就是我们预测后的pose
}