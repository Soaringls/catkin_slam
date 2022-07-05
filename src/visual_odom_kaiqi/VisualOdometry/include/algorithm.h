#pragma once

#include "common_include.h"
#include "parameters.h"


/**
 * linear triangulation with SVD
 * @param poses     poses,
 * @param points    points in normalized plane
 * @param pt_world  triangulated point in the world
 * @return true if success
 */
inline bool triangulation(const std::vector<SE3> &poses,
                   const std::vector<Vec3> points, Vec3 &pt_world) {
    MatXX A(2 * poses.size(), 4);
    VecX b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); ++i) {
        Mat34 m = poses[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        // 解质量不好，放弃
        return false;
    }
    return true;
}


/**
 * remove Border
 * @param pt feature u,v
 */
inline bool inBorder(const cv::Point2f &pt){
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);//cvRound返回跟参数最接近的整数值，即四舍五入；
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < IMAGE_COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < IMAGE_ROW - BORDER_SIZE;
}

inline double distance(cv::Point2f pt1, cv::Point2f pt2){
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}



inline void reduceVector(vector<cv::Point2f> &v, vector<uchar> status){
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

inline void reduceVector(vector<cv::Point3f> &v, vector<uchar> status){
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}



















