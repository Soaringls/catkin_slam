#pragma once

#include "feature_solve.h"
#include "common_include.h"
#include "parameters.h"
#include "camera.h"
#include "algorithm.h"


class FeatureSolver
{
   public:
    typedef std::shared_ptr<FeatureSolver> Ptr;
    FeatureSolver();



bool solvePoseByPnP(vector<cv::Point2f> &pts2D,
                    vector<cv::Point3f> &pts3D, Mat33 &R, Vec3 &P,
                    cv::Mat &status);

void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);

// 三角化
double checkTriangulation(const vector<cv::Point2f> &l,
                          const vector<cv::Point2f> &r,
                          vector<cv::Point3f> &points3d,
                          vector<uchar> &status,
                          cv::Mat_<double> R, cv::Mat_<double> t);

double checkTriangulation2(const vector<cv::Point2f> &l,
                          const vector<cv::Point2f> &r,
                          vector<cv::Point3f> &points3d,
                          vector<uchar> &status,
                          cv::Mat_<double> R, cv::Mat_<double> t);

// 计算深度中位数
float ComputeSceneMedianDepth(const vector<cv::Point3f> points3d, 
                                                const vector<uchar> status, 
                                                    const int q);






};
