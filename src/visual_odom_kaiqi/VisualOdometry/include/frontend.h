#pragma once

#include "common_include.h"
#include "frame.h"
#include "camera.h"
#include "parameters.h"
#include "feature.h"
#include "algorithm.h"
#include "mappoint.h"
#include "feature_solve.h"
#include "map.h"
#include "Optimizer.h"


//前端状态枚举
enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

/**
 * 前端
 * 估计当前帧Pose，在满足关键帧条件时向地图加入关键帧并触发优化
 */
class Frontend {
   public:
    std::queue<std::shared_ptr<Frame>> keyframe_;
    std::vector<std::shared_ptr<Frame>> allframe_;
   public:
    typedef std::shared_ptr<Frontend> Ptr;
    double iScale = 1;

    Frontend();

    /// Set函数
    void SetMap(MAP::Ptr map) { map_ = map; }

    //前端初始化
    bool frontendInit();

    //前端流程
    bool process(Frame::Ptr new_frame);

    //特征追踪
    int featureTrack(); 

    int DetectFeatures();


    int BuildInitMap(vector<cv::Point3f> points3d_, vector<uchar> trackstatus_, vector<uchar> trianstatus_, float scale_);

    int TrackLastFrame();

    bool InsertKeyframe();

    int CreateNewMapPoints();

    int  RemoveOutlierMappoint(MAP::LandmarksType landmarks);
    //跟踪失败
    bool Reset();

    cv::Mat showBA(Frame::Ptr curFrame, MAP::LandmarksType landmarks);
    cv::Mat showframeReject(Frame::Ptr curFrame);

    std_msgs::Header header;

   private:
    Frame::Ptr current_frame_ = nullptr;  // 当前帧
    Frame::Ptr last_frame_ = nullptr;     // 上一帧

    MAP::Ptr map_ = nullptr;

    FeatureSolver::Ptr featuresolver_;

    FrontendStatus status_ = FrontendStatus::INITING;//前端状态标识，初始为初始化

    SE3 relative_motion_;  // 当前帧与上一帧的相对运动，用于估计当前帧pose初值
    bool init_success_ = false;

    std::vector<Eigen::Vector3d> show_pointCloud;
    
    cv::Mat show_Trackleftimage;
    cv::Mat mshow_Initimage;

    //初始化私有成员变量
    bool mpInitializer = false; //初始化器状态
    Frame::Ptr mInitialFrame_ = nullptr; // 第一帧
    Frame::Ptr mLastFrame = nullptr;
    vector<uchar> mTrackStatus;//追踪匹配点状态向量

    //卡方检测
    double kafang[6] = {0.00, 5.99, 9.49, 12.59, 15.51, 18.31};

};