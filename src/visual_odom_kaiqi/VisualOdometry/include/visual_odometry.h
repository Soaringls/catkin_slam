#pragma once


#include "common_include.h"
#include "frontend.h"
#include "camera.h"



/**
 * VO 对外接口
 */
class VisualOdometry {
    public:

        std::thread frontendthread;


    public:
        VisualOdometry();
        ~VisualOdometry();
        
        bool Init();
        void Run(double t, const cv::Mat &left_img, const cv::Mat &right_img);
        bool frontendThread();

    private:
        bool initFlag_ = false;
        bool newFrameFlag_ = false;

        Frame::Ptr new_frame_;

        Frontend::Ptr frontend_ = nullptr;
        MAP::Ptr map_ = nullptr;

};





