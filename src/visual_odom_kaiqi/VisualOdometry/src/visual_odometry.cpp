#include "visual_odometry.h"
#include <chrono>



VisualOdometry::VisualOdometry(){
    initFlag_ = false;
}
VisualOdometry::~VisualOdometry(){
    frontendthread.join();
    printf("join thread \n");
}



bool VisualOdometry::Init() {
    
    // create components and links
    frontend_ = Frontend::Ptr(new Frontend);   
    map_ = MAP::Ptr(new MAP);

    frontend_->SetMap(map_);

    frontendthread = std::thread(std::bind(&VisualOdometry::frontendThread, this));
    initFlag_ = true;
    return true;
}

bool VisualOdometry::frontendThread() {

    while(1){
        if(newFrameFlag_){
            //TicToc featureTrackerTime;
            //printf("featureTracker time: %f\n", featureTrackerTime.toc());

            frontend_->process(new_frame_);

            newFrameFlag_ = false;
        }
        
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
    return 1;
}


void VisualOdometry::Run(double t, const cv::Mat &left_img, const cv::Mat &right_img) {
        
        new_frame_ = Frame::CreateFrame();
        new_frame_->left_img_ = left_img;
        new_frame_->right_img_ = right_img;

        if(initFlag_ == false){
            ROS_INFO("------ VO start ------");
            Init();
        } else {
            newFrameFlag_ = true;
        }
     
        //ROS_ERROR("------ VO exit ------");

    }


    





































