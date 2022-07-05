#pragma once

#include <opencv2/features2d.hpp>
#include "common_include.h"

struct Frame;
struct MapPoint;

/**
 * 2D 特征点
 * 在三角化之后会被关联一个地图点
 */
struct Feature {
   public:
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;         // 持有该feature的frame
    //cv::KeyPoint position_;              // 2D提取位置
    cv::Point2f pt_;                       //特征点坐标
    std::weak_ptr<MapPoint> map_point_;  // 关联地图点
    long int age_ = 1;                         // 特征年龄（被观测次数)
    unsigned long id_ = 0;                 //特征ID

    bool is_outlier_ = false;            // 是否为异常点
    bool is_on_left_image_ = true;       // 标识是否提在左图，false为右图

   public:
    Feature() {}
    Feature(std::shared_ptr<Frame> frame, const cv::Point2f &pt)
        : frame_(frame), pt_(pt) {}

    
    // factory function
    static Feature::Ptr CreateNewFeature(std::shared_ptr<Frame> frame, const cv::Point2f &pt);
};

