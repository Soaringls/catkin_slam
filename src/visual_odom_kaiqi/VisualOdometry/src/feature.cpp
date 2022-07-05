  #include "feature.h"


std::shared_ptr<Feature> Feature::CreateNewFeature(
                                std::shared_ptr<Frame> frame, 
                                const cv::Point2f &pt) {
    static unsigned long factory_id = 0;
    std::shared_ptr<Feature> new_feature(new Feature(frame, pt));
    new_feature->id_ = factory_id++;
    return new_feature;
}