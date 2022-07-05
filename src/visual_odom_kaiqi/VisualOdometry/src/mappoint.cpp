#include "mappoint.h"



MapPoint::MapPoint(long id, Vec3 position) : id_(id), pos_(position) {}

std::shared_ptr<MapPoint> MapPoint::CreateNewMappoint() {
    static unsigned long factory_id = 0;
    std::shared_ptr<MapPoint> new_mappoint(new MapPoint);
    new_mappoint->id_ = factory_id++;
    return new_mappoint;
}

void MapPoint::RemoveObservation(std::shared_ptr<Feature> feat) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    for (auto iter = observations_.begin(); iter != observations_.end();
         iter++) {
        if (iter->lock() == feat) {
            observations_.erase(iter);
            feat->map_point_.reset(); //reset 指向新对象，不填为空
            observed_times_--;
            break;
        }
    }
}


