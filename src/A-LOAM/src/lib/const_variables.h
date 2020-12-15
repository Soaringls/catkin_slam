#ifndef _CONST_VARIABLES_H_
#define _CONST_VARIABLES_H_
#include <string.h>

#include <cmath>
#include <string>

#define PI 3.14159265

using namespace std;

// Using velodyne cloud "ring" channel for image projection (other lidar may
// have different name for this channel, change "PointXYZIR" below)
extern const bool useCloudRing =
    false;  // if true, ang_res_y and ang_bottom are not used

// VLP-16
extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0 + 0.1;
extern const int groundScanInd = 7;

// HDL-32E
// extern const int N_SCAN = 32;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 41.33/float(N_SCAN-1);
// extern const float ang_bottom = 30.67;
// extern const int groundScanInd = 20;

// VLS-128
// extern const int N_SCAN = 128;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 0.2;
// extern const float ang_res_y = 0.3;
// extern const float ang_bottom = 25.0;
// extern const int groundScanInd = 10;

// Ouster users may need to uncomment line 159 in imageProjection.cpp
// Usage of Ouster imu data is not fully supported yet (LeGO-LOAM needs 9-DOF
// IMU), please just publish point cloud data Ouster OS1-16 extern const int
// N_SCAN = 16; extern const int Horizon_SCAN = 1024; extern const float
// ang_res_x = 360.0/float(Horizon_SCAN); extern const float ang_res_y
// = 33.2/float(N_SCAN-1); extern const float ang_bottom = 16.6+0.1; extern
// const int groundScanInd = 7;

// Ouster OS1-64
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 15;

extern const bool loopClosureEnableFlag = false;
extern const double mappingProcessInterval = 0.3;

extern const float scanPeriod = 0.1;
extern const int systemDelay = 0;
extern const int imuQueLength = 200;

extern const float sensorMinimumRange = 1.0;
extern const float sensorMountAngle = 0.0;
extern const float segmentTheta =
    60.0 / 180.0 * M_PI;  // decrese this value may improve accuracy
extern const int segmentValidPointNum = 5;
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;

extern const int edgeFeatureNum = 2;
extern const int surfFeatureNum = 4;
extern const int sectionsTotal = 6;
extern const float edgeThreshold = 0.1;
extern const float surfThreshold = 0.1;
extern const float nearestFeatureSearchSqDist = 25;

// Mapping Params
extern const float surroundingKeyframeSearchRadius =
    50.0;  // key frame that is within n meters from current pose will be
           // considerd for scan-to-map optimization (when loop closure
           // disabled)
extern const int surroundingKeyframeSearchNum =
    50;  // submap size (when loop closure enabled)
// history key frames (history submap for loop closure)
extern const float historyKeyframeSearchRadius =
    7.0;  // key frame that is within n meters from current pose will be
          // considerd for loop closure
extern const int historyKeyframeSearchNum =
    25;  // 2n+1 number of hostory key frames will be fused into a submap for
         // loop closure
extern const float historyKeyframeFitnessScore =
    0.3;  // the smaller the better alignment

extern const float globalMapVisualizationSearchRadius =
    500.0;  // key frames with in n meters will be visualized

#endif