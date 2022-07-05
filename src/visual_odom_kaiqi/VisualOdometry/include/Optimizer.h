#pragma once

#include "common_include.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "parameters.h"
#include "mappoint.h"
#include "map.h"
#include "frame.h"
#include "camera.h"
#include "map.h"



struct CeresMappoint{
    MapPoint::Ptr ptr;
    double position[3];//3d坐标
    
};

struct CeresFrame{
    Frame::Ptr ptr;
    int common_point = 1;//共视点数量（创建的时候初始为1）
    double camera_r[4];
    double camera_t[3];
    
};


struct CeresPose{
    Frame::Ptr ptr;
    double camera[3];//3d坐标
};


/** @brief 优化器,所有的优化相关的函数都在这个类中; 并且这个类只有成员函数没有成员变量 */
class Optimizer
{
public:
    // 定义ceres 重投影误差 #归一化平面
    struct ReprojectionError {
        ReprojectionError(double observed_x, double observed_y)
            : observed_x(observed_x), observed_y(observed_y) {}
        template <typename T>
        bool operator()(const T* const camera_r, const T* const camera_t,
            const T* const point, T* residuals) const {
            /* Pw --> Pc
            *  camera_r are the angle-axis rotation.
            *  camera_t are the translation.
            */ 
            T p[3];
            // Ceres Quaternion (w, x, y, z)
            ceres::QuaternionRotatePoint(camera_r, point, p);
            p[0] += camera_t[0]; p[1] += camera_t[1]; p[2] += camera_t[2];

            // Camera intrinsics
            T xp = p[0]/p[2];
            T yp = p[1]/p[2];

            // 重投影误差 
            residuals[0] = xp - T(observed_x);
            residuals[1] = yp - T(observed_y);
//   cout << "Reprojection : "<< observed_x  << " " << observed_y   << " xp : " << xp << " "<< yp << " residuals[0] : "<< residuals[0] <<" residuals[1] : "<< residuals[1] <<endl; 
            return true;
        }
        
        //工厂模式构建(可选)：避免每次重复创建实例和析构实例
        static ceres::CostFunction* Create(const double observed_x, 
                                           const double observed_y) {
            /* @param : 仿函数（functor）类型, 残差维数, 参数1维数、参数2维数...; 接受参数类型为仿函数指针CostFunctor* */
            return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3, 3>(
                new ReprojectionError(observed_x, observed_y)));
        }
        double observed_x;
        double observed_y;
    };

    /**
     * @brief bundle adjustment Optimization
     * 
     * 3D-2D 最小化重投影误差 e = (u,v) - project(Tcw*Pw) \n
     * 
     *         
     * @param   vpKFs    关键帧 
     *          vpMP     MapPoints
     *          nIterations 迭代次数（20次）
     *          pbStopFlag  是否强制暂停
     *          nLoopKF  关键帧的个数 -- 但是我觉得形成了闭环关系的当前关键帧的id
     *          bRobust  是否使用核函数
     */
    void BundleAdjustment(MAP::KeyframesType &key1frames,
                                 MAP::LandmarksType &landmarks, 
                                 int nIterations);

    /**
     * @brief 进行全局BA优化，但主要功能还是调用 BundleAdjustment,这个函数相当于加了一个壳.
     * @param[in] pMap          地图对象的指针
     * @param[in] nIterations   迭代次数
     * @param[in] pbStopFlag    外界给的控制GBA停止的标志位
     * @param[in] nLoopKF       当前回环关键帧的id，其实也就是参与GBA的关键帧个数
     * @param[in] bRobust       是否使用鲁棒核函数
     */
    void  GlobalBundleAdjustemnt(int nIterations);

    static bool LocalBA(Frame::Ptr currectFrame,Frame::Ptr initFrame, 
                       MAP::KeyframesType &keyframes,
                       MAP::LandmarksType &landmarks,
                       int nIterations);
    static bool GlobalBA(Frame::Ptr currectFrame,Frame::Ptr initFrame, 
                       MAP::KeyframesType &keyframes,
                       MAP::LandmarksType &landmarks,
                       int nIterations);

    //单目初始化BA优化
    static bool InitBA(Frame::Ptr firstFrame, 
                    Frame::Ptr secondFrame, 
                    MAP::LandmarksType &landmarks,
                    int nIterations);


private:


};



