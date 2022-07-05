#pragma once

#include "common_include.h"




// Pinhole stereo camera model
class Camera {
   public:
    typedef std::shared_ptr<Camera> Ptr;

    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0,
           baseline_ = 0;  // Camera intrinsics
    SE3 T_rect_;             // extrinsic, from stereo camera to single camera
    SE3 T_rect_inv_;         // inverse of extrinsics
    SO3 R_rect_;            //相机相对于相机0坐标系的R
    SO3 R_rect_inv_;
    Vec3 t_rect_;           //相机相对于相机0坐标系的t

    SE3 T_l_c_;
    SE3 T_v_l_;

    Camera();
    Camera(double fx, double fy, double cx, double cy,
           const SO3 &R_rect, const Vec3 t_rect, const SE3 T_l_c)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), R_rect_(R_rect), t_rect_(t_rect), T_l_c_(T_l_c.inverse()) {
        R_rect_inv_ = R_rect_.inverse();
        T_rect_ = SE3(R_rect_, t_rect_);
        T_rect_inv_ = T_rect_.inverse();
        
    }

    SE3 pose() const { return T_rect_; }

    // return intrinsic matrix
    Mat33 K() const {
        Mat33 k;
        k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        return k;
    }

    // coordinate transform: world, camera, pixel
    Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);

    Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);

    Vec2 camera2pixel(const Vec3 &p_c);

    Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);

    Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);

    Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w);

    Vec3 camera2lidar(const Vec3 &p_c);

};


