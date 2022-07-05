#include "parameters.h"

int MAX_CNT;
int MIN_DIST;

int IMAGE_ROW;
int IMAGE_COL;

int SCALE_S;

Camera::Ptr camera_left_ = nullptr;
Camera::Ptr camera_right_ = nullptr;



void setParameter(std::string config_file){
 
    // read from params file
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened()){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK(); 
    }
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    IMAGE_COL = fsSettings["image_width"];
    IMAGE_ROW = fsSettings["image_height"];
    SCALE_S = fsSettings["Scale"];

    //设置相机参数
    cv::Mat P,R;
    Mat34 eigen_P;
    Mat33 eigen_R;
    Mat33 K;
    Vec3 t;
    fsSettings["P_rect_00"] >> P;//相机0
    cv::cv2eigen(P, eigen_P);
    K << eigen_P(0, 0), eigen_P(0, 1), eigen_P(0, 2),
         eigen_P(1, 0), eigen_P(1, 1), eigen_P(1, 2),
         eigen_P(2, 0), eigen_P(2, 1), eigen_P(2, 2);
    t << eigen_P(0, 3), eigen_P(1, 3), eigen_P(2, 3);
    t = K.inverse() * t;   //KT=P，T=K^-1*P 
    fsSettings["R_rect_00"] >> R;
    cv::cv2eigen(R, eigen_R);
    Eigen::Quaterniond q1(eigen_R.inverse());
    Sophus::SO3d SO3_R0(q1);

    cv::Mat l_T_c;
    Mat44 eigen_l_T_c;
    Mat33 R_l_c;
    Vec3 t_l_c;
    fsSettings["velo_T_cam0"] >> l_T_c;//camera to lidar T
    cv::cv2eigen(l_T_c, eigen_l_T_c);
    R_l_c << eigen_l_T_c(0, 0), eigen_l_T_c(0, 1), eigen_l_T_c(0, 2),
             eigen_l_T_c(1, 0), eigen_l_T_c(1, 1), eigen_l_T_c(1, 2),
             eigen_l_T_c(2, 0), eigen_l_T_c(2, 1), eigen_l_T_c(2, 2);
      
    t_l_c << eigen_l_T_c(0, 3), eigen_l_T_c(1, 3), eigen_l_T_c(2, 3);
    Eigen::Quaterniond q11(R_l_c);
    q11 = q11.normalized();
    Sophus::SO3d SO3_R11(q11);

    Camera::Ptr camera0(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                   SO3(), t, SE3(SO3_R11, t_l_c)));
                                   
    camera_left_ = camera0;

    fsSettings["P_rect_01"] >> P;//相机1
    cv::cv2eigen(P, eigen_P);
    K << eigen_P(0, 0), eigen_P(0, 1), eigen_P(0, 2),
         eigen_P(1, 0), eigen_P(1, 1), eigen_P(1, 2),
         eigen_P(2, 0), eigen_P(2, 1), eigen_P(2, 2);
    t << eigen_P(0, 3), eigen_P(1, 3), eigen_P(2, 3);
    t = K.inverse() * t;   //KT=P，T=K^-1*P 
    fsSettings["R_rect_01"] >> R;
    cv::cv2eigen(R, eigen_R);
    Eigen::Quaterniond q2(eigen_R.inverse());
    q2 = q2.normalized();
    Sophus::SO3d SO3_R1(q2);

    Camera::Ptr camera1(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                   SO3(), t, SE3(Mat33::Identity(), Vec3::Zero())));
    camera_right_ = camera1;

    cout << "-------camera parameters--------: "<< endl;
    cout << "camera_left_: " << "fx:" << camera_left_->fx_ << " fy: " << camera_left_->fy_  << " cx_: " << camera_left_->cx_ << " cy_: " << camera_left_->cy_ << endl;
    cout << "T_00_rect: " << endl;
    cout << camera_left_->T_rect_.matrix() << endl;

    cout << "camera_right_: "<< endl;
    cout << "fx: " << camera_right_->fx_ << " fy: " << camera_right_->fy_  << " cx_: " << camera_right_->cx_ << " cy_: " << camera_right_->cy_ << endl;
    cout << "T_00_rect: " << endl;
    cout << camera_right_->T_rect_.matrix() << endl;

    // LOG(INFO) << "Camera " << "1" << " extrinsics: " << R;
    // LOG(INFO) << "Camera " << "1" << " extrinsics: " << t.transpose();
    ROS_INFO("setParameters success.");
}

