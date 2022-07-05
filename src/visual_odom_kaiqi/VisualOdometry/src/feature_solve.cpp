#include "feature_solve.h"


FeatureSolver::FeatureSolver(){ 
}


// 求解新图像的位姿
bool FeatureSolver::solvePoseByPnP(vector<cv::Point2f> &pts2D,
                                   vector<cv::Point3f> &pts3D, Mat33 &R, Vec3 &P, cv::Mat &status) {
    if (int(pts2D.size()) < 4){
        ROS_WARN("feature tracking not enough, Can't run PnP \n");
        return false;
    }

    Eigen::Matrix3d R_initial = R;
    Eigen::Vector3d P_initial = P;

    cv::Mat r, rvec, t, K, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    Mat33 eigen_K = camera_left_->K();
    cv::eigen2cv(eigen_K, K);

    /** 
    *bool cv::solvePnPRansac(    求解 pnp问题 >= 4对点
    *   InputArray  objectPoints,   特征点的3D坐标数组
    *   InputArray  imagePoints,    特征点对应的图像坐标
    *   InputArray  cameraMatrix,   相机内参矩阵
    *   InputArray  distCoeffs,     失真系数的输入向量
    *   OutputArray     rvec,       旋转向量
    *   OutputArray     tvec,       平移向量
    *   bool    useExtrinsicGuess = false, 为真则使用提供的初始估计值
    *   int     flags = SOLVEPNP_ITERATIVE 采用LM优化
    *   int iterationsCount = 100   迭代次数
    *   float reprojectionError     重投影误差
    *   double confidence = 0.99    置信度
    *   inliers – Output vector that contains indices of inliers in objectPoints and imagePoints .
    *)   
    */
    bool pnp_succ;
    pnp_succ = cv::solvePnPRansac(pts3D, pts2D, K, D, rvec, t,  true, 100, 10, 0.99, status);//后期手写

    if(!pnp_succ){
        ROS_WARN("pnp failed ! \n");
        return false;
    }

    cv::Rodrigues(rvec, r);
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd t_pnp;
    cv::cv2eigen(t, t_pnp);

    //赋值当前帧在世界坐标系下的位姿 c_T_w
    R = R_pnp;
    P = t_pnp;
    return true;
}

//本质矩阵E SVD分解,求解出两个旋转和两个位移
void FeatureSolver::decomposeE(cv::Mat E,
                                 cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                                 cv::Mat_<double> &t1, cv::Mat_<double> &t2)
{
    cv::SVD svd(E, cv::SVD::MODIFY_A);
    cv::Matx33d W(0, -1, 0,
                  1, 0, 0,
                  0, 0, 1);
    cv::Matx33d Wt(0, 1, 0,
                   -1, 0, 0,
                   0, 0, 1);
    R1 = svd.u * cv::Mat(W) * svd.vt;
    R2 = svd.u * cv::Mat(Wt) * svd.vt;
    t1 = svd.u.col(2);
    t2 = -svd.u.col(2);
}


// 三角化（opencv）   Point2f 为像素坐标
double FeatureSolver::checkTriangulation(const vector<cv::Point2f> &l,
                                         const vector<cv::Point2f> &r,
                                         vector<cv::Point3f> &points3d,
                                         vector<uchar> &status,
                                         cv::Mat_<double> R, cv::Mat_<double> t)
{
    cv::Mat pointcloud;
    vector<cv::Point3f> points3d_;
    vector<uchar> status_;
    cv::Matx34f P = cv::Matx34f(1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, 1, 0);
    cv::Matx34f P1 = cv::Matx34f(R(0, 0), R(0, 1), R(0, 2), t(0),
                                 R(1, 0), R(1, 1), R(1, 2), t(1),
                                 R(2, 0), R(2, 1), R(2, 2), t(2));
    cv::triangulatePoints(P, P1, l, r, pointcloud);
    
    int front_count = 0;
    for (int i = 0; i < pointcloud.cols; i++){
        double normal_factor = pointcloud.col(i).at<double>(3);
        cv::Mat_<double> p_3d_l = cv::Mat(P) * (pointcloud.col(i) / normal_factor);
        cv::Mat_<double> p_3d_r = cv::Mat(P1) * (pointcloud.col(i) / normal_factor);

        if (p_3d_l(2) > 0 && p_3d_r(2) > 0 && !isinf(p_3d_l(2)) && !isinf(p_3d_r(2))){ //判断是否无穷远
            front_count++;
            status_.push_back(1);
            cv::Mat_<double> x = pointcloud.col(i) / normal_factor;

            cv::Point3f p(x.at<double>(0, 0), x.at<double>(1, 0), x.at<double>(2, 0));
            points3d_.push_back(p);
        } else {
            status_.push_back(0);
        }
    }
    //拷贝
    status = status_;
    points3d = points3d_;

    return front_count;
}

// 三角化(手写)   Point2f 为归一化平面坐标
double FeatureSolver::checkTriangulation2(const vector<cv::Point2f> &l,
                                         const vector<cv::Point2f> &r,
                                         vector<cv::Point3f> &points3d,
                                         vector<uchar> &status,
                                         cv::Mat_<double> R, cv::Mat_<double> t)
{
    cv::Mat pointcloud;
    vector<cv::Point3f> points3d_;
    vector<uchar> status_;

    Mat33 R_;
    R_ << R(0, 0), R(0, 1), R(0, 2), 
          R(1, 0), R(1, 1), R(1, 2), 
          R(2, 0), R(2, 1), R(2, 2);
    SO3 SO_R(R_);
    Vec3 t_;
    t_ << t(0), t(1), t(2);
    SE3 T_c_w(SO_R, t_);
                
    std::vector<SE3> poses{SE3(SO3(), Vec3::Zero()), T_c_w}; 
    int front_count = 0;
    for (size_t i = 0; i < l.size(); ++i) {
        //归一化平面坐标
        std::vector<Vec3> points{Vec3(l[i].x, l[i].y, 1), Vec3(r[i].x, r[i].y, 1)};
        Vec3 pworld = Vec3::Zero();    
        bool isSvd = triangulation(poses, points, pworld);
        Vec3 r_pose = camera_left_->world2camera(pworld, T_c_w);
        if ( isSvd && pworld(2) > 0 && r_pose(2) > 0 ) {
            front_count++;
            cv::Point3f p(pworld(0), pworld(1), pworld(2));
            points3d_.push_back(p);
            status_.push_back(1);
        } else {
            status_.push_back(0);
        }
     }
    //拷贝
    status = status_;
    points3d = points3d_;
    return front_count;
}





// Compute Scene Depth (q=2 median). Used in monocular. 评估当前关键帧场景深度，q=2表示中值. 只是在单目情况下才会使用
// 其实过程就是对当前关键帧下所有地图点的深度进行从小到大排序,返回距离头部其中1/q处的深度值作为当前场景的平均深度
float FeatureSolver::ComputeSceneMedianDepth(const vector<cv::Point3f> points3d, 
                                                const vector<uchar> status, 
                                                    const int q)
{
    vector<float> vDepths;
    // 遍历每一个地图点,计算并保存其在当前关键帧下的深度
    for(size_t i=0; i<points3d.size(); i++){
        if(status[i]){
            float z = points3d[i].z; // 
           // cout << points3d[i].x << " , "<<points3d[i].y << " , "<< points3d[i].z<< endl;
            vDepths.push_back(z);
        }
    }
    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}





