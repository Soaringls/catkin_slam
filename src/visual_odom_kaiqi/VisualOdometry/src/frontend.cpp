/*******************************************************
 * Copyright (C) 2021, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VO.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Kai Qi (kaiqigu0110@163.com)
 *******************************************************/

#include "frontend.h"



Frontend::Frontend() {


}


//前端流程
bool Frontend::process(Frame::Ptr new_frame){
    current_frame_ = new_frame;
    allframe_.push_back(current_frame_);
    switch (status_) {
    case FrontendStatus::INITING:
        frontendInit();
        break;
    case FrontendStatus::TRACKING_GOOD:
    case FrontendStatus::TRACKING_BAD:
        featureTrack();
        break;
    case FrontendStatus::LOST:
        Reset();
        break;
    }
    last_frame_ = current_frame_;
    return true;
}


/* --前端初始化--
 * 计算基础矩阵和单应性矩阵，选取其中一个模型，恢复出最开始两帧之间的相对姿态以及点云
 * 得到初始两帧的匹配、相对运动、初始MapPoints
 * Step 1：（未创建）选取特征点数量大于100的帧，作为初始化的第一
 * Step 2：（已创建）筛选相对于第一帧特征点平均视差大于15、三角化质量大于百分之60，作为初始化的第二帧
 *      Step 2.1： 计算E矩阵（RANSAC）
 *      Step 2.2： SVD方法分解R t
 *      Step 2.3： 三角化求特征点、同时筛选R，t（两帧相机系点深度同时大于0）
 *      Step 2.4： 根据三角化质量判断是否可以初始化
 * Step 3: 尺度归一化,根据深度中值
 * Step 4: 创建初始地图
 * Step 5: 全局BA （固定第一帧）
 * 
*/
bool Frontend::frontendInit(){
    cv::namedWindow("showBA"); 
    cv::namedWindow("showPNP"); 

    // Step 1：创建初始化第一帧
    if(!mpInitializer){
        mInitialFrame_ = current_frame_;
        int init_features_num = DetectFeatures();
        if(1){ //生成第一帧图像
            cv::cvtColor(current_frame_->left_img_, mshow_Initimage, CV_GRAY2RGB);
            for(size_t i = 0; i < current_frame_->features_left_.size(); i++){
                cv::circle(mshow_Initimage, current_frame_->features_left_[i]->pt_, 4, cv::Scalar(255,0,0), 2);
            }
            pubTrackImage(header, mshow_Initimage);
        }
        if(init_features_num > 100){ // 初始帧特征点数量必须大于100
            ROS_INFO("Mono Init: Create first frame. features num: %d", init_features_num);
            mpInitializer = true;
            return true;
        }
        // 如果初始化失败，丢弃当前帧
        allframe_.clear();

    // Step 2：创建初始化第二帧    
    } else {
        last_frame_ = mInitialFrame_;
        int num_track_last = TrackLastFrame();
        //计算平均视差
        int ii = 0; double esum = 0;
        for (size_t i = 0; i < mInitialFrame_->features_left_.size(); ++i) {
            if(mTrackStatus[i]){
                double ex = mInitialFrame_->features_left_[i]->pt_.x - current_frame_->features_left_[ii]->pt_.x;
                double ey = mInitialFrame_->features_left_[i]->pt_.y - current_frame_->features_left_[ii]->pt_.y;
                esum += ((ex*ex) + (ey*ey)); 
                ii++;
            }
        }
        double eavg = esum/(double)ii;
        if(num_track_last < (mInitialFrame_->features_left_.size() * 0.5)){ //追踪到小于50%的点（100 * 0.4）
            mpInitializer = false;
            allframe_.clear();
            ROS_WARN("Mono Init: Translation is not enough, Track Last points :%d", num_track_last);
            return false;
        /*平均视差大于15*/
        } else if(eavg < 15*15){
            //舍弃当前帧
            allframe_.pop_back();
            ROS_WARN("Mono Init: The average parallax so small : %f", eavg);
            return false;
        } else {
            ROS_INFO("Mono Init: Create second frame. features num: %d", num_track_last);
            // Step 2.1：计算 E矩阵 
            vector<cv::Point2f> u_init_points,init_points;
            vector<cv::Point2f> u_cur_points,cur_points;
            // 把两帧特征转换到归一化平面下
            size_t j = 0;
            for (size_t i = 0; i < mInitialFrame_->features_left_.size(); ++i) {
                if(mTrackStatus[i]){
                    Vec3 eInitPoint = camera_left_->pixel2camera(
                        Vec2(mInitialFrame_->features_left_[i]->pt_.x,
                             mInitialFrame_->features_left_[i]->pt_.y));
                    Vec3 eCurPoint = camera_left_->pixel2camera(
                        Vec2(current_frame_->features_left_[j]->pt_.x,
                             current_frame_->features_left_[j]->pt_.y)); 
                    cv::Point2f init_point(eInitPoint[0], eInitPoint[1]);
                    cv::Point2f cur_point(eCurPoint[0], eCurPoint[1]);  
                    u_init_points.push_back(init_point);
                    u_cur_points.push_back(cur_point);

                    j++;
                }
            }
            TicToc frontendInit;
            /* 求解 本质矩阵E
            *  Mat cv::findFundamentalMat(  返回通过RANSAC算法求解两幅图像之间的本质矩阵E
            *      nputArray  points1,             第一幅图像点的数组
            *      InputArray  points2,            第二幅图像点的数组
            *      int     method = FM_RANSAC,     RANSAC 算法
            *      double  param1 = 3.,            点到对极线的最大距离，超过这个值的点将被舍弃
            *      double  param2 = 0.99,          矩阵正确的可信度
            *      OutputArray mask = noArray())   输出在计算过程中没有被舍弃的点
            */     
            vector<uchar> RANSACstatus;
            cv::Mat E = cv::findFundamentalMat(u_init_points, u_cur_points, CV_FM_RANSAC, 1, 0.99, RANSACstatus);

            // Step 2.2：SVD 分解R t
            // SVD 从 E 分解 R、t
            cv::Mat_<double> R1, R2, t1, t2;
            featuresolver_->decomposeE(E, R1, R2, t1, t2);
            if (determinant(R1) + 1.0 < 1e-09){//如果行列式为负，SVD分解-E
                E = -E;
                featuresolver_->decomposeE(E, R1, R2, t1, t2);
            } 
            // Step 2.3：根据三角测量筛选R，t ，同时计算空间点
            // 计算三角化出来四个深度大于0的比例,比例最多的那组R t
            vector<cv::Point3f> points3d_1, points3d_2, points3d_3, points3d_4;
            vector<uchar> status1, status2, status3, status4;
            int SuccNum1 = featuresolver_->checkTriangulation2(u_init_points, u_cur_points, 
                                                               points3d_1, status1, R1, t1);
            int SuccNum2 = featuresolver_->checkTriangulation2(u_init_points, u_cur_points, 
                                                               points3d_2, status2, R1, t2);
            int SuccNum3 = featuresolver_->checkTriangulation2(u_init_points, u_cur_points, 
                                                               points3d_3, status3, R1, t1);
            int SuccNum4 = featuresolver_->checkTriangulation2(u_init_points, u_cur_points, 
                                                               points3d_4, status4, R2, t2);
            // 选取最大可三角化测量的点的数目
            int maxratioNum = max(SuccNum1, max(SuccNum2, max(SuccNum3, SuccNum4)));
            ROS_INFO("Mono Init : Triangulation Point per: %f", ((double)maxratioNum / u_init_points.size()));

  //          ROS_INFO("frontendInit costs: %f ms", frontendInit.toc()); ///////////////
            // Step 2.4: 判断是否初始化成功
            if(((double)maxratioNum / u_init_points.size()) < 0.6){// 如果3d点数量少于40%，则初始化失败
                // 如果失败舍弃当前帧
                allframe_.pop_back();
                ROS_WARN("Translation is not enough, Triangulation Point per: %f", ((double)maxratioNum / u_init_points.size()));
                return false;
            } 

            Mat33 R_;
            Vec3 t_;
            vector<cv::Point3f> init_points3d;
            vector<uchar> TrianStatus;
            if(maxratioNum == SuccNum1){
                cv::cv2eigen(R1, R_);
                t_ << t1(0), t1(1), t1(2);
                init_points3d = points3d_1;
                TrianStatus = status1;
            }
            if(maxratioNum == SuccNum2){
                cv::cv2eigen(R1, R_);
                t_ << t2(0), t2(1), t2(2);
                init_points3d = points3d_2; 
                TrianStatus = status2;
            }
            if(maxratioNum == SuccNum3){
                cv::cv2eigen(R2, R_);
                t_ << t1(0), t1(1), t1(2);
                init_points3d = points3d_3;
                TrianStatus = status3;
            }
            if(maxratioNum == SuccNum4){
                cv::cv2eigen(R2, R_);
                t_ << t2(0), t2(1), t2(2);
                init_points3d = points3d_4;
                TrianStatus = status4;
            }
            for(size_t i = 0; i < RANSACstatus.size(); i++){ 
                //当正向和反向都跟踪成功，并且反向跟踪回来的点与前一帧的点像素小于1时，跟踪成功（绝对匹配）
                if(TrianStatus[i] && RANSACstatus[i]) 
                    TrianStatus[i] = 1;
                else 
                    TrianStatus[i] = 0;  
            }  

            // Step 3: 尺度归一化
            // 选取 场景点的中值深度，用作尺度归一化,2为 1/2的sort深度位置
            float medianDepth = featuresolver_->ComputeSceneMedianDepth(init_points3d, TrianStatus, 2);
            double invMedianDepth = 1.0f/medianDepth;
            iScale = invMedianDepth;
            iScale = iScale * SCALE_S;
            cout  <<"medianDepth:" <<medianDepth  <<" Scale: " << 1.0f/medianDepth << endl;
            ROS_DEBUG("(Init)Scale is: %f", iScale);

            // 设置归一化位姿
            SO3 SO3_R_first(Mat33::Identity());
            mInitialFrame_->SetPose(SE3(SO3_R_first, Vec3::Zero()));// set第一帧位姿
            SO3 SO3_R_second(R_);
            current_frame_->SetPose(SE3(SO3_R_second, t_ * iScale));// set第二帧位姿

            // Step 4: 建立初始地图
            int features_landmarks_num = BuildInitMap(init_points3d, mTrackStatus, TrianStatus, iScale);
            ROS_INFO("BuildInitMap 3D landmarks num: %d", features_landmarks_num);

            // 把第一帧和当前帧都加到关键帧里
            mInitialFrame_->SetKeyFrame();
            current_frame_->SetKeyFrame();
            map_->InsertKeyFrame(mInitialFrame_);
            map_->InsertKeyFrame(current_frame_);

            keyframe_.push(mInitialFrame_);
            keyframe_.push(current_frame_);

            // Step 5 : BA优化
            //Optimizer optimizer_; //创建优化器
            MAP::LandmarksType landmarks = map_->GetAllMapPoints();
            Optimizer::InitBA(mInitialFrame_, 
                              current_frame_, 
                              landmarks,
                              0);
    // 验证重投影效果
            Mat44 T_BA = current_frame_->Pose().matrix();
            cout << "after BA Pose: " << T_BA << endl;

            cv::Mat mshow_initimageBA, mshow_secondimageBA;
            cv::cvtColor(mInitialFrame_->left_img_, mshow_initimageBA, CV_GRAY2RGB);
            for(auto feat : mInitialFrame_->features_left_){
                if(feat->map_point_.lock() == nullptr) continue;
                cv::circle(mshow_initimageBA, feat->pt_, 4, cv::Scalar(250,0,0), 2);
                auto map = feat->map_point_.lock();
                Vec3 pose = map->Pos();
                Vec2 ba_pose = camera_left_->camera2pixel(pose);
                cv::circle(mshow_initimageBA, cv::Point2f(ba_pose(0),ba_pose(1)), 3, cv::Scalar(0,0,250), -1);
                cv::line(mshow_initimageBA, feat->pt_, cv::Point2f(ba_pose(0),ba_pose(1)), cv::Scalar(0,255,0), 1);
            }
            cv::cvtColor(current_frame_->left_img_, mshow_secondimageBA, CV_GRAY2RGB);
            for(auto feat : current_frame_->features_left_){
                if(feat->map_point_.lock() == nullptr) continue;
                cv::circle(mshow_secondimageBA, feat->pt_, 4, cv::Scalar(250,0,0), 2);
                auto map = feat->map_point_.lock();
                Vec3 w_pose = map->Pos();
                SE3 T_c_w = current_frame_->Pose();
                Vec3 c_pose = camera_left_->world2camera(w_pose, T_c_w);
                Vec2 ba_pose = camera_left_->camera2pixel(c_pose);
                cv::circle(mshow_secondimageBA, cv::Point2f(ba_pose(0),ba_pose(1)), 3, cv::Scalar(0,0,250), -1);
                cv::line(mshow_secondimageBA, feat->pt_, cv::Point2f(ba_pose(0),ba_pose(1)), cv::Scalar(0,255,0), 1);
            }    
            pubTrackImage(header, mshow_secondimageBA);   
    
            //初始化成功
            ROS_INFO("--------VO Init success--------");

            //补充新特征点
            TicToc t_DetectFeatures;
            int new_ptr_num = DetectFeatures(); 
            ROS_DEBUG("Add new features number: %d, cost: %f", new_ptr_num, t_DetectFeatures.toc());

            status_ = FrontendStatus::TRACKING_GOOD; 
            //ROS_WARN("-----restart Init------");
            return true;
        }
    }
    
}


/* --追踪--
 * 计算基础矩阵和单应性矩阵，选取其中一个模型，恢复出最开始两帧之间的相对姿态以及点云
 * 得到初始两帧的匹配、相对运动、初始MapPoints
 * Step 1：特征追踪
 * Step 2：帧间位姿估计
 *      Step 2.1： 获取当前帧的观测数据：3D和2D点
 *      Step 2.2： 剔除outlier
 *      Step 2.3： RANSAC + EPNP 求解R、t
 * Step 3: 判断关键帧（两种方式）
 *      Step 3.1： 直方图分析
 *      Step 3.2： 共视点  
 * Step 4: 采用共视图做重投影优化（ORB：Covibilitygraph）
 * Step 5: 筛选oulier-卡方检测
 * Step 6: 三角化新地图点
*/
int Frontend::featureTrack(){
    current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
    TicToc t_featureTrack;
    //Step 1：特征追踪
    TicToc t_TrackLastFrame;
    int num_track_last = TrackLastFrame();
    ROS_DEBUG("Track last features number: %d, cost: %f", num_track_last, t_TrackLastFrame.toc());

    //Step 2：PNP求解 R、t
    vector<cv::Point2f> p_points2d;
    vector<cv::Point3f> w_points3d;   
    vector<MapPoint::Ptr> ptr_points3d;
    cout << "PNP 2d point num: " << current_frame_->features_left_.size()<<endl;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i){
        auto map_point = current_frame_->features_left_[i]->map_point_.lock();
        if (map_point && !map_point->is_outlier_){
            //cout << "PNP 2d point age: " << current_frame_->features_left_[i]->age_<<endl;
            cv::Point2f p_point2d(current_frame_->features_left_[i]->pt_);
            cv::Point3f w_point3d(map_point->pos_[0], map_point->pos_[1], map_point->pos_[2]);
            p_points2d.push_back(p_point2d);
            w_points3d.push_back(w_point3d);
            ptr_points3d.push_back(map_point);
        }
    }

    TicToc time_pnp;
    //加入pnp解的判断（可以通过内外点数量比）
    Mat33 R_pnp;
    Vec3 t_pnp;
    Mat44 T_pnp;
    //设置初值
    T_pnp = current_frame_->pose_.matrix();
    R_pnp = T_pnp.block(0,0,3,3);
    t_pnp = T_pnp.block(0,3,3,1);
    cv::Mat pnp_status;
    bool ifpnp = featuresolver_->solvePoseByPnP(p_points2d, w_points3d, R_pnp, t_pnp, pnp_status);

    ROS_DEBUG("Epnp cost: %f", time_pnp.toc());

    SO3 SO3_R_pnp(R_pnp);
    current_frame_->SetPose(SE3(SO3_R_pnp, t_pnp));

    //Step 3： 判断是否需要插入关键帧
    Frame::Ptr near_keyframe = keyframe_.back();
    size_t nearkeyframeindex;
    for(size_t i = 0;i < allframe_.size();i++){
        if(allframe_[i] == near_keyframe){
            nearkeyframeindex = i;
            break;
        }
    }
    int refAge = allframe_.size() - nearkeyframeindex;
    //求共视点数量
    int commonfeat = 0;
    for(auto feat : current_frame_->features_left_)
        if(feat->age_ >= refAge) commonfeat++;
    bool ifneadBA = 0;
    if (commonfeat < MAX_CNT * 0.4){
        //Insert keyframe
        ifneadBA = 1; //需要优化
        InsertKeyframe();
    }


    static int index = 0;
    cout << " ---------------BA: "<< index<<" --------------"<< endl;
    // for(auto frame : allframe_){
    //     string writename = "./out/beforBA"+to_string(index)+ "-" + to_string(frame->id_) +".jpg";
    //     cv::Mat showimagePNP = showframeReject(frame); 
    //     cv::imwrite(writename, showimagePNP);
    //     if(frame == current_frame_){
    //         cv::imshow("showPNP", showimagePNP);
    //         cv::waitKey(10);
    //     }
    // }
 
    TicToc t_LocalBA;
    // Step 4: 局部BA优化
    MAP::KeyframesType keyframes = map_->GetAllKeyFrames();
    MAP::LandmarksType landmarks = map_->GetCurrectMapPoints(current_frame_);

    if(1){
        Optimizer::LocalBA(current_frame_,mInitialFrame_, 
                           keyframes,
                           landmarks,
                           10);
    }
    cout << "LocalBA cost: "<< t_LocalBA.toc() << endl;

    // Step 5: 重投影剔除outline（卡方）
     int outlier = RemoveOutlierMappoint(landmarks);
     cout << "Remove Outlier: "  << outlier <<endl;

    // for(auto frame : allframe_){
    //     string writename = "./out/afterBA"+to_string(index)+ "-" + to_string(frame->id_) +".jpg";
    //     cv::Mat showimageBA = showframeReject(frame); 
    //     //string writename = "BA"+to_string(index)+to_string(keyframe.second->id_) + ".jpg";
    //     cv::imwrite(writename, showimageBA);
    //     if(frame == current_frame_){
    //         cv::imshow("showBA", showimageBA);
    //         cv::waitKey(10);
    //     }
    // }
    index++;

    // 显示优化结果
    cv::Mat showimageBA = showframeReject(current_frame_); 
    pubBAImage(header, showimageBA);


    // Step 6: 三角化新地图点
    TicToc t_TriangulateNewPoints;
    int new_point_num = CreateNewMapPoints();
    ROS_DEBUG("Add new 3d points number: %d, cost: %f", new_point_num, t_TriangulateNewPoints.toc());

    // Step 6: 补充特征点
    if (num_track_last < MAX_CNT){
        //Detect new features
        TicToc t_DetectFeatures;
        int new_ptr_num = DetectFeatures(); 
        ROS_DEBUG("Add new features number: %d, cost: %f", new_ptr_num, t_DetectFeatures.toc());
    }

    /// 直方图分析（统计特征点年龄，用于验证关键帧)
        if(0){
            cv::Mat histimage;
            //画直方图
            int featureSize = current_frame_->features_left_.size();
            Eigen::VectorXf  eigen_age(featureSize);
            for (int i = 0; i < featureSize; ++i){
                eigen_age[i] = current_frame_->features_left_[i]->age_;
            }

            //直方图
            Eigen::Matrix<int,100,1> my_eigenHist = Eigen::Matrix<int,100,1>::Zero();
            cv::Mat myHist;
            float sum = 0;
            for(int i=0;i < eigen_age.rows();i++){
                if(eigen_age[i] < 100) {
                    my_eigenHist[eigen_age[i]]++;
                    sum += eigen_age[i];
                }
            }
            int scale = 15;
            int hist_height = 400;
            cv::Mat hist_img = cv::Mat::zeros(hist_height, 101 * scale, CV_8UC3); //创建一个黑底的8位的3通道图像，高256，宽256*2
            //遍历直方图得到的数据
            for (int i = 0; i < my_eigenHist.rows(); i++){
                int bin_val = my_eigenHist[i];   //遍历hist元素（注意hist中是float类型）
                int intensity = bin_val*3;  //绘制高度`````````````````````````
                char txt[3],txt1[3];
                sprintf(txt, "%d", intensity/3); 
                sprintf(txt1, "%d", i+1); 
                if(bin_val > 0){
                    cv::rectangle(hist_img, cv::Point(i*scale, hist_height - 1), cv::Point((i + 1)*scale - 2, hist_height - intensity - 10), cv::Scalar(255, 255, 255),CV_FILLED);//绘制直方图
                    cv::putText(hist_img, txt1, cv::Point(i*scale, hist_height - 3), CV_FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 200, 0));
                    cv::putText(hist_img, txt, cv::Point(i*scale, hist_height - intensity - 13), CV_FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255));
                }
            }
            char median[20],average[20];
            int median_ = eigen_age[cvRound((float)(eigen_age.rows())/2)];//取中位数
            int average_ = (int)(sum/100);
            sprintf(median, "median: %d", median_); 
            sprintf(average, "average: %d", average_); 

            cv::putText(hist_img, median, cv::Point(101 * scale - 200, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 200, 0));
            cv::putText(hist_img, average, cv::Point(101 * scale - 200, 40), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 200, 0));

            cv::namedWindow("histimage");       
            cv::imshow("histimage", hist_img);
            cv::waitKey(33); 
        }
    ///.


    ROS_INFO("featureTrack costs: %f ms", t_featureTrack.toc());

    //测试
    if(1){
        cv::Mat show_leftimage, showimage;
        showimage = show_Trackleftimage.clone();
        pubTrackImage(header, showimage);

        // 空间点显示
        std::unordered_map<unsigned long, MapPoint::Ptr> landmarks_ = map_->GetAllMapPoints(); //获取地图点
        std::vector<Eigen::Vector3d> show_landmarks_;
        for (auto landmark : landmarks_) {
            if(landmark.second->is_outlier_) continue;
            auto pos = landmark.second->Pos();
            Vec3 elandmark(pos[0], pos[1], pos[2]);
            show_landmarks_.push_back(elandmark);
        }
        std::vector<Eigen::Vector3d> show_cur_mappoints;
        for(size_t i = 0; i < current_frame_->features_left_.size(); ++i){
            auto map_point = current_frame_->features_left_[i]->map_point_.lock();
            if (map_point && !map_point->is_outlier_){
                Vec3 eigen_w_mappoind;  
                eigen_w_mappoind << map_point->pos_[0], map_point->pos_[1], map_point->pos_[2];
                show_cur_mappoints.push_back(eigen_w_mappoind);
            }
        }
        pubMapPoint(header, show_landmarks_);// 显示所有地图点
        pubPointCloud(header, show_cur_mappoints);//显示当前观测的到的地图点
        // show_pointCloud.clear();

        /*坐标变换*/
        // cam_T_w ---> w_T_cam
        Mat33 w_R_c = R_pnp.inverse(); 
        Vec3  w_t_c = w_R_c * (-t_pnp);

        //pubPath(header, w_t_c, RRR);
        pubTF(header, w_t_c, w_R_c);
        pubOdometry(header, w_t_c, w_R_c);

        if(current_frame_->is_keyframe_ == true){
            pubKeyframePose(header, w_t_c, w_R_c);
        }
    }
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();
} 

int Frontend::TrackLastFrame() {
    // use LK flow to track in the last image
    std::vector<cv::Point2f> last_pts, cur_pts;
    //设置当前图像点初值（快速收敛）
    //
    for (auto &pts : last_frame_->features_left_){
        last_pts.push_back(pts->pt_);
        cur_pts.push_back(pts->pt_);
    }
    /*光流追踪*/
    vector<uchar> status;
    vector<float> err;
    cv::calcOpticalFlowPyrLK(last_frame_->left_img_, current_frame_->left_img_, 
                            last_pts, cur_pts, status, err, cv::Size(21, 21), 3); 
    /*反向光流*/
    vector<cv::Point2f> reverse_last_pts;
    vector<uchar> reverse_status;
    vector<float> reverse_err;
    cv::calcOpticalFlowPyrLK(current_frame_->left_img_, last_frame_->left_img_, 
        cur_pts, reverse_last_pts, reverse_status, reverse_err, cv::Size(21, 21), 3); 
 
    for(size_t i = 0; i < status.size(); i++){ 
        //当正向和反向都跟踪成功，并且反向跟踪回来的点与前一帧的点像素小于1时，跟踪成功（绝对匹配）
        if(status[i] && reverse_status[i] && inBorder(cur_pts[i]) 
                     && distance(last_pts[i], reverse_last_pts[i]) < 1)
            status[i] = 1;
        else 
            status[i] = 0;  
    }   
    mTrackStatus = status; //状态向量更新
    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            Feature::Ptr feature(new Feature(current_frame_, cur_pts[i]));
            feature->age_ = last_frame_->features_left_[i]->age_ + 1;
            feature->id_ = last_frame_->features_left_[i]->id_;
            feature->map_point_ = last_frame_->features_left_[i]->map_point_;
            auto map =  feature->map_point_.lock();
            if(feature->map_point_.lock() != nullptr)
                map->AddObservation(feature);
            current_frame_->features_left_.push_back(feature);
            num_good_pts++;
        }
    }
    //测试
    if(1){
        cv::cvtColor(current_frame_->left_img_, show_Trackleftimage, CV_GRAY2RGB);
        for(size_t i = 0; i < cur_pts.size(); i++){
            cv::circle(show_Trackleftimage, cur_pts[i], 4, cv::Scalar(200,0,0), 2);
            if (status[i]){
                cv::line(show_Trackleftimage, cur_pts[i], last_pts[i], cv::Scalar(0,255,0), 2);
            }                                                         
        }
        for(auto feature : current_frame_->features_left_){
            char txt[20];
            sprintf(txt, "%ld| %d", (long int)feature->id_, feature->age_);
            cv::Point2f left_pt(feature->pt_.x - 8,feature->pt_.y - 8 );
            int agecolor = 50 + feature->age_ * 3;
            if (agecolor > 255) agecolor = 255;
            cv::putText(show_Trackleftimage, txt, left_pt, CV_FONT_HERSHEY_SIMPLEX, 
                                        0.4, cv::Scalar(agecolor, 20, 255 - agecolor), 1.6); 
        }
        header.frame_id = "base_link";
        header.stamp = ros::Time(ros::Time::now()); 
        pubTrackImage(header, show_Trackleftimage);


        string writename = "./out/trackimage"+ to_string(current_frame_->id_) +".jpg";
        cv::imwrite(writename, show_Trackleftimage); 
     }
    return num_good_pts;
}



bool Frontend::InsertKeyframe() {
    // Current frame is a new keyframe
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);
    keyframe_.push(current_frame_);

    //Set Observations For KeyFrame
    // for (auto &feat : current_frame_->features_left_) {
    //     auto mp = feat->map_point_.lock();
    //     if (mp) mp->AddObservation(feat);
    // }

    ROS_INFO("Insert new Keyframe, id: %d", keyframe_.size());
    return true;
}


int Frontend::DetectFeatures(){
    //设置mask
    cv::Mat mask = cv::Mat(IMAGE_ROW, IMAGE_COL, CV_8UC1, cv::Scalar(255));
    if(!current_frame_->features_left_.empty()){
        for (auto &feat : current_frame_->features_left_) {
            cv::circle(mask, feat->pt_, MIN_DIST, 0, -1);
        }
    }

    vector<cv::Point2f> new_pts;
    TicToc t_detect;
    cv::goodFeaturesToTrack(current_frame_->left_img_, new_pts, 
        MAX_CNT - current_frame_->features_left_.size(), 
            0.01, MIN_DIST, mask);
    ROS_DEBUG("detect feature costs: %f ms", t_detect.toc());

    for (auto &pt : new_pts) {
        current_frame_->features_left_.push_back(
        Feature::CreateNewFeature(current_frame_, pt));
    }
    // 测试角点用
    if(0){ 
        //cv::Mat showimage = current_frame_->left_img_.clone();
        cv::Mat showimage = mask.clone();
        cv::cvtColor(showimage, showimage, CV_GRAY2RGB);
        for(uint i=0;i<new_pts.size();i++){
            cv::circle(showimage, new_pts[i], 4, cv::Scalar(255,0,0), 2);
        }                    
        cv::namedWindow("showimage");       
        cv::imshow("showimage", showimage);
        cv::waitKey(333); 
    }
    return new_pts.size();
}


int Frontend::BuildInitMap(vector<cv::Point3f> points3d_, vector<uchar> trackstatus_, vector<uchar> trianstatus_, float scale_){
    int trackIndex = 0;
    int landmarksIndex = 0;
    for (size_t i = 0; i < trackstatus_.size(); i++) {
        if (trackstatus_[i]){ //如果追踪到了当前帧
            if (trianstatus_[trackIndex]){ //当前帧有地图点
                Vec3 pworld(points3d_[landmarksIndex].x*scale_, 
                            points3d_[landmarksIndex].y*scale_, 
                            points3d_[landmarksIndex].z*scale_);
                //cout << " pworld: " << pworld.transpose() <<endl;
                // // 重投影剔除outline（2次卡方）    
                // Vec3 c_pose = camera_left_->world2camera(pworld, mInitialFrame_->Pose());
                // Vec2 Reject_pose = camera_left_->camera2pixel(c_pose);
                // double errX1 = Reject_pose(0) - mInitialFrame_->features_left_[i]->pt_.x;
                // double errY1 = Reject_pose(1) - mInitialFrame_->features_left_[i]->pt_.y;
                // // 假设测量有一个像素的偏差，2自由度卡方检验阈值是5.991
                // if((errX1*errX1+errY1*errY1)>5.991)
                //     continue;
                // Vec3 c_pose2 = camera_left_->world2camera(pworld, current_frame_->Pose());
                // Vec2 Reject_pose2 = camera_left_->camera2pixel(c_pose2);
                // double errX2 = Reject_pose2(0) - current_frame_->features_left_[trackIndex]->pt_.x;
                // double errY2 = Reject_pose2(1) - current_frame_->features_left_[trackIndex]->pt_.y;
                // // 假设测量有一个像素的偏差，2自由度卡方检验阈值是5.991
                // if((errX2*errX2+errY2*errY2)>5.991)
                //     continue;

                MapPoint::Ptr new_map_point = MapPoint::CreateNewMappoint();
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(mInitialFrame_->features_left_[i]);
                new_map_point->AddObservation(current_frame_->features_left_[trackIndex]);
                mInitialFrame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_left_[trackIndex]->map_point_ = new_map_point;
                map_->InsertMapPoint(new_map_point);
                landmarksIndex++;
            }
            trackIndex++;
        }

    }
    //测试
    if(1){
        std::vector<Eigen::Vector3d> show_pointCloud;
        for(size_t i = 0; i < current_frame_->features_left_.size(); ++i){
            auto map_point = current_frame_->features_left_[i]->map_point_.lock();
            if (map_point){
                //转到Lidar系
                Vec3 eigen_l_mappoind;
                Vec3 v_mappoint;
                v_mappoint << map_point->pos_[0], map_point->pos_[1], map_point->pos_[2];
                eigen_l_mappoind = camera_left_->camera2lidar(v_mappoint);
                show_pointCloud.push_back(eigen_l_mappoind); 
            }
        }

        header.frame_id = "velo_link";
        
       // cout <<"show_pointCloud : " <<show_pointCloud.size() <<endl;
        // for(size_t i = 0;i<show_pointCloud.size();i++){
        //     cout << show_pointCloud[i].transpose()<<endl;
        // }
        pubPointCloud(header, show_pointCloud);
        show_pointCloud.clear();
    }
    return landmarksIndex;
}

/* --创建新地图点--
 * 帧间三角化对合适的地图点进行创建
 * Step 1：判断特征点年龄是否大于2
 * Step 2：计算观测基线是否足够（大于景深/100） 
 * Step 3: 三角化
 * Step 4: 重投影剔除outlier（卡方）
 * Step 5: 添加新地图点
*/
int Frontend::CreateNewMapPoints() {
    vector<cv::Point3f> points3d;
    vector<uchar> status;
    // 计算景深 : 深度中值
    SE3  cT_c_w = current_frame_->Pose(); //Tcw
    for(auto curfeat : current_frame_->features_left_){
        auto map = curfeat->map_point_.lock();
        if(map == nullptr)  continue;
        Vec3 w_pose = map->Pos();
        Vec3 c_pose = camera_left_->world2camera(w_pose, cT_c_w);
        points3d.push_back(cv::Point3f(c_pose(0), c_pose(1), c_pose(2)));
        status.push_back(1);
    }
    float medianDepth = featuresolver_->ComputeSceneMedianDepth(points3d, 
                                                                status, 2);
    int cnt_triangulated_pts = 0;
    for(auto curfeat : current_frame_->features_left_){
        // Step 1：判断特征点年龄是否大于2
        if(curfeat->age_ < 2)  continue;
        auto map = curfeat->map_point_.lock();
        if(map != nullptr)  continue;
        // Step 2：计算观测基线是否足够（大于景深/100）
        // 计算当前帧和最老帧基线
        int oldframeIndex = allframe_.size() - curfeat->age_;
        auto oldframe = allframe_[oldframeIndex];
        SE3  oT_c_w = oldframe->Pose(); //Tcw
        Mat44 ecT_c_w = cT_c_w.matrix();
        Mat44 eoT_c_w = oT_c_w.matrix();
        double dx = ecT_c_w(0, 3) - eoT_c_w(0, 3);
        double dy = ecT_c_w(1, 3) - eoT_c_w(1, 3);
        double dz = ecT_c_w(2, 3) - eoT_c_w(2, 3);
        double baseline = sqrt(dx*dx + dy*dy +dz*dz);
        //cout << "baseline/medianDepth:"<< baseline/medianDepth << " age:" << curfeat->age_ <<" id:" <<  curfeat->id_ <<endl;
        if(baseline/medianDepth < 0.01)continue;
        Feature::Ptr oldfeat = nullptr;
        for(auto feat : oldframe->features_left_){
            if(feat->id_ == curfeat->id_){
                oldfeat = feat;
                break;
            }
        }
        
        // Step 3: 三角化
        std::vector<SE3> poses;
        std::vector<Vec3> points;
        std::vector<Feature::Ptr> vfeat;
 
        for(size_t i = oldframeIndex; i < allframe_.size();i++){
            poses.push_back(allframe_[i]->Pose());
            for(auto feat : allframe_[i]->features_left_){
                if(feat->id_ == curfeat->id_){
                    vfeat.push_back(feat);
                    break;
                }
            }
            // 归一化平面坐标，尝试三角化
            Feature::Ptr nowfeat = vfeat.back();
            points.push_back(camera_left_->pixel2camera(
                                Vec2(nowfeat->pt_.x, nowfeat->pt_.y)));
            
            //cout << "pt_:"<< nowfeat->pt_.x << "," << nowfeat->pt_.y << " |"<< allframe_.size() << " | "  << i<<endl;
            Mat44 T_ = allframe_[i]->Pose().matrix();
            Vec3 t_ = T_.block(0,3,3,1);
            //cout << "POSE:"<< t_.transpose()  <<endl;
        }
        Vec3 pworld = Vec3::Zero();

        triangulation(poses, points, pworld); // 三角化
        //cout << "triangulation:"<< pworld[0] << "," << pworld[1] << ","<< pworld[2]  <<endl;
        if (pworld[2] > 0) {
            int dof_num = 0;
            double error = 0;
            for(size_t i = oldframeIndex; i < allframe_.size();i++){
                
                Vec3 c_pose = camera_left_->world2camera(pworld, allframe_[i]->Pose());
                Vec2 Reject_pose = camera_left_->camera2pixel(c_pose);
                double errX1 = Reject_pose(0) - vfeat[dof_num]->pt_.x;
                double errY1 = Reject_pose(1) - vfeat[dof_num]->pt_.y;
                //cout << "error:"<< Reject_pose(0) << "," << vfeat[dof_num]->pt_.x <<endl;
                //cout << "error:"<< Reject_pose(1) << "," << vfeat[dof_num]->pt_.y <<endl;
                error += errX1*errX1 + errY1*errY1;
                //cout << "kafang:"<< errX1*errX1+errY1*errY1 << " age:" << curfeat->age_ <<" id:" <<  curfeat->id_ <<endl;
                dof_num++;
                if(dof_num>=5)break; //卡方检测自由度小于10、降低运算
            }
            //cout << "Z_kafang:"<< error << " kafang[dof_num]:"<<kafang[dof_num] <<endl;
            // 假设测量有一个像素的偏差，2自由度卡方检验阈值是5.991
            if(error > kafang[dof_num])
                continue;

            //Step 5: 添加新地图点
            auto new_map_point = MapPoint::CreateNewMappoint();
            new_map_point->SetPos(pworld);
            // new_map_point->AddObservation(oldfeat);
            // new_map_point->AddObservation(curfeat);
            for(auto feat : vfeat){
                new_map_point->AddObservation(feat);
                feat->map_point_ = new_map_point;
            }
            // oldfeat->map_point_ = new_map_point;
            // curfeat->map_point_ = new_map_point;
            map_->InsertMapPoint(new_map_point);
            cnt_triangulated_pts++;
        }
    }
    cout << "CreatNewMappoint:" << cnt_triangulated_pts<<endl;
    return cnt_triangulated_pts;
}

//跟踪失败
bool Frontend::Reset() {
    ROS_ERROR("------ Frontend::Reset() ------");
    return true;
}

// 显示关键帧上观测到的地图点
cv::Mat Frontend::showBA(Frame::Ptr curFrame, MAP::LandmarksType landmarks) {
    int num =0;
    cv::Mat mshow_initimageBA;
    cv::cvtColor(curFrame->left_img_, mshow_initimageBA, CV_GRAY2RGB);
    SE3 T_c_w = curFrame->Pose();
    for (auto &landmark : landmarks) {
        Vec3 w_pose = landmark.second->Pos();
        auto observations = landmark.second->GetObs();
        for (auto &obs : observations) { 
            if (obs.lock() == nullptr) continue;
            auto feat = obs.lock();
            auto pframe = feat->frame_.lock(); 
            if (pframe != curFrame) continue;  
            cv::circle(mshow_initimageBA, feat->pt_, 4, cv::Scalar(250,0,0), 2);
            if (landmark.second->is_outlier_){
                cv::circle(mshow_initimageBA, feat->pt_, 4, cv::Scalar(255,0,255), 2);
            }
            Vec3 c_pose = camera_left_->world2camera(w_pose, T_c_w);
            Vec2 ba_pose = camera_left_->camera2pixel(c_pose);
            cv::circle(mshow_initimageBA, cv::Point2f(ba_pose(0),ba_pose(1)), 3, cv::Scalar(0,0,250), -1);
            cv::line(mshow_initimageBA, feat->pt_, cv::Point2f(ba_pose(0),ba_pose(1)), cv::Scalar(0,255,0), 1);

            char txt[20];
            sprintf(txt, "%ld:%d", (long int)feat->id_, feat->age_);
            cv::Point2f left_pt(feat->pt_.x - 8,feat->pt_.y - 8 );
            cv::putText(mshow_initimageBA, txt, left_pt, CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 20, 255)); 
            num++;
        }
    }
    return mshow_initimageBA;
}


// 显示当前帧上观测到的地图点
cv::Mat Frontend::showframeReject(Frame::Ptr curFrame) {
    cv::Mat mshow_initimageBA;
    cv::cvtColor(curFrame->left_img_, mshow_initimageBA, CV_GRAY2RGB);
    SE3 T_c_w = curFrame->Pose();
    for(auto curfeat : curFrame->features_left_){
        auto map = curfeat->map_point_.lock();
        if(map == nullptr)  continue;
        Vec3 w_pose = map->Pos();
        Vec3 c_pose = camera_left_->world2camera(w_pose, T_c_w);
        Vec2 ba_pose = camera_left_->camera2pixel(c_pose);
        cv::circle(mshow_initimageBA, curfeat->pt_, 10, cv::Scalar(250,0,0), 6);
        if (map->is_outlier_){
            cv::circle(mshow_initimageBA, curfeat->pt_, 10, cv::Scalar(0,0,255), 6);
        }
        cv::circle(mshow_initimageBA, cv::Point2f(ba_pose(0),ba_pose(1)), 8, cv::Scalar(0,0,250), -1);
        cv::line(mshow_initimageBA, curfeat->pt_, cv::Point2f(ba_pose(0),ba_pose(1)), cv::Scalar(255,255,0), 4);

        char txt[20];
        sprintf(txt, "%ld:%d| %2d", (long int)curfeat->id_, curfeat->age_, c_pose(2));
        cv::Point2f left_pt(curfeat->pt_.x - 8,curfeat->pt_.y - 8 );
        //cv::putText(mshow_initimageBA, txt, left_pt, CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 20, 255), 2); 
    }
    return mshow_initimageBA;
}

// 剔除错误点
int  Frontend::RemoveOutlierMappoint(MAP::LandmarksType landmarks){ // 剔除outlier
    int outlier = 0; 
    for(auto landmark : landmarks){
        auto observations = landmark.second->GetObs();
        for (auto &obs : observations) {
            if (obs.lock() == nullptr) continue;
            auto feat = obs.lock();
            auto pframe = feat->frame_.lock();
            // if (pframe != current_frame_) continue; //只在当前帧重投影
            // 计算重投影
            SE3  T_c_w = pframe->Pose();
            Vec3 w_pose = landmark.second->Pos();
            Vec3 c_pose = camera_left_->world2camera(w_pose, T_c_w);
            Vec2 RejectPose = camera_left_->camera2pixel(c_pose);
            double errX = RejectPose(0) - feat->pt_.x;
            double errY = RejectPose(1) - feat->pt_.y;
            // 卡方检测, 2自由度卡方检验阈值是9.21 （1%的错误率）
            if((errX*errX+errY*errY)>9.21){
                landmark.second->is_outlier_ = true;
                outlier++;
                break;
            }
        }
    }    
    return outlier;
}


