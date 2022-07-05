#include "Optimizer.h"
#include<mutex>




 

   
/**
* @brief 进行全局BA优化，但主要功能还是调用 BundleAdjustment,这个函数相当于加了一个壳.
* @param[in] pMap          地图对象的指针
* @param[in] nIterations   迭代次数
* @param[in] pbStopFlag    外界给的控制GBA停止的标志位
* @param[in] nLoopKF       当前回环关键帧的id，其实也就是参与GBA的关键帧个数
* @param[in] bRobust       是否使用鲁棒核函数
*/
void Optimizer::GlobalBundleAdjustemnt(int nIterations){





}

/**
 * @brief bundle adjustment 优化过程
 * Step 1:  构建代价函数
 * Step 2:  构建优化问题
 * Step 3:  配置求解器         
 * 
 * @param[in] vpKFs                 参与BA的所有关键帧
 * @param[in] vpMP                  参与BA的所有地图点
 * @param[in] nIterations           优化迭代次数
 * @param[in] pbStopFlag            外部控制BA结束标志
 * @param[in] nLoopKF               形成了闭环的当前关键帧的id
 * @param[in] bRobust               是否使用核函数
 */
void Optimizer::BundleAdjustment(MAP::KeyframesType &key1frames,
                                 MAP::LandmarksType &landmarks, 
                                 int nIterations){
    //     // Step 1:构建代价函数 
        
    //     // Step 2:构建优化问题 
    // 	ceres::Problem problem;

    //     vector<CeresMappoint>  multiMappoint;
    //     vector<CeresPose>  multiPose;

    //  for (auto &landmark : landmarks) {
    //         if (landmark.second->is_outlier_) continue;
    //         auto observations = landmark.second->GetObs();
    //         for (auto &obs : observations) {
    //             if (obs.lock() == nullptr) continue;
    //             if (obs.lock() == firstFrame) continue; // 第一帧不优化
    //             auto feat = obs.lock();
    //             if (feat->is_outlier_ || feat->frame_.lock() == nullptr) continue;
    //             auto frame = feat->frame_.lock();
    //             double camera[6];
    //             double mappoint[3];
    //             // 设置帧位姿 
    //             SE3 curPose = frame->Pose();      //??
    //             Mat44 T_ = current_frame_->pose_.matrix(); //??
    //             Mat33 R_ = T_.block(0,0,3,3);
    //             Vec3 t_ = T_.block(0,3,3,1);
    //             Eigen::AngleAxisd R_vector(R_);
    //             camera[0] = R_vector(0);
    //             camera[1] = R_vector(1); 
    //             camera[2] = R_vector(2); 
    //             camera[3] = t_(0); 
    //             camera[4] = t_(1); 
    //             camera[5] = t_(2); 
    //             // 设置地图点
    //             Vec3 curMappoint = landmark.second->Pos();
    //             mappoint[0] = curMappoint[0];
    //             mappoint[1] = curMappoint[1];    
    //             mappoint[2] = curMappoint[2];
    //             vector<double *> MpsCams;

    //             // 将像素点转换到归一化平面
    //             Vec3 point = camera_left_->pixel2camera(Vec2(feat->pt_.x, feat->pt_.y));
                
    //             // 创建实例构造函数，同时输入观测数据
    // 			ceres::CostFunction* cost_function = ReprojectionError::Create(point(0),
    //                                                                            point(1));
    //     		// 增加残差块 地图点打到所有观测帧上的重投影
    //             problem.AddResidualBlock(cost_function, NULL, camera, mappoint);	 
    //         }
    //     for (auto &keyframe : keyframes) {
    //         auto kf = keyframe.second;



    //         vertices.insert({kf->keyframe_id_, vertex_pose});
    //     }



    //     // Step 3:  配置求解器
    //     ceres::Solver::Options options;
    //     options.linear_solver_type = ceres::DENSE_SCHUR;
    //     options.minimizer_progress_to_stdout = true;
    //     //options.max_solver_time_in_seconds = 0.2; //最大处理时间
    //     ceres::Solver::Summary summary;
    //     ceres::Solve(options, &problem, &summary);

    // 	if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03){
    // 		cout << "vision only BA converge" << endl;
    // 	} else {
    // 		cout << "vision only BA not converge " << endl;
    // 		return false;
    // 	}


    //     return true;
}
bool Optimizer::GlobalBA(Frame::Ptr currectFrame, Frame::Ptr initFrame,
                       MAP::KeyframesType &keyframes,
                       MAP::LandmarksType &landmarks,
                       int nIterations){

    TicToc t_BA;
    // Step 1:构建代价函数 
    
    // Step 2:构建重投影误差优化函数
	ceres::Problem problem;
    ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();// 四元数deta -- 参数化9
    // 定义problem->AddResidualBlock()函数中需要的Huber核函数
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);

    vector<std::shared_ptr<CeresMappoint>> multiMappoint;
    std::map<Frame::Ptr, std::shared_ptr<CeresFrame>> multiFrame;

    vector<cv::Point2f> RAWfeat;
    vector<cv::Point2f> BAfeat;
    double errorsum = 0;
    int blocknum = 0;
    // 遍历地图点
    for (auto &landmark : landmarks) {
        if(landmark.second->is_outlier_) continue; 
        std::shared_ptr<CeresMappoint> cpoint(new CeresMappoint);// 设置地图点
        Vec3 curMappoint = landmark.second->Pos();
        cpoint->position[0] = curMappoint(0);
        cpoint->position[1] = curMappoint(1);    
        cpoint->position[2] = curMappoint(2);
        cpoint->ptr = landmark.second;
        multiMappoint.push_back(cpoint);
        
        auto observations = landmark.second->GetObs();
        
        for (auto &obs : observations) { 
            if (obs.lock() == nullptr) continue;
            auto feat = obs.lock();
            auto pframe = feat->frame_.lock();
            //if (!pframe->is_keyframe_) continue; //仅优化关键帧
            
            //设置优化帧         
            std::shared_ptr<CeresFrame> ceresframe(new CeresFrame);
            SE3 cPose = pframe->Pose(); //Tcw
            Mat44 T_ = cPose.matrix();
            Mat33 R_ = T_.block(0,0,3,3);
            Vec3 t_ = T_.block(0,3,3,1);
            Eigen::Quaterniond q_(R_);
            ceresframe->camera_r[0] = q_.w();
            ceresframe->camera_r[1] = q_.x(); 
            ceresframe->camera_r[2] = q_.y(); 
            ceresframe->camera_r[3] = q_.z();
            ceresframe->camera_t[0] = t_(0); 
            ceresframe->camera_t[1] = t_(1); 
            ceresframe->camera_t[2] = t_(2); 
            ceresframe->ptr = pframe;
            // 如果当前frame还没有被加入优化，则新加一个
            if (multiFrame.find(pframe) == multiFrame.end()) {
                multiFrame.insert({pframe, ceresframe}); 
            } else {
                multiFrame[pframe]->common_point += 1;
            }
            
            // 设置、固定参数块
            problem.AddParameterBlock(multiFrame[pframe]->camera_r, 4, local_parameterization); //指定参数化
            problem.AddParameterBlock(multiFrame[pframe]->camera_t, 3); 
            problem.SetParameterBlockConstant(multiFrame[pframe]->camera_r);
            problem.SetParameterBlockConstant(multiFrame[pframe]->camera_t); 


            //cout << "观测数据: "<< feat << " ||  "<<  (double)feat->pt_.x << " | "<< (double)feat->pt_.y << endl;
            // cout << "相机参数: "<< multiFrame[pframe] << " "<< cpoint << " ||  "<< T_  << " | "<< curMappoint.transpose() << endl;

            Vec3 w_pose2 = curMappoint;
            SE3 T_c_w2 = cPose;
            Vec3 c_pose2 = camera_left_->world2camera(w_pose2, T_c_w2);
            Vec2 ba_pose2 = camera_left_->camera2pixel(c_pose2);
            double error =  (feat->pt_.x-ba_pose2(0))*(feat->pt_.x-ba_pose2(0))  
                          + (feat->pt_.y-ba_pose2(1))*(feat->pt_.y-ba_pose2(1)); 
            errorsum += error;
            //cout << "观测数据: " <<  (double)feat->pt_.x << " | "<< (double)feat->pt_.y << " ||  " << error << endl;
            if(pframe == currectFrame){
            cv::Point2f qq((double)feat->pt_.x, (double)feat->pt_.y);
            cv::Point2f qqq(ba_pose2(0), ba_pose2(1));
            RAWfeat.push_back(qq);
            BAfeat.push_back(qqq);
            }

            Vec3 efeat = camera_left_->pixel2camera(Vec2(feat->pt_.x, feat->pt_.y));
            // 创建实例构造函数，同时输入观测数据
            ceres::CostFunction* cost_function = ReprojectionError::Create(efeat(0),
                                                                           efeat(1));
            // 增加残差块                 
            problem.AddResidualBlock(cost_function, loss_function, 
                                     multiFrame[pframe]->camera_r, 
                                     multiFrame[pframe]->camera_t, 
                                     cpoint->position);
            blocknum++;
        }
    }
    cout << "构建残差块数量: " << blocknum <<endl;
    //cout << "总误差: " << errorsum <<endl;
             

    //遍历帧列表，取消固定帧（待添加：将共视点大于15的帧取消固定）
    for(auto frame : multiFrame){
        if(/* frame.second->common_point > 15 && */ frame.first != initFrame){
            problem.SetParameterBlockVariable(frame.second->camera_r);
            problem.SetParameterBlockVariable(frame.second->camera_t);
        }
        //cout << "frame ID: " << frame.first->id_ << " | " <<  frame.second->common_point << endl;
    }

    // Step 3:  配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    //options.max_solver_time_in_seconds = 0.1; //最大处理时间
    options.max_num_iterations = 10;//最大迭代次数
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    ROS_INFO("GlobalBA costs: %f ms", t_BA.toc());

    //收敛成功目标函数
	if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03){
		ROS_INFO("GlobalBA converge, cost : %f", summary.final_cost);
        //更新pose 
        for(auto frame : multiFrame){
            //Eigen::Quaterniond(w,x,y,z)
            Eigen::Quaterniond convergeQ;
            convergeQ.w() = frame.second->camera_r[0]; 
            convergeQ.x() = frame.second->camera_r[1];
            convergeQ.y() = frame.second->camera_r[2]; 
            convergeQ.z() = frame.second->camera_r[3];
            Sophus::SO3d SconvergeR(convergeQ);
            Vec3 converget(frame.second->camera_t[0], 
                           frame.second->camera_t[1], 
                           frame.second->camera_t[2]); 
            frame.first->SetPose(SE3(SconvergeR, converget));
        }
        //更新地图点
        for(auto mappoint : multiMappoint){
            Vec3 pose(mappoint->position[0], mappoint->position[1], mappoint->position[2]);
            mappoint->ptr->SetPos(pose);
        }
	} else {
        ROS_INFO("BA not converge");
		return false;
	}

    return true;
}


bool Optimizer::LocalBA(Frame::Ptr currectFrame, Frame::Ptr initFrame,
                       MAP::KeyframesType &keyframes,
                       MAP::LandmarksType &landmarks,
                       int nIterations){
    TicToc t_BA;
    // Step 1:构建代价函数 
    
    // Step 2:构建重投影误差优化函数
	ceres::Problem problem;
    ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();// 四元数deta -- 参数化9
    // 定义problem->AddResidualBlock()函数中需要的Huber核函数
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);

    vector<std::shared_ptr<CeresMappoint>> multiMappoint;
    std::map<Frame::Ptr, std::shared_ptr<CeresFrame>> multiFrame;

    int blocknum = 0;
    // 遍历地图点
    for (auto &landmark : landmarks) {
        if(landmark.second->is_outlier_) continue; 
        std::shared_ptr<CeresMappoint> cpoint(new CeresMappoint);// 设置地图点
        Vec3 curMappoint = landmark.second->Pos();
        cpoint->position[0] = curMappoint(0);
        cpoint->position[1] = curMappoint(1);    
        cpoint->position[2] = curMappoint(2);
        cpoint->ptr = landmark.second;
        multiMappoint.push_back(cpoint);
        auto observations = landmark.second->GetObs();
        
        for (auto &obs : observations) { 
            if (obs.lock() == nullptr) continue;
            auto feat = obs.lock();
            auto pframe = feat->frame_.lock();
            //if (!pframe->is_keyframe_) continue; //仅优化关键帧
            
            //设置优化帧         
            std::shared_ptr<CeresFrame> ceresframe(new CeresFrame);
            SE3 cPose = pframe->Pose(); //Tcw
            Mat44 T_ = cPose.matrix();
            Mat33 R_ = T_.block(0,0,3,3);
            Vec3 t_ = T_.block(0,3,3,1);
            Eigen::Quaterniond q_(R_);
            ceresframe->camera_r[0] = q_.w();
            ceresframe->camera_r[1] = q_.x(); 
            ceresframe->camera_r[2] = q_.y(); 
            ceresframe->camera_r[3] = q_.z();
            ceresframe->camera_t[0] = t_(0); 
            ceresframe->camera_t[1] = t_(1); 
            ceresframe->camera_t[2] = t_(2); 
            ceresframe->ptr = pframe;
            // 如果当前frame还没有被加入优化，则新加一个
            if (multiFrame.find(pframe) == multiFrame.end()) {
                multiFrame.insert({pframe, ceresframe}); 
            } else {
                multiFrame[pframe]->common_point += 1;
            }
            
            // 先固定所有位姿变量固定
            problem.AddParameterBlock(multiFrame[pframe]->camera_r, 4, local_parameterization); //指定参数化
            problem.AddParameterBlock(multiFrame[pframe]->camera_t, 3); 
            problem.SetParameterBlockConstant(multiFrame[pframe]->camera_r);
            problem.SetParameterBlockConstant(multiFrame[pframe]->camera_t); 

            Vec3 efeat = camera_left_->pixel2camera(Vec2(feat->pt_.x, feat->pt_.y));
            // 创建实例构造函数，同时输入观测数据
            ceres::CostFunction* cost_function = ReprojectionError::Create(efeat(0),
                                                                           efeat(1));
            // 增加残差块                 
            problem.AddResidualBlock(cost_function, loss_function, 
                                     multiFrame[pframe]->camera_r, 
                                     multiFrame[pframe]->camera_t, 
                                     cpoint->position);
            blocknum++;
        }
    }
    cout << "构建残差块数量: " << blocknum <<endl;
    //遍历帧列表，取消固定帧（待添加：将共视点大于15的帧取消固定）
    for(auto frame : multiFrame){
        if( frame.second->common_point > 15 &&  frame.first != initFrame){
            problem.SetParameterBlockVariable(frame.second->camera_r);
            problem.SetParameterBlockVariable(frame.second->camera_t);
        }
        //cout << "frame ID: " << frame.first->id_ << " | " <<  frame.second->common_point << endl;
    }    

    // Step 3:  配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    //options.max_solver_time_in_seconds = 0.1; //最大处理时间
    options.max_num_iterations = nIterations;//最大迭代次数
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    ROS_INFO("LocalBA costs: %f ms", t_BA.toc());

    //收敛成功目标函数
	if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03){
		ROS_INFO("LocalBA converge, cost : %f", summary.final_cost);
        //更新pose 
        for(auto frame : multiFrame){
            //Eigen::Quaterniond(w,x,y,z)
            Eigen::Quaterniond convergeQ;
            convergeQ.w() = frame.second->camera_r[0]; 
            convergeQ.x() = frame.second->camera_r[1];
            convergeQ.y() = frame.second->camera_r[2]; 
            convergeQ.z() = frame.second->camera_r[3];
            Sophus::SO3d SconvergeR(convergeQ);
            Vec3 converget(frame.second->camera_t[0], 
                           frame.second->camera_t[1], 
                           frame.second->camera_t[2]); 
            frame.first->SetPose(SE3(SconvergeR, converget));
        }
        //更新地图点
        for(auto mappoint : multiMappoint){
            Vec3 pose(mappoint->position[0], mappoint->position[1], mappoint->position[2]);
            mappoint->ptr->SetPos(pose);
        }
	} else {
        ROS_INFO("BA not converge");
		return false;
	}

    return true;










}




bool Optimizer::InitBA(Frame::Ptr firstFrame, 
                       Frame::Ptr secondFrame, 
                       MAP::LandmarksType &landmarks,
                       int nIterations){

    TicToc t_BA;
    // Step 1:构建代价函数 
    
    // Step 2:构建优化问题  MapPoint -> 第二帧归一化平面的重投影误差
	ceres::Problem problem;
    ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();// 四元数deta -- 参数化
    // 设置帧位姿（第一帧）
    double camera0_r[4],camera0_t[3];
    camera0_r[0] = 1;
    camera0_r[1] = 0; 
    camera0_r[2] = 0; 
    camera0_r[3] = 0;
    camera0_t[0] = 0; 
    camera0_t[1] = 0; 
    camera0_t[2] = 0; 
    // 固定参数块
    problem.AddParameterBlock(camera0_r, 4, local_parameterization); //指定参数化
    problem.AddParameterBlock(camera0_t, 3); 
    problem.SetParameterBlockConstant(camera0_r);
    problem.SetParameterBlockConstant(camera0_t); 

    vector<CeresPose> multiPose;
    // 设置帧位姿（第二帧）
    double camera_r[4],camera_t[3];
    SE3 secondPose = secondFrame->Pose(); //Tcw
    Mat44 T_ = secondPose.matrix();
    Mat33 R_ = T_.block(0,0,3,3);
    Vec3 t_ = T_.block(0,3,3,1);
    Eigen::Quaterniond q_(R_);
    camera_r[0] = q_.w();
    camera_r[1] = q_.x(); 
    camera_r[2] = q_.y(); 
    camera_r[3] = q_.z();
    camera_t[0] = t_(0); 
    camera_t[1] = t_(1); 
    camera_t[2] = t_(2); 
    // 添加参数块
    problem.AddParameterBlock(camera_r, 4, local_parameterization); //指定参数化
    problem.AddParameterBlock(camera_t, 3);

    vector<CeresMappoint> multiMappoint;
    vector<Vec3>  multiFeature0;
    vector<Vec3>  multiFeature;
    for (auto &landmark : landmarks) {
        auto observations = landmark.second->GetObs();
        // 设置地图点
        Vec3 curMappoint = landmark.second->Pos();
        CeresMappoint cpoint;
        cpoint.position[0] = curMappoint(0);
        cpoint.position[1] = curMappoint(1);    
        cpoint.position[2] = curMappoint(2);

        cpoint.ptr = landmark.second;
        multiMappoint.push_back(cpoint);
        for (auto &obs : observations) {
            if (obs.lock() == nullptr) continue;
            auto feat = obs.lock();
            if (feat->frame_.lock() == firstFrame) {
                Vec3 eCurPoint0 = camera_left_->pixel2camera(
                    Vec2(feat->pt_.x, feat->pt_.y));
                multiFeature0.push_back(eCurPoint0);
            }
            if (feat->frame_.lock() == secondFrame) {
                Vec3 eCurPoint = camera_left_->pixel2camera(
                    Vec2(feat->pt_.x, feat->pt_.y));
                multiFeature.push_back(eCurPoint);
            }
        } 
    }  
    for (auto Mappoint : multiMappoint) {
        problem.AddParameterBlock(Mappoint.position, 3);
    } 
    // 定义problem->AddResidualBlock()函数中需要的Huber核函数
    ceres::LossFunction *loss_function = new ceres::CauchyLoss(1);
    //添加第一帧残差块
    for(size_t i = 0;i < multiFeature0.size();i++){
        // 创建实例构造函数，同时输入观测数据
        ceres::CostFunction* cost_function = ReprojectionError::Create((double)multiFeature0[i](0),
                                                                       (double)multiFeature0[i](1));
        // 增加残差块                 
        problem.AddResidualBlock(cost_function, loss_function, camera0_r, camera0_t, multiMappoint[i].position);
    }
    //添加第二帧残差块
    for(size_t i = 0;i <multiFeature.size();i++){
        ceres::CostFunction* cost_function = ReprojectionError::Create((double)multiFeature[i](0),
                                                                       (double)multiFeature[i](1));                
        problem.AddResidualBlock(cost_function, loss_function, camera_r, camera_t, multiMappoint[i].position);
    }

    // Step 3:  配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_solver_time_in_seconds = 0.1; //最大处理时间
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    ROS_INFO("BA costs: %f ms", t_BA.toc());

    //收敛成功目标函数
	if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03){
		ROS_INFO("BA converge, cost : %f", summary.final_cost);
        //更新pose 
        //Eigen::Quaterniond(w,x,y,z)
        Eigen::Quaterniond convergeQ;
        convergeQ.w() = camera_r[0]; 
		convergeQ.x() = camera_r[1]; 
		convergeQ.y() = camera_r[2]; 
		convergeQ.z() = camera_r[3]; 
        Sophus::SO3d SconvergeR(convergeQ);
        Vec3 converget(camera_t[0], camera_t[1], camera_t[2]); 
        secondFrame->SetPose(SE3(SconvergeR, converget));
        //更新地图点
        for(auto mappoint : multiMappoint){
            Vec3 pose(mappoint.position[0], mappoint.position[1], mappoint.position[2]);
            mappoint.ptr->SetPos(pose);
        }

	} else {
        ROS_INFO("BA not converge");
		return false;
	}



    return true;
}