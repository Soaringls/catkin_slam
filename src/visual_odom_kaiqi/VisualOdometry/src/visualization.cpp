#include "visualization.h"

ros::Publisher pub_image_track;
ros::Publisher pub_image_BA;
ros::Publisher pub_cur_point;
ros::Publisher pub_map_point;
ros::Publisher pub_camera_pose;
ros::Publisher pub_global_path;
ros::Publisher pub_odometry;
ros::Publisher pub_keyframe_pose;
ros::Publisher pub_extrinsic;
ros::Publisher pub_path;


nav_msgs::Path path;


// register topic
void registerPub(ros::NodeHandle &n){

    pub_image_track = n.advertise<sensor_msgs::Image>("image_track", 1000);
    pub_image_BA = n.advertise<sensor_msgs::Image>("image_BA", 1000);
    pub_cur_point = n.advertise<sensor_msgs::PointCloud>("cur_point_cloud", 1000);
    pub_map_point = n.advertise<sensor_msgs::PointCloud>("map_point_cloud", 1000);
    pub_camera_pose = n.advertise<nav_msgs::Odometry>("camera_pose", 1000);
    pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    pub_extrinsic = n.advertise<nav_msgs::Odometry>("extrinsic", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);

}



void pubTrackImage(std_msgs::Header &header, const cv::Mat &image){
    header.frame_id = "vo_world";
    header.stamp = ros::Time(ros::Time::now()); 
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    pub_image_track.publish(imgTrackMsg);
}

void pubBAImage(std_msgs::Header &header, const cv::Mat &image){
    header.frame_id = "vo_world";
    header.stamp = ros::Time(ros::Time::now()); 
    sensor_msgs::ImagePtr imgBAMsg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    pub_image_BA.publish(imgBAMsg);
}


void pubCameraPose(const std_msgs::Header &header, const Eigen::Vector3d Ps, const Eigen::Matrix3d Rs){
        Eigen::Vector3d P = Ps;
        Eigen::Quaterniond R = Quaterniond(Rs);

        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "vo_world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_camera_pose.publish(odometry);
}

void pubKeyframePose(const std_msgs::Header &header, const Eigen::Vector3d Ps, const Eigen::Matrix3d Rs){
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "vo_world";
        Quaterniond tmp_Q(Rs);
        odometry.pose.pose.position.x = Ps.x();
        odometry.pose.pose.position.y = Ps.y();
        odometry.pose.pose.position.z = Ps.z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();


        pub_keyframe_pose.publish(odometry);
}

// 发布坐标系的相关消息
void pubTF(const std_msgs::Header &header, const Eigen::Vector3d Ps, const Eigen::Matrix3d Rs){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    Vec3 correct_t;
    Quaterniond correct_q;
    correct_t = Ps;  // w_T_c ,世界坐标系在camera坐标系下的位姿
    correct_q = Rs;

    transform.setOrigin(tf::Vector3(correct_t(0),
                                    correct_t(1),
                                    correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "vo_world", "camera_link"));

  
    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "vo_world";
    odometry.pose.pose.position.x = 0;
    odometry.pose.pose.position.y = 0;
    odometry.pose.pose.position.z = 0;
   
    odometry.pose.pose.orientation.x = 0;
    odometry.pose.pose.orientation.y = 0;
    odometry.pose.pose.orientation.z = 0;
    odometry.pose.pose.orientation.w = 1;
    pub_extrinsic.publish(odometry);
}



// 发布里程计消息
void pubOdometry(const std_msgs::Header &header, const Eigen::Vector3d Ps, const Eigen::Matrix3d Rs){
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "vo_world";
        odometry.child_frame_id = "vo_world";
        Quaterniond tmp_Q(Rs);
        odometry.pose.pose.position.x = Ps.x();
        odometry.pose.pose.position.y = Ps.y();
        odometry.pose.pose.position.z = Ps.z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        // odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        // odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        // odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        pub_odometry.publish(odometry);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "vo_world";
        pose_stamped.pose = odometry.pose.pose;
        path.header = header;
        path.header.frame_id = "vo_world";
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);
}




void pubPointCloud(std_msgs::Header header, const vector<Eigen::Vector3d> point3d){
    sensor_msgs::PointCloud point_cloud;
    header.frame_id = "vo_world";
    point_cloud.header = header;

    for (auto feature : point3d)
    {
        //Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];
        geometry_msgs::Point32 p;
        p.x = feature(0);
        p.y = feature(1);
        p.z = feature(2);
        point_cloud.points.push_back(p);
    }
    pub_cur_point.publish(point_cloud);

}

//显示所有地图点
void pubMapPoint(std_msgs::Header header, const vector<Eigen::Vector3d> point3d){
    sensor_msgs::PointCloud point_cloud;
    header.frame_id = "vo_world";
    point_cloud.header = header;

    for (auto feature : point3d)
    {
        //Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];
        geometry_msgs::Point32 p;
        p.x = feature(0);
        p.y = feature(1);
        p.z = feature(2);
        point_cloud.points.push_back(p);
    }
    pub_map_point.publish(point_cloud);

}


void pubPath(const std_msgs::Header header, const Eigen::Vector3d Ps, const Eigen::Matrix3d Rs){

    //  Eigen::Vector3d P = Ps;
    //  Eigen::Quaterniond R = Quaterniond(Rs);

    // static nav_msgs::Path path;

    // geometry_msgs::PoseStamped pose_stamped;
    // pose_stamped.header = header;
    // pose_stamped.pose.position.x = P.x();
    // pose_stamped.pose.position.y = P.y();
    // pose_stamped.pose.position.z = P.z();
    // pose_stamped.pose.orientation.x = R.x();
    // pose_stamped.pose.orientation.y = R.y();
    // pose_stamped.pose.orientation.z = R.z();
    // pose_stamped.pose.orientation.w = R.w();

    // path.header = pose_stamped.header;
    // path.poses.push_back(pose_stamped);
    
    // pub_global_path.publish(path);
}

void pubKeyframePose(const std_msgs::Header header, const Eigen::Vector3d Ps[], const Eigen::Matrix3d Rs[]){
     Eigen::Vector3d P = Ps[0];
     Eigen::Quaterniond R = Quaterniond(Rs[0]);

    static nav_msgs::Path path;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.pose.position.x = P.x();
    pose_stamped.pose.position.y = P.y();
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = R.x();
    pose_stamped.pose.orientation.y = R.y();
    pose_stamped.pose.orientation.z = R.z();
    pose_stamped.pose.orientation.w = R.w();

    path.header = pose_stamped.header;
    path.poses.push_back(pose_stamped);
    
    pub_global_path.publish(path);
}
















