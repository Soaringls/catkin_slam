#include "common_include.h"
#include "visual_odometry.h"
#include "parameters.h"

std::mutex m_buf;

std::queue<sensor_msgs::ImageConstPtr> leftimg_buf;
std::queue<sensor_msgs::ImageConstPtr> rightimg_buf;

VisualOdometry visualodometry;

//图像格式转化
cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg){
    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat img = ptr->image.clone();
    return img;
}

//图像回调函数
void leftimg_callback(const sensor_msgs::ImageConstPtr& msg)
{
    m_buf.lock();
    leftimg_buf.push(msg);
    m_buf.unlock();
}
void rightimg_callback(const sensor_msgs::ImageConstPtr& msg)
{
    m_buf.lock();
    rightimg_buf.push(msg);
    m_buf.unlock();
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        cv::Mat image_left, image_right;
        std_msgs::Header header;
        double time = 0;
        m_buf.lock();
        if (!leftimg_buf.empty() && !rightimg_buf.empty()){
            double time0 = leftimg_buf.front()->header.stamp.toSec();
            double time1 = rightimg_buf.front()->header.stamp.toSec();
            // 0.003s sync tolerance
            if(time0 < time1 - 0.003){
                leftimg_buf.pop();
                printf("throw img0\n");
            } else if(time0 > time1 + 0.003){
                rightimg_buf.pop();
                printf("throw img1\n");
            } else {
                time = leftimg_buf.front()->header.stamp.toSec();
                header = leftimg_buf.front()->header;
                image_left = getImageFromMsg(leftimg_buf.front());
                leftimg_buf.pop();
                image_right = getImageFromMsg(rightimg_buf.front());
                rightimg_buf.pop();
            }
        }
        m_buf.unlock();
        if(!image_left.empty()){
            visualodometry.Run(time, image_left, image_right);  
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}




int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Sensor_sub");
  ros::NodeHandle n;
  /*设置参数*/
  setParameter("./src/vo_project/config/params.yaml");

  /* 注册Pub */
  registerPub(n);

  /* 注册Sub */  ///                       /          话题名             //频率/  /    回调函数   / 
  ros::Subscriber leftimg = n.subscribe("/kitti/camera_color_left/image_raw", 100, leftimg_callback);    
  ros::Subscriber rightimg = n.subscribe("/kitti/camera_color_right/image_raw", 100, rightimg_callback);  

  std::thread sync_thread{sync_process};
  ros::spin();

}