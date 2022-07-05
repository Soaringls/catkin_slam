单目离线建图版
===

#### 0、依赖

##### (1)OpenCV

OpenCV version 3.3.1

##### (2)Eigen

Eigen version 3.3.5

##### (3)Ceres

Ceres version: 1.14.0

##### (4)Sophus

##### (5)Glog

glog version: 0.3.5

##### (6)GFlags

gflags version: 2.2.2


#### 1、kitti数据

启动：rosbag play -r 0.03 ~/data/kitti_loop.bag

#### 2、rviz界面

配置文件地址：

config/voRvizConfig.rviz

#### 3、roslog 中间输出数据显示

rosrun rqt_console rqt_console

rosrun rqt_logger_level rqt_logger_level

#### 4、运行 SLAM

rosrun vo_project vo_odometry_node

