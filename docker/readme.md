# set the docker environment
```sh
#step1. 修改docker.sh中所使用镜像名称  目录映射   进入docker后默认目录(option)
bash docker.sh <docker-name> <work-local-path>   #docker.sh所在目录下执行
#e.g bash docker.sh ubuntu16-slam /home/lyu/dexter/bak_gitopen/slam_code/catkin_slam/
#将形成名为autobot_ubuntu16-slam_$USER的容器, 

bash dockerinto.sh
```