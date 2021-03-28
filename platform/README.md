# platform
How to create platform env for neolix software

Using an arm docker image requires the following command to be executed after the system starts.    
docker run --rm --privileged docker/binfmt:66f9012c56a8316f9244ffd7622d7c21c1f6f28d   
        
1. Install docker

    please follow the step: https://docs.docker.com/v17.09/engine/installation/linux/docker-ce/ubuntu/#install-using-the-repository and don't miss post-install step: https://docs.docker.com/v17.09/engine/installation/linux/linux-postinstall/

2. Build up the docker
images:  cyberrt:gcc48
        cd /xxx/xxx/platform/docker
        docker build -t autobot:v0.1 -f x86_64.dockerfile .

        for gcc8 docker, could use command:
        docker build -t autobot:gcc8 -f cyber18.x86_64_gcc8.dockerfile .

        for arm docker
        docker build -t autobot:aarch64 -f cyber18.x86_64_gcc4.8-arm.dockerfile .

3. Start docker with docker.sh

    At first, make sure gears, platform, and to-be-build component are under the same path, which will be value of following working_path:

        bash docker.sh $component_name $working_path  
        
        docker_gcc4.8和docker_gcc8在101行修改了镜像名称，请各位自行修改为第2步骤中-t后输入的名称。   

        for gcc8 docker, first copy platform/docker to the same folder of your program:
        /tmp/project/
                ├── common
                ├── control
                ├── cyber
                └── docker
                        ├── apollo_base.sh
                        ├── cyber18.x86_64_gcc8.dockerfile
                        ├── docker_adduser.sh
                        ├── docker_gcc8_into.sh
                        ├── docker_gcc8.sh
                        └── docker_into.sh
        enter into "/tmp/project" build docker with the command:
                bash docker/docker_gcc8.sh gcc8 /tmp/project/
	or  
   		bash docker/docker_gcc4.8.sh gcc4.8 /tmp/project/  
        or    
                bash docker/docker_gcc4.8.arm.sh armgcc4.8 /tmp/project/         

4. Enter into docker

        bash docker_into.sh  
        or  
        bash docker_gcc4.8_into.arm.sh  

        docker_gcc8_into.sh和docker_gcc4.8_into.sh使用前需要修改22行-it autobot_new_wrll为第三步输入的component_name和username。     

        进入docker后需要重新编译未在此版本docker下编译的依赖库，比如fast-rtps、protobuf等。

5. Then you can run your build now.
