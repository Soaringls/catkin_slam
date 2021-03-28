#!/usr/bin/env bash

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

INCHINA="no"
LOCAL_IMAGE="no"
VERSION=""
ARCH=$(uname -m)
VERSION_X86_64="ubuntu"
VERSION_OPT=""
COMPONENT_NAME=$1
WORK_PATH=$2

function show_usage()
{
cat <<EOF
Usage: $(basename $0) [options] ...
OPTIONS:
    -C                     Pull docker image from China mirror.
    -h, --help             Display this help and exit.
    -t, --tag <version>    Specify which version of a docker image to pull.
    -l, --local            Use local docker image.
    stop                   Stop all running Apollo containers.
EOF
exit 0
}


function stop_containers()
{
    running_containers=$(docker ps --format "{{.Names}}")

    for i in ${running_containers[*]}
    do
    if [[ "$i" =~ autobot_* ]];then
        printf %-*s 70 "stopping container: $i ..."
        docker stop $i > /dev/null
        if [ $? -eq 0 ];then
        printf "\033[32m[DONE]\033[0m\n"
        else
        printf "\033[31m[FAILED]\033[0m\n"
        fi
    fi
    done
}

AUTOBOT_ROOT_DIR="${WORK_PATH}"
echo $AUTOBOT_ROOT_DIR

if [ ! -e /autobot ]; then
    sudo ln -sf ${AUTOBOT_ROOT_DIR} /autobot
fi

while [ $# -gt 0 ]
do
    case "$1" in
    -C|--docker-cn-mirror)
        INCHINA="yes"
        ;;
    -t|--tag)
        VAR=$1
        [ -z $VERSION_OPT ] || echo -e "\033[093mWarning\033[0m: mixed option $VAR with $VERSION_OPT, only the last one will take effect.\n"
        shift
        VERSION_OPT=$1
        [ -z ${VERSION_OPT// /} ] && echo -e "Missing parameter for $VAR" && exit 2
        [[ $VERSION_OPT =~ ^-.* ]] && echo -e "Missing parameter for $VAR" && exit 2
        ;;
    -h|--help)
        show_usage
        ;;
    -l|--local)
        LOCAL_IMAGE="yes"
        ;;
    stop)
	stop_containers
	exit 0
	;;
    *)
        echo -e "\033[93mWarning\033[0m: Unknown option: $1"
        #exit 2
        ;;
    esac
    shift
done

#IMG=172.16.0.56:5000/cybertron:latest
IMG=soaringls/ubuntu16:191010

function local_volumes() {
    # Apollo root and bazel cache dirs are required.
    volumes="-v ${AUTOBOT_ROOT_DIR}:/home/caros/catkin_ws"
    #volumes="-v ${AUTOBOT_ROOT_DIR}:/autobot"
    # volumes="-v ${AUTOBOT_ROOT_DIR}/:/autobot/ \
	#      -v ${AUTOBOT_ROOT_DIR}/gears:/autobot/gears \
	#      -v ${AUTOBOT_ROOT_DIR}/cyber:/autobot/cyber \
    #      -v ${AUTOBOT_ROOT_DIR}/common_neolix/:/autobot/common_neolix/ \
	#      -v ${AUTOBOT_ROOT_DIR}/opt:/autobot/opt \
    #      -v ${AUTOBOT_ROOT_DIR}/ranger:/autobot/ranger \
    #      -v ${AUTOBOT_ROOT_DIR}/cyberverse:/autobot/cyberverse \
    #      -v ${AUTOBOT_ROOT_DIR}/data:/autobot/data "

    case "$(uname -s)" in
        Linux)
            case "$(lsb_release -r | cut -f2)" in
                14.04)
                    volumes="${volumes} "
                    ;;
                *)
                    volumes="${volumes} -v /dev:/dev "
                    ;;
            esac
            volumes="${volumes} -v /media:/media \
                                -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                                -v /etc/localtime:/etc/localtime:ro \
                                -v /usr/src:/usr/src \
                                -v /lib/modules:/lib/modules"
            ;;
    esac
    echo "${volumes}"
}

function main(){
    source apollo_base.sh
    info "Start pulling docker image $IMG ..."
    #docker pull $IMG
    #docker build -t autobot:v0.1 .
    if [ $? -ne 0 ];then
        error "Failed to pull docker image."
        exit 1
    fi

    AUTOBOT_CYBER="autobot_${COMPONENT_NAME}_${USER}"
    docker ps -a --format "{{.Names}}" | grep "$AUTOBOT_CYBER" 1>/dev/null
    if [ $? == 0 ]; then
        docker stop $AUTOBOT_CYBER 1>/dev/null
        docker rm -v -f $AUTOBOT_CYBER 1>/dev/null
    fi

    local display=""
    if [[ -z ${DISPLAY} ]];then
        display=":0"
    else
        display="${DISPLAY}"
    fi

    setup_device

    USER_ID=$(id -u)
    GRP=$(id -g -n)
    GRP_ID=$(id -g)
    LOCAL_HOST=`hostname`
    DOCKER_HOME="/home/$USER"
    if [ "$USER" == "root" ];then
        DOCKER_HOME="/root"
    fi
    if [ ! -d "$HOME/.cache" ];then
        mkdir "$HOME/.cache"
    fi

    info "Starting docker container \"${APOLLO_CYBER}\" ..."

    DOCKER_CMD="nvidia-docker"
    USE_GPU=1
    if ! [ -x "$(command -v ${DOCKER_CMD})" ]; then
        DOCKER_CMD="docker"
        USE_GPU=0
    fi

    ${DOCKER_CMD} run -it \
        -d \
        --privileged \
        --name $AUTOBOT_CYBER \
        -e DISPLAY=$display \
        -e DOCKER_USER=$USER \
        -e USER=$USER \
        -e DOCKER_USER_ID=$USER_ID \
        -e DOCKER_GRP="$GRP" \
        -e DOCKER_GRP_ID=$GRP_ID \
        -e DOCKER_IMG=$IMG \
        -e USE_GPU=$USE_GPU \
        -e OMP_NUM_THREADS=1 \
        $(local_volumes) \
        --net host \
        -w /autobot \
        --add-host autobot_docker:127.0.0.1 \
        --add-host ${LOCAL_HOST}:127.0.0.1 \
        --hostname autobot_docker \
        --shm-size 2G \
        --pid=host \
        -v /dev/null:/dev/raw1394 \
        $IMG \
        /bin/bash
    echo "${DOCKER_CMD} run -it \
        -d \
        --privileged \
        --name $AUTOBOT_CYBER \
        -e DISPLAY=$display \
        -e DOCKER_USER=$USER \
        -e USER=$USER \
        -e DOCKER_USER_ID=$USER_ID \
        -e DOCKER_GRP="$GRP" \
        -e DOCKER_GRP_ID=$GRP_ID \
        -e DOCKER_IMG=$IMG \
        -e USE_GPU=$USE_GPU \
        -e OMP_NUM_THREADS=1 \
        $(local_volumes) \
        --net host \
        -w /autobot \
        --add-host autobot_docker:127.0.0.1 \
        --add-host ${LOCAL_HOST}:127.0.0.1 \
        --hostname autobot_docker \
        --shm-size 2G \
        --pid=host \
        -v /dev/null:/dev/raw1394 \
        $IMG \
        /bin/bash"
    if [ $? -ne 0 ];then
        error "Failed to start docker container \"${AUTOBOT_CYBER}\" based on image: $IMG"
        exit 1
    fi

    if [ "${USER}" != "root" ]; then
        docker exec $AUTOBOT_CYBER bash -c '/autobot/docker/docker_adduser.sh'
    fi

    ok "Finished setting up Apollo docker environment. Now you can enter with: \nbash docker/docker_into.sh"
    ok "Enjoy!"
}

main
