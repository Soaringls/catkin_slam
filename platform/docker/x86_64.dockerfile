FROM ubuntu:14.04
RUN  sed -i s@/archive.ubuntu.com/@/mirrors.aliyun.com/@g /etc/apt/sources.list
RUN apt update -y && \
    apt install -y \
    apt-transport-https \
    bc \
    build-essential \
    cmake \
    cppcheck \
    curl \
    curlftpfs \
    debconf-utils \
    doxygen \
    gdb \
    git \
    google-perftools \
    graphviz \
    lcov \
    libblas-dev \
    libboost-all-dev \
    libcurl4-openssl-dev \
    libfreetype6-dev \
    liblapack-dev \
    libpcap-dev \
    libsqlite3-dev \
    tcl-vtk python-vtk libvtk-java \
    locate \
    lsof \
    nfs-common \
    realpath \
    shellcheck \
    software-properties-common \
    sshfs \
    subversion \
    unzip \
    v4l-utils \
    vim \
    wget \
    zip && \
    echo '\n\n\n' | ssh-keygen -t rsa

RUN apt update -y && \
    apt install -y\
    libeigen3-dev \
    libiomp-dev \
    zlib1g-dev \
    libssl-dev \
    python-dev \
    libconsole-bridge-dev \
    libbz2-dev liblz4-dev \
    libyaml-cpp-dev \
    libgoogle-glog-dev \
    libpoco-dev \
    libopencv-dev \
    libopenni-dev \
    libproj-dev \
    libpcap-dev \
    libusb-dev \
    'libqhull*' \
    node \
    libboost-all-dev \
    python-yaml \
    software-properties-common \
    libflann-dev \
    libvtk6-dev \
    python-software-properties

# install gflags glog gtest
RUN apt-get install -y \
    libgflags-dev \
    libgoogle-glog-dev \
    libgtest-dev \
    google-perftools \
    libgoogle-perftools-dev

RUN cd /usr/src/gtest/ && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_SHARED_LIBS=ON .. && \
    make && \
    cp libgtest* /usr/lib/ && \
    cp libgtest* /usr/local/lib/ && \
    cd .. && \
    rm -rf build

COPY src /tmp/src
RUN cd /tmp/src/ && \
    tar -xf eigen3.tar.gz && \
    sudo cp -r eigen3 /usr/include

