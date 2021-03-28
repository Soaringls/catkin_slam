FROM ubuntu:14.04
LABEL maintainer "NVIDIA CORPORATION <cudatools@nvidia.com>"

RUN apt update && apt install -y --no-install-recommends \
ca-certificates apt-transport-https gnupg-curl && \
    NVIDIA_GPGKEY_SUM=d1be581509378368edeec8c1eb2958702feedf3bc3d17011adbf24efacce4ab5 && \
    NVIDIA_GPGKEY_FPR=ae09fe4bbd223a84b2ccfce3f60f4b3d7fa2af80 && \
    apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1404/x86_64/7fa2af80.pub && \
    apt-key adv --export --no-emit-version -a $NVIDIA_GPGKEY_FPR | tail -n +2 > cudasign.pub && \
    echo "$NVIDIA_GPGKEY_SUM  cudasign.pub" | sha256sum -c --strict - && rm cudasign.pub && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1404/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    apt purge --auto-remove -y gnupg-curl && \
rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 8.0.61

ENV CUDA_PKG_VERSION 8-0=$CUDA_VERSION-1

# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt update && apt install -y --no-install-recommends \
        cuda-cudart-$CUDA_PKG_VERSION \
cuda-nvrtc-$CUDA_PKG_VERSION \
        cuda-nvgraph-$CUDA_PKG_VERSION \
        cuda-cusolver-$CUDA_PKG_VERSION \
        cuda-cublas-8-0=8.0.61.2-1 \
        cuda-cufft-$CUDA_PKG_VERSION \
        cuda-curand-$CUDA_PKG_VERSION \
        cuda-cusparse-$CUDA_PKG_VERSION \
        cuda-npp-$CUDA_PKG_VERSION && \
ln -s cuda-8.0 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

# Required for nvidia-docker v1
LABEL com.nvidia.volumes.needed="nvidia_driver"
LABEL com.nvidia.cuda.version="${CUDA_VERSION}"

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=8.0 "

ENV DEBIAN_FRONTEND=noninteractive

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
    libgtest-dev \
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
    libgtest-dev \
    libgoogle-glog-dev \
    libpoco-dev \
    libopencv-dev \
    libproj-dev \
    libpcap-dev \
    node \
    libboost-all-dev \
    python-yaml \
    software-properties-common \
    python-software-properties

# install PCL
RUN add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/for-ros 
RUN apt update -y
RUN apt install -y libpcl-*

RUN apt install -y tcl-vtk \
    python-vtk \
    libvtk-java

# install gflags glog gtest and  gmock
RUN apt-get install -y \
    libgflags-dev \
    libgoogle-glog-dev \
    libgtest-dev \
    google-perftools \
    libgoogle-perftools-dev

RUN cd /usr/src/googletest/googletest && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    cp libgtest* /usr/lib/ && \
    cp libgtest* /usr/local/lib/ && \
    cd .. && \
    rm -rf build && \
    cd /usr/src/googletest/googlemock && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    cp libgmock* /usr/lib/ && \
    cp libgmock* /usr/local/lib/ && \
    cd .. && \
    rm -rf build
