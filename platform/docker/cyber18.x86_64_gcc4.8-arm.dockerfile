# FROM ubuntu:18.04
FROM arm64v8/ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive
RUN  sed -i s@/ports.ubuntu.com/@/mirrors.aliyun.com/@g /etc/apt/sources.list

RUN  apt-get clean

RUN apt-get update -y && \
    apt-get install -y \
    sudo \
    apt-transport-https \
    autotools-dev \
    automake \
    gcc-4.8 \
    g++-4.8 \
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
    iproute2 \
    iputils-ping \
    lcov \
    libblas-dev

RUN apt-get install -y \
    libssl-dev \
    libcurl4-openssl-dev \
    libfreetype6-dev \
    liblapack-dev \
    libpcap-dev \
    libpython3-dev \
    libsqlite3-dev \
    libgtest-dev \
    locate \
    lsof \
    net-tools \
    nfs-common \
    python-autopep8 \
    python3 \
    shellcheck \
    software-properties-common \
    sshfs \
    subversion \
    unzip \
    uuid-dev \
    v4l-utils \
    vim \
    wget \
    zip

RUN apt-get install -y \
    libasound2-dev \
    libasio-dev \
    libtinyxml2-6 \
    libtinyxml2-dev \
    libncurses5-dev \
    libavcodec57 \
    libavcodec-dev \
    libswscale4 \
    libswscale-dev \
    libunwind-dev \
    libflann-dev \
    libqhull-dev \
    libpcap0.8 \
    libpcap0.8-dev \
    libusb-1.0-0 \
    libusb-1.0-0-dev

RUN apt-get install -y \
    libopenni0 \
    libopenni2-0 \
    libbz2-dev \
    liblz4-dev \
    libpoco-dev \
    libopenni-dev \
    libopenni2-dev \
    libncurses5-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    echo '\n\n\n' | ssh-keygen -t rsa

#install gcc 4.8.5
RUN rm -f /usr/bin/gcc
RUN ln -s /usr/bin/gcc-4.8 /etc/alternatives/gcc
RUN ln -s /etc/alternatives/gcc /usr/bin/gcc
RUN rm -f /usr/bin/g++
RUN ln -s /usr/bin/g++-4.8 /etc/alternatives/g++
RUN ln -s /etc/alternatives/g++ /usr/bin/g++

# Install Python deps
COPY installers_arm /tmp/installers
RUN bash /tmp/installers/install_python_modules.sh

RUN echo "deb http://mirrors.aliyun.com/ubuntu-ports/ trusty main" >> /etc/apt/sources.list
RUN echo "deb http://mirrors.aliyun.com/ubuntu-ports/ trusty universe" >> /etc/apt/sources.list

RUN apt-get clean
RUN apt-get update -y
RUN apt install -y --allow-downgrades\
    libkml-dev \
    libboost-system1.54.0 \
    libboost-thread1.54.0 \
    libboost-signals1.54.0 \
    libboost-filesystem1.54.0 \
    libboost-iostreams1.54.0 \
    libboost-chrono1.54.0 \
    libboost1.54-dev \
    libboost-timer1.54-dev \
    libboost-serialization1.54-dev \
    libboost-dev=1.54.0.1ubuntu1 \
    libgdal-dev \
    libvtk6-dev \
    libvtk6.3 \
    vtk6 \
    libpocofoundation9

RUN rm -f /usr/lib/libPocoFoundation.so
RUN ln -s /usr/lib/libPocoFoundation.so.9 /usr/lib/libPocoFoundation.so
RUN ln -s /usr/lib/python2.7/dist-packages/vtk/libvtkRenderingPythonTkWidgets.aarch64-linux-gnu.so /usr/lib/aarch64-linux-gnu/libvtkRenderingPythonTkWidgets.so

RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_iostreams.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_iostreams.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_date_time.so.1.65.1 /usr/lib/aarch64-linux-gnu/libboost_date_time.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.65.1 /usr/lib/aarch64-linux-gnu/libboost_regex.so
#RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_serialization.so.1.65.1 /usr/lib/aarch64-linux-gnu/libboost_serialization.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_signals.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_signals.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_system.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_system.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_thread.so
#RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_wserialization.so.1.65.1 /usr/lib/aarch64-linux-gnu/libboost_wserialization.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_chrono.so

RUN apt install -y --allow-downgrades\
    libopencv-core-dev=2.4.8+dfsg1-2ubuntu1 \
    libopencv-imgproc-dev=2.4.8+dfsg1-2ubuntu1 \
    libopencv-highgui-dev=2.4.8+dfsg1-2ubuntu1
RUN apt install -y libopencv-dev

# Install gflags.
RUN mkdir /data && cd /data && \
    wget https://github.com/gflags/gflags/archive/v2.2.0.tar.gz && \
    tar xzf v2.2.0.tar.gz && \
    mkdir gflags-2.2.0/build && \
    cd gflags-2.2.0/build && \
    CXXFLAGS="-fPIC" cmake -DBUILD_SHARED_LIBS=true .. && \
    make -j8 && \
    make install && \
    cd ..

# Install glog which also depends on gflags.
RUN cd /data && \
    wget https://github.com/google/glog/archive/v0.3.5.tar.gz && \
    tar xzf v0.3.5.tar.gz && \
    cd glog-0.3.5 && \
    ./configure --build=aarch64-unknown-linux-gnu --enable-shared && \
    make CXXFLAGS='-Wno-sign-compare -Wno-unused-local-typedefs -fPIC -D_START_GOOGLE_NAMESPACE_="namespace google {" -D_END_GOOGLE_NAMESPACE_="}" -DGOOGLE_NAMESPACE="google" -DHAVE_PTHREAD -DHAVE_SYS_UTSNAME_H -DHAVE_SYS_SYSCALL_H -DHAVE_SYS_TIME_H -DHAVE_STDINT_H -DHAVE_STRING_H -DHAVE_PREAD -DHAVE_FCNTL -DHAVE_SYS_TYPES_H -DHAVE_SYSLOG_H -DHAVE_LIB_GFLAGS -DHAVE_UNISTD_H' && \
    make install && \
    cd ..

RUN cd /data && \ 
    wget https://github.com/google/googletest/archive/release-1.8.0.tar.gz && \
    tar -xzf release-1.8.0.tar.gz && \
    cd /data/googletest-release-1.8.0/ && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_SHARED_LIBS=ON .. && \
    make -j12 && \
    cd /data/googletest-release-1.8.0 && \
    cp -a build/googlemock/gtest/*.so build/googlemock/libgmock.so /usr/lib/ && \
    cp -a googletest/include/gtest /usr/include && \
    cp -a googlemock/include/gmock /usr/include

RUN apt-get install -y \
    libvlfeat-dev

WORKDIR /neolix
