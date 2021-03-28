# FROM ubuntu:18.04
FROM autobot:aarch64

RUN  apt-get clean

COPY arm-cuda /tmp/arm-cuda
RUN dpkg -i  /tmp/arm-cuda/cuda-repo-l4t-10-0-local-10.0.326_1.0-1_arm64.deb
RUN apt-key add /var/cuda-repo-10-0-local-10.0.326/7fa2af80.pub

RUN apt-get update -y && \
    apt-get install -y \
    cuda-toolkit-10-0

RUN dpkg -i  /tmp/arm-cuda/libcudnn7_7.5.0.56-1+cuda10.0_arm64.deb
RUN dpkg -i /tmp/arm-cuda/libcudnn7-dev_7.5.0.56-1+cuda10.0_arm64.deb
RUN dpkg -i  /tmp/arm-cuda/libcudnn7-doc_7.5.0.56-1+cuda10.0_arm64.deb
RUN dpkg -i  /tmp/arm-cuda/libnvinfer5_5.1.6-1+cuda10.0_arm64.deb
RUN dpkg -i  /tmp/arm-cuda/libnvinfer-dev_5.1.6-1+cuda10.0_arm64.deb
RUN dpkg -i  /tmp/arm-cuda/libnvinfer-samples_5.1.6-1+cuda10.0_all.deb
RUN dpkg -i  /tmp/arm-cuda/tensorrt_5.1.6.1-1+cuda10.0_arm64.deb

RUN rm -fr /tmp/arm-cuda

WORKDIR /neolix
