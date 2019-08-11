FROM dockcross/linux-armv7 as dockcross
FROM debian:stretch

USER root

RUN dpkg --add-architecture armhf && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    crossbuild-essential-armhf \
    cmake \
    pkg-config \
    wget \
    libgtk-3-dev:armhf

WORKDIR /root

# Install Inference Engine
RUN wget --no-check-certificate https://download.01.org/opencv/2019/openvinotoolkit/R2/l_openvino_toolkit_runtime_raspbian_p_2019.2.242.tgz
RUN mkdir -p /opt/intel/openvino && \
    tar -xf l_openvino_toolkit_runtime_raspbian_p_2019.2.242.tgz --strip 1 -C /opt/intel/openvino

COPY arm-openvino.toolchain.cmake .

WORKDIR /root/build
