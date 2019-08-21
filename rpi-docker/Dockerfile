FROM dockcross/linux-armv7 as dockcross
FROM debian:stretch

USER root

RUN dpkg --add-architecture armhf && apt-get update

RUN apt-get install -y --no-install-recommends wget
RUN apt-get install -y --no-install-recommends cmake
RUN apt-get install -y --no-install-recommends pkg-config
RUN apt-get install -y --no-install-recommends build-essential
RUN apt-get install -y --no-install-recommends crossbuild-essential-armhf
RUN apt-get install -y --no-install-recommends libgtk-3-dev:armhf

WORKDIR /root

# Install CMake
RUN wget --no-check-certificate https://www.cmake.org/files/v3.14/cmake-3.14.3.tar.gz && \
    tar xf cmake-3.14.3.tar.gz && \
    (cd cmake-3.14.3 && ./bootstrap --parallel=$(nproc --all) && make --jobs=$(nproc --all) && make install) && \
    rm -rf cmake-3.14.3 cmake-3.14.3.tar.gz

# Install Inference Engine
RUN wget --no-check-certificate https://download.01.org/opencv/2019/openvinotoolkit/R2/l_openvino_toolkit_runtime_raspbian_p_2019.2.242.tgz
RUN mkdir -p /opt/intel/openvino && \
    tar -xf l_openvino_toolkit_runtime_raspbian_p_2019.2.242.tgz --strip 1 -C /opt/intel/openvino

# Install Boost
RUN wget --no-check-certificate https://sourceforge.net/projects/boost/files/boost/1.70.0/boost_1_70_0.tar.bz2
RUN tar xjf boost_1_70_0.tar.bz2

RUN cd boost_1_70_0 && \
    ./bootstrap.sh --prefix=/root/boost && \
    echo "using gcc : arm : arm-linux-gnueabihf-g++ ;" > ~/user-config.jam && \
    ./b2 -j 6 --with-atomic --with-chrono --with-date_time --with-filesystem --with-regex --with-thread --with-system --no-samples --no-tests toolset=gcc-arm threading=multi link=static runtime-link=static --layout=tagged --build-type=complete stage

WORKDIR /root

COPY arm-openvino.toolchain.cmake .

WORKDIR /root/build
