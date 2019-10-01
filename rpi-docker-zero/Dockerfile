FROM ubuntu:18.04

RUN apt-get update
RUN apt-get install -y build-essential gawk gcc g++ gfortran git texinfo bison libncurses-dev
RUN apt-get install -y --no-install-recommends wget

WORKDIR /root

RUN wget --no-check-certificate https://sourceforge.net/projects/raspberry-pi-cross-compilers/files/Raspberry%20Pi%20GCC%20Cross-Compiler%20Toolchains/GCC%209.1.0/Raspberry%20Pi%201%2C%20Zero/cross-gcc-9.1.0-pi_0-1.tar.gz
RUN apt-get install -y --no-install-recommends bsdtar
RUN bsdtar xf cross-gcc-9.1.0-pi_0-1.tar.gz && rm cross-gcc-9.1.0-pi_0-1.tar.gz

ENV PATH=/root/cross-pi-gcc-9.1.0-0/bin:$PATH

RUN apt-get install -y --no-install-recommends unzip

RUN wget --no-check-certificate https://github.com/opencv/opencv/archive/3.2.0.zip
RUN unzip 3.2.0.zip && rm 3.2.0.zip

RUN wget --no-check-certificate https://github.com/opencv/opencv_contrib/archive/3.2.0.zip
RUN unzip 3.2.0.zip && rm 3.2.0.zip

RUN apt-get install -y --no-install-recommends cmake

COPY rpi.toolchain.cmake .

RUN mkdir opencv-build && cd opencv-build && cmake \
    -DCMAKE_BUILD_TYPE=RELEASE \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_DOCS=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_opencv_flann=OFF \ 
    -DBUILD_opencv_ml=OFF \ 
    -DBUILD_opencv_surface_matching=OFF \ 
    -DBUILD_opencv_dnn=OFF \ 
    -DBUILD_opencv_fuzzy=OFF \ 
    -DBUILD_opencv_shape=OFF \ 
    -DBUILD_opencv_objdetect=OFF \ 
    -DBUILD_opencv_plot=OFF \ 
    -DBUILD_opencv_superres=OFF \ 
    -DBUILD_opencv_xobjdetect=OFF \ 
    -DBUILD_opencv_xphoto=OFF \ 
    -DBUILD_opencv_bgsegm=OFF \ 
    -DBUILD_opencv_bioinspired=OFF \ 
    -DBUILD_opencv_dpm=OFF \ 
    -DBUILD_opencv_face=OFF \ 
    -DBUILD_opencv_features2d=OFF \ 
    -DBUILD_opencv_line_descriptor=OFF \ 
    -DBUILD_opencv_saliency=OFF \ 
    -DBUILD_opencv_calib3d=OFF \ 
    -DBUILD_opencv_ccalib=OFF \ 
    -DBUILD_opencv_datasets=OFF \ 
    -DBUILD_opencv_stereo=OFF \ 
    -DBUILD_opencv_tracking=OFF \ 
    -DBUILD_opencv_videostab=OFF \ 
    -DBUILD_opencv_xfeatures2d=OFF \ 
    -DBUILD_opencv_ximgproc=OFF \ 
    -DBUILD_opencv_aruco=OFF \ 
    -DBUILD_opencv_optflow=OFF \ 
    -DBUILD_opencv_phase_unwrapping=OFF \ 
    -DBUILD_opencv_stitching=OFF \ 
    -DBUILD_opencv_structured_light=OFF \
    -DOPENCV_EXTRA_MODULES_PATH=/root/opencv_contrib-3.2.0/modules \
    -DCMAKE_TOOLCHAIN_FILE=/root/rpi.toolchain.cmake \ 
    /root/opencv-3.2.0

RUN make --jobs=$(nproc --all) -C opencv-build

RUN wget https://www.cmake.org/files/v3.14/cmake-3.14.3.tar.gz && \
    tar xf cmake-3.14.3.tar.gz && \
    (cd cmake-3.14.3 && ./bootstrap --parallel=$(nproc --all) && make --jobs=$(nproc --all) && make install) && \
    rm -rf cmake-3.14.3 cmake-3.14.3.tar.gz

RUN git clone https://github.com/opencv/dldt.git

# cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=/root/rpi.toolchain.cmake -DTHREADS_PTHREAD_ARG="-pthread" -DENABLE_SSE42=OFF -DTHREADING=SEQ -DENABLE_GNA=OFF ..

# Install Boost
# RUN wget --no-check-certificate https://sourceforge.net/projects/boost/files/boost/1.70.0/boost_1_70_0.tar.bz2
# RUN tar xjf boost_1_70_0.tar.bz2

# RUN cd boost_1_70_0 && \
#     ./bootstrap.sh --prefix=/root/boost && \
#     echo "using gcc : arm : arm-linux-gnueabihf-g++ ;" > ~/user-config.jam && \
#     ./b2 -j 6 --with-atomic --with-chrono --with-date_time --with-filesystem --with-regex --with-thread --with-system --no-samples --no-tests toolset=gcc-arm threading=multi link=static runtime-link=static --layout=tagged --build-type=complete stage

# RUN apt-get install -y --no-install-recommends libopencv-dev

WORKDIR /root/build
