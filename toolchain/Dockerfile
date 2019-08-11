FROM dockcross/linux-armv7

ENV DEFAULT_DOCKCROSS_IMAGE armv7_openvino

RUN apt-get install -y --no-install-recommends \
    libgtk-3-dev:armhf

RUN wget --no-check-certificate https://download.01.org/opencv/2019/openvinotoolkit/R2/l_openvino_toolkit_runtime_raspbian_p_2019.2.242.tgz && \
    mkdir -p /opt/intel/openvino && \
    tar -xf l_openvino_toolkit_runtime_raspbian_p_2019.2.242.tgz --strip 1 -C /opt/intel/openvino
