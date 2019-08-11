FROM dockcross/linux-armv7

ENV DEFAULT_DOCKCROSS_IMAGE armv7_openvino

RUN apt-get install -y --no-install-recommends \
    libgtk-3-dev:armhf

RUN wget --no-check-certificate https://download.01.org/opencv/2019/openvinotoolkit/l_openvino_toolkit_raspbi_p_2019.1.094.tgz && \
    mkdir -p /opt/intel/openvino && \
    tar -xf l_openvino_toolkit_raspbi_p_<version>.tgz --strip 1 -C /opt/intel/openvino
