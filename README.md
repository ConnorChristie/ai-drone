## Building for Raspberry PI using Docker

1. Build the base docker image

        docker build -t armv7_openvino ./rpi-docker

2. Run the initial CMake command

        docker run -v D:\Git\drone-c++:/root/build armv7_openvino cmake -DCMAKE_TOOLCHAIN_FILE=/root/arm-openvino.toolchain.cmake -BbuildArm -H.

3. Build the project

        docker run -v D:\Git\drone-c++:/root/build armv7_openvino make -j7 -C buildArm

## Running on Raspberry PI

1. [Install the OpenVINO™ Toolkit for Raspbian OS Package](https://docs.openvinotoolkit.org/latest/_docs_install_guides_installing_openvino_raspbian.html#install-package)

2. Copy the drone binary and libs folder (found in `buildArm\armv7l\Release`) over to the raspberry pi.

3. Copy any required inference models over. They should be placed in a folder named models right beside the executable.

4. Navigate to the directory that contains the binary and run the following command:

        LD_LIBRARY_PATH=lib ./drone \
        -m models/Transportation/object_detection/vehicle/mobilenet-reduced-ssd/dldt/FP16/vehicle-detection-adas-0002.xml \
        -ma models/Security/object_attributes/vehicle/resnet10_update_1/dldt/FP16/vehicle-attributes-recognition-barrier-0039.xml \
        -d MYRIAD \
        -msp_port_name /dev/serial0 \
        -i car_smaller.mp4

## Running on Windows

This is helpful for debugging since you can run the project from right within Visual Studio.

1. Create a `build` directory and run CMake.

        mkdir build && cd build
        cmake ..

2. Open `build/Drone.sln` with Visual Studio

    a. You should be able to build from within VS but running might be a bit tricky due to the necessary environment variables required for OpenVINO to run.  I normally just run the app from the command line anyways.

3. Run the command:

        drone.exe -m "models\Transportation\object_detection\vehicle\mobilenet-reduced-ssd\dldt\FP16\vehicle-detection-adas-0002.xml" -ma "models\Security\object_attributes\vehicle\resnet10_update_1\dldt\FP16\vehicle-attributes-recognition-barrier-0039.xml" -d MYRIAD -msp_port_name COM3 -i vids\car_480p.mp4
