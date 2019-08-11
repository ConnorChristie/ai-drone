Initial CMAKE command:
```shell
$ docker run -v D:\Git\drone-c++:/root/build armv7_openvino cmake -DCMAKE_TOOLCHAIN_FILE=/root/arm-openvino.toolchain.cmake -BbuildArm -H.
```

Build command:
```shell
$ docker run -v D:\Git\drone-c++:/root/build armv7_openvino make -j7 -C buildArm
```

Windows run command:
```shell
$ drone.exe -d MYRIAD -m "D:\Git\drone-c++\models\Transportation\object_detection\vehicle\mobilenet-reduced-ssd\dldt\FP16\vehicle-detection-adas-0002.xml" -i D:\Git\drone\car_small.mp4
```

Linux run command:
```shell
$ LD_LIBRARY_PATH=lib ./drone /dev/serial0
```