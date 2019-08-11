Initial CMAKE command:
```shell
$ docker run -v D:\Git\drone-c++:/root/build armv7_openvino cmake -DCMAKE_TOOLCHAIN_FILE=/root/arm-openvino.toolchain.cmake -BbuildArm -H.
```

Build command:
```shell
$ docker run -v D:\Git\drone-c++:/root/build armv7_openvino make -j7 -C buildArm
```