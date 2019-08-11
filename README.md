## Running
```shell
$ cd /Users/connor/Documents/git/deep-learning/drone
```

```shell
$ docker run -v /Users/connor/Documents/git/deep-learning/drone:/root/build armv7_openvino cmake -DCMAKE_TOOLCHAIN_FILE=/root/arm-openvino.toolchain.cmake -BbuildArm -H.
```

```shell
$ docker run -v /Users/connor/Documents/git/deep-learning/drone:/root/build armv7_openvino make -j7 -C buildArm
```