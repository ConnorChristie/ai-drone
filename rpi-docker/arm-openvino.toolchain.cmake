SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_VERSION 1)
SET(CMAKE_SYSTEM_PROCESSOR armv7l)
SET(CMAKE_CROSSCOMPILING True)
set(CMAKE_COMPILER_IS_RASPBERRY_CROSS_COMPILER ON)

SET(FLOAT_ABI_SUFFIX "hf")

SET(GCC_COMPILER_VERSION "" CACHE STRING "GCC Compiler version")
SET(GNU_MACHINE "arm-linux-gnueabihf" CACHE STRING "GNU compiler triple")

# cross compiler tools
set(CC_RPI_GCC /usr/bin/arm-linux-gnueabihf-gcc)
set(CC_RPI_GXX /usr/bin/arm-linux-gnueabihf-g++)
set(CC_RPI_LIBS /usr/lib/arm-linux-gnueabihf)

# specify the cross compiler
SET(CMAKE_C_COMPILER   ${CC_RPI_GCC})
SET(CMAKE_CXX_COMPILER ${CC_RPI_GXX})

# where is the target environment
SET(CMAKE_FIND_ROOT_PATH /opt/intel/openvino/inference_engine/share /opt/intel/openvino/inference_engine /opt/intel/openvino/opencv /root/boost_1_59_0/stage)

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
