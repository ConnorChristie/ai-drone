SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_VERSION 1)
SET(CMAKE_SYSTEM_PROCESSOR armv7l)

SET(FLOAT_ABI_SUFFIX "hf")

SET(GCC_COMPILER_VERSION "" CACHE STRING "GCC Compiler version")
SET(GNU_MACHINE "arm-linux-gnueabihf" CACHE STRING "GNU compiler triple")

# specify the cross compiler
SET(CMAKE_C_COMPILER   /usr/bin/arm-linux-gnueabihf-gcc)
SET(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabihf-g++)

# where is the target environment
SET(CMAKE_FIND_ROOT_PATH  /inference_engine_vpu_arm/inference_engine/share /inference_engine_vpu_arm/inference_engine /inference_engine_vpu_arm/opencv)

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
