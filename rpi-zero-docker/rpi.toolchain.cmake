set( CMAKE_SYSTEM_NAME Linux )
set( CMAKE_SYSTEM_VERSION 1 )
set( CMAKE_SYSTEM_PROCESSOR arm )
set( CMAKE_LIBRARY_ARCHITECTURE arm-linux-gnueabihf )
set( FLOAT_ABI_SUFFIX "hf" )

SET(CMAKE_CROSSCOMPILING True)
set(CMAKE_COMPILER_IS_RASPBERRY_CROSS_COMPILER ON)

SET(GCC_COMPILER_VERSION "" CACHE STRING "GCC Compiler version")
SET(GNU_MACHINE "arm-linux-gnueabihf" CACHE STRING "GNU compiler triple")

# cross compiler tools
set(CC_RPI_GCC arm-linux-gnueabihf-gcc)
set(CC_RPI_GXX arm-linux-gnueabihf-g++)

# specify the cross compiler
SET(CMAKE_C_COMPILER   ${CC_RPI_GCC})
SET(CMAKE_CXX_COMPILER ${CC_RPI_GXX})

SET(BOOST_ROOT /root/boost_1_70_0)
set(Boost_ARCHITECTURE "-a32")

# where is the target environment
SET(CMAKE_FIND_ROOT_PATH /usr/lib/arm-linux-gnueabihf)
SET(CMAKE_FIND_ROOT_PATH /root/inference_engine/share /root/inference_engine /root/opencv-4.1.1 ${BOOST_ROOT}/stage)

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
