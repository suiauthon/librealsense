set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER clang)
set(CMAKE_C_COMPILER_TARGET arm-linux-gnueabihf)
set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_CXX_COMPILER_TARGET arm-linux-gnueabihf)

#required for building, but is not included in search path by default
include_directories(SYSTEM /usr/arm-linux-gnueabihf/include/c++/5/arm-linux-gnueabihf)