set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER clang)
set(CMAKE_C_COMPILER_TARGET aarch64-linux-gnu)
set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_CXX_COMPILER_TARGET aarch64-linux-gnu)

#required for building, but is not included in search path by default
include_directories(SYSTEM /usr/aarch64-linux-gnu/include/c++/5/aarch64-linux-gnu)