# CMake 最低版本号要求
cmake_minimum_required(VERSION 3.10) 

# 工程名
project(communication)

# 指定C++标准
set(CMAKE_CXX_STANDARD 11)

# 源文件目录
set(SRC_DIR src)

# 头文件目录
set(INCLUDE_DIR include)

# 源文件
set(SOURCES
  ${SRC_DIR}/communication.cpp
)

# 头文件 
set(HEADERS
  ${INCLUDE_DIR}/data.hpp
  ${INCLUDE_DIR}/serial.hpp
)

# 生成可执行文件
add_executable(communication ${SOURCES} ${HEADERS})
target_link_libraries(communication pthread)

# 指定编译参数
target_compile_options(communication PRIVATE -Wall -Wextra -pedantic)