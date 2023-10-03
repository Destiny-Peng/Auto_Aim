# CMake 最低版本号要求
cmake_minimum_required(VERSION 3.10) 

# 工程名
project(communication)

# 指定C++标准
set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)
# 源文件目录
set(SRC_DIR src)

# 头文件目录
set(INCLUDE_DIR include)

# 源文件
set(SOURCES
  ${SRC_DIR}/communication.cpp
  ${SRC_DIR}/CamDrv.cpp
)
# 头文件 
set(HEADERS
  ${INCLUDE_DIR}/data.hpp
  ${INCLUDE_DIR}/serial.hpp
  ${INCLUDE_DIR}/coordinateTrans.hpp
  ${INCLUDE_DIR}/time_stamp.hpp
  ${INCLUDE_DIR}/uart.hpp
  ${INCLUDE_DIR}/Track.hpp
  ${INCLUDE_DIR}/LightBar.hpp
  ${INCLUDE_DIR}/Armor.hpp
  ${INCLUDE_DIR}/CamDrv.hpp
  ${INCLUDE_DIR}/PreProcess.hpp
  )
# 设置第三方库文件夹位置
set(OPENCV_DIR /usr/local/lib/)
# configure_file(config.h.in config.h)
find_package(OpenCV REQUIRED)
# 生成可执行文件



add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(communication ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} pthread ${OPENCV_LIBS} /lib/libMVSDK.so)

target_include_directories(communication PUBLIC ${OpenCV_INCLUDE_DIRS}/.)
include_directories(${OpenCV_INCLUDE_DIRS})

# 指定编译参数
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic)