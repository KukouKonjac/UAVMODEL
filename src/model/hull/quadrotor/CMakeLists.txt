cmake_minimum_required(VERSION 3.15)

# 项目名称和版本
project(UAV_Controller VERSION 1.0)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_TOOLCHAIN_FILE "D:/vcpkg/scripts/buildsystems/vcpkg.cmake")
# 添加可执行文件
#add_executable(main1
#    main.cpp
#    PID.cpp 
#    uav_hover.cpp 
#    uav.cpp 
#)

# 查找 Eigen3
#find_package(Eigen3 CONFIG REQUIRED)
# 链接 Eigen3 库到目标
#target_link_libraries(main1 PRIVATE Eigen3::Eigen)

# 包含头文件目录
#target_include_directories(main1 PRIVATE ${CMAKE_SOURCE_DIR})
