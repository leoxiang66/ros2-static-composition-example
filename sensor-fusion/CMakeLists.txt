cmake_minimum_required(VERSION 3.8)
project(sensor-fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

add_definitions(-DBUILDING_ROS2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(PCL REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
ament_auto_find_build_dependencies()

# check apr
find_package(PkgConfig)
pkg_check_modules(APR apr-1)
if (APR_FOUND)
  message(${APR_INCLUDE_DIRS})
  message(${APR_LIBRARIES})
endif (APR_FOUND)

# Specify the paths for Hikvision API and camera libraries
find_library(HIKVISION_API_LIBRARY hikvision_api PATHS /usr/local/lib)
find_library(MVCAMERA_LIBRARY MvCameraControl PATHS $ENV{MVCAM_COMMON_RUNENV}/64)

## make sure the livox_lidar_sdk_shared library is installed
find_library(LIVOX_LIDAR_SDK_LIBRARY liblivox_lidar_sdk_shared.so /usr/local/lib REQUIRED)

##
# find_path(LIVOX_LIDAR_SDK_INCLUDE_DIR
# NAMES "livox_lidar_api.h" "livox_lidar_def.h"
# REQUIRED)

## PCL library
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Include directories for Hikvision API and other dependencies
include_directories(
  /usr/local/include
  ./include/hikvision
  ./include/livox
  ./include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



add_executable(sensors 
src/nodes.cpp 
src/hikvision/image_publisher.cpp 
src/hikvision/utils.cpp
src/livox/livox_ros_driver2.cpp
src/livox/lddc.cpp
src/livox/driver_node.cpp
src/livox/lds.cpp
src/livox/lds_lidar.cpp
src/livox/comm/comm.cpp
src/livox/comm/ldq.cpp
src/livox/comm/semaphore.cpp
src/livox/comm/lidar_imu_data_queue.cpp
src/livox/comm/cache_index.cpp
src/livox/comm/pub_handler.cpp
src/livox/parse_cfg_file/parse_cfg_file.cpp
src/livox/parse_cfg_file/parse_livox_lidar_cfg.cpp
src/livox/call_back/lidar_common_callback.cpp
src/livox/call_back/livox_lidar_callback.cpp
)

ament_target_dependencies(sensors 
rclcpp 
std_msgs
cv_bridge
image_transport
sensor_msgs
)

# Manually link additional libraries
target_link_libraries(sensors
  ${HIKVISION_API_LIBRARY}
  ${MVCAMERA_LIBRARY}
  pthread
  ${LIVOX_LIDAR_SDK_LIBRARY}
  ${PPT_LIBRARY}
  ${Boost_LIBRARY}
  ${PCL_LIBRARIES}
  ${APR_LIBRARIES}
)


# Install the executable
install(TARGETS sensors
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
