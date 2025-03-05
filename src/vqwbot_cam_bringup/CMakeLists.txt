cmake_minimum_required(VERSION 3.8)
project(vqwbot_cam_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(cam_pan_tilt_hardware_interface REQUIRED)
find_package(camera_ros REQUIRED)

find_package(image_proc REQUIRED)

# Install directories
install(DIRECTORY
  launch
  params
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
