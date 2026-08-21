cmake_minimum_required(VERSION 3.5)
project(hikrobot_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  /opt/MVS/include/
)

link_directories(/opt/MVS/lib/64/)

add_executable(${PROJECT_NAME}
  src/hikrobot_camera.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  cv_bridge
  image_transport
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  MvCameraControl
  MVGigEVisionSDK
  MVRender
  MvUsb3vTL
  MediaProcess
  X11
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()