cmake_minimum_required(VERSION 3.5)
project(visualization)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()



# 찾기 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 실행 파일 추가 및 링크 라이브러리
add_executable(local_visualization src/local_visualization.cpp)
ament_target_dependencies(local_visualization
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  tf2
  PCL
  OpenCV
  pcl_conversions
)
target_link_libraries(local_visualization
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(global_visualization src/global_visualization.cpp)
ament_target_dependencies(global_visualization
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  tf2
  PCL
  OpenCV
  pcl_conversions
)
target_link_libraries(global_visualization
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# 설치 및 테스트 설정
install(TARGETS
  local_visualization
  # global_visualization
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
