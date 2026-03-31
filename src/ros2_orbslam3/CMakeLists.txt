cmake_minimum_required(VERSION 3.10)
project(ros2_orbslam3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Force system OpenCV 4 before cv_bridge finds OpenCV
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")
find_package(OpenCV 4 REQUIRED PATHS /usr/lib/x86_64-linux-gnu/cmake/opencv4 NO_DEFAULT_PATH)

find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenGL REQUIRED)

set(ORB_SLAM3_ROOT "/home/madhuram/Desktop/multi_robot_orb_ws/src/ros2_orbslam3/config/ORB_SLAM3")

set(ORB_SLAM3_INCLUDE_DIRS
  ${ORB_SLAM3_ROOT}
  ${ORB_SLAM3_ROOT}/include
  ${ORB_SLAM3_ROOT}/include/CameraModels
  ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
)

set(ORB_SLAM3_LIB ${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so)
set(DBOW2_LIB ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so)
set(G2O_LIB ${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so)

# Link Pangolin component libs directly
set(PANGO_CORE_LIB /usr/local/lib/libpango_core.so.0)
set(PANGO_DISPLAY_LIB /usr/local/lib/libpango_display.so.0)
set(PANGO_OPENGL_LIB /usr/local/lib/libpango_opengl.so.0)
set(PANGO_WINDOWING_LIB /usr/local/lib/libpango_windowing.so.0)
set(PANGO_IMAGE_LIB /usr/local/lib/libpango_image.so.0)
set(PANGO_VARS_LIB /usr/local/lib/libpango_vars.so.0)

add_compile_definitions(GL_GLEXT_PROTOTYPES)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${ORB_SLAM3_INCLUDE_DIRS}
)

add_executable(orbslam3_node
  src/ros2_orbslam3_node.cpp
)

ament_target_dependencies(orbslam3_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
)

target_link_libraries(orbslam3_node
  /lib/x86_64-linux-gnu/libopencv_core.so.406
  /lib/x86_64-linux-gnu/libopencv_imgproc.so.406
  /lib/x86_64-linux-gnu/libopencv_imgcodecs.so.406
  /lib/x86_64-linux-gnu/libopencv_highgui.so.406
  /lib/x86_64-linux-gnu/libopencv_features2d.so.406
  /lib/x86_64-linux-gnu/libopencv_calib3d.so.406
  /lib/x86_64-linux-gnu/libopencv_flann.so.406
  ${ORB_SLAM3_LIB}
  ${DBOW2_LIB}
  ${G2O_LIB}
  ${PANGO_CORE_LIB}
  ${PANGO_DISPLAY_LIB}
  ${PANGO_OPENGL_LIB}
  ${PANGO_WINDOWING_LIB}
  ${PANGO_IMAGE_LIB}
  ${PANGO_VARS_LIB}
  ${OPENGL_LIBRARIES}
)

target_include_directories(orbslam3_node PRIVATE
  ${ORB_SLAM3_INCLUDE_DIRS}
)

set_target_properties(orbslam3_node PROPERTIES
  BUILD_RPATH "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib;/usr/local/lib"
  INSTALL_RPATH "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib;/usr/local/lib"
)

install(
  TARGETS orbslam3_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
