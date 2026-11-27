cmake_minimum_required(VERSION 3.5)
project(insta360_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(imu_tools REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# Optional Insta360 MediaSDK (realtime stitcher, media_sdk_stitcher).
# MediaSDK >= 3.2 is installed with `sudo apt install ./MediaSDK-<version>-linux-amd64.deb`
# into /opt/MediaSDK-<version>-linux. That runtime already carries an $ORIGIN RPATH and its own
# CUDA runtime, so nothing is extracted or patched here. Its headers ship separately in the SDK
# archive (include/) and go in include/media_sdk. Without both, media_sdk_stitcher is skipped.
set(INSTA360_MEDIA_SDK_ROOT "" CACHE PATH
  "Insta360 MediaSDK prefix containing lib/libMediaSDK.so (default: newest /opt/MediaSDK-*-linux)")
file(GLOB MEDIA_SDK_ROOTS LIST_DIRECTORIES true "/opt/MediaSDK-*-linux")
if(MEDIA_SDK_ROOTS)
  list(SORT MEDIA_SDK_ROOTS COMPARE NATURAL ORDER DESCENDING)
endif()
# NO_DEFAULT_PATH: never fall back to /usr, where the legacy libmediasdk-dev 3.1.x package puts
# headers and a libMediaSDK.so that are incompatible with the 3.2 API used by the stitcher.
find_path(MEDIA_SDK_INCLUDE_DIR ins_realtime_stitcher.h
  HINTS "${PROJECT_SOURCE_DIR}/include/media_sdk" "${INSTA360_MEDIA_SDK_ROOT}/include"
  NO_DEFAULT_PATH)
find_library(MEDIA_SDK_LIBRARY MediaSDK
  HINTS ${INSTA360_MEDIA_SDK_ROOT} ${MEDIA_SDK_ROOTS} PATH_SUFFIXES lib
  NO_DEFAULT_PATH)
if(MEDIA_SDK_INCLUDE_DIR)
  # The stitcher uses CameraInfo::SetCalibration(), introduced with the 3.2 headers.
  file(STRINGS "${MEDIA_SDK_INCLUDE_DIR}/ins_common.h" MEDIA_SDK_HAS_SET_CALIBRATION REGEX "SetCalibration")
  if(NOT MEDIA_SDK_HAS_SET_CALIBRATION)
    message(WARNING "MediaSDK headers in ${MEDIA_SDK_INCLUDE_DIR} predate MediaSDK 3.2 "
      "(no CameraInfo::SetCalibration); media_sdk_stitcher will not be built")
    set(MEDIA_SDK_INCLUDE_DIR "")
  endif()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraPreviewInfo.msg"
  "msg/ExposureData.msg"
  "msg/CameraCompressedStreamData.msg"
  "srv/ClockSync.srv"
  DEPENDENCIES std_msgs sensor_msgs
)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_search_module(SWSCALE REQUIRED libswscale)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)
# NVIDIA hardware acceleration (nvh264enc, nvh264dec) is provided at runtime by gstreamer1.0-plugins-bad.
# Requires NVIDIA GPU with NVENC/NVDEC support and nvidia-container-toolkit on the host.
# Run the container with: docker run --gpus all --runtime=nvidia ...

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIRS}
  ${AVFORMAT_INCLUDE_DIRS}
  ${AVUTIL_INCLUDE_DIRS}
  ${SWSCALE_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
)

link_directories(
  ${PROJECT_SOURCE_DIR}/lib
  ${PROJECT_SOURCE_DIR}/include
  ${GSTREAMER_LIBRARY_DIRS}
)

install(
  DIRECTORY config launch lib
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.deb" EXCLUDE
)

install(
  DIRECTORY include/
  DESTINATION include
)

add_executable(${PROJECT_NAME}_node src/main.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rosidl_default_runtime
  camera_info_manager
  cv_bridge
  image_transport
  std_msgs
  std_srvs
  OpenCV
)
target_link_libraries(${PROJECT_NAME}_node CameraSDK  ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVFORMAT_LIBRARIES} ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")

# libCameraSDK.so is installed to share/${PROJECT_NAME}/lib (install(DIRECTORY ... lib) above);
# without this a non-symlink install cannot find it at runtime.
set(CAMERA_SDK_INSTALL_RPATH "$ORIGIN/../../share/${PROJECT_NAME}/lib")
set_target_properties(${PROJECT_NAME}_node PROPERTIES INSTALL_RPATH "${CAMERA_SDK_INSTALL_RPATH}")

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(ffmpeg_decoder src/ffmpeg_decoder.cpp)
ament_target_dependencies(ffmpeg_decoder
  rclcpp
  camera_info_manager
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  std_srvs
  OpenCV
  rosidl_default_runtime
)
target_link_libraries(ffmpeg_decoder CameraSDK  ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVFORMAT_LIBRARIES} ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES})
target_link_libraries(ffmpeg_decoder "${cpp_typesupport_target}")
set_target_properties(ffmpeg_decoder PROPERTIES INSTALL_RPATH "${CAMERA_SDK_INSTALL_RPATH}")

install(TARGETS ffmpeg_decoder
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(gstreamer_encoder src/gstreamer_encoder.cpp src/gstreamer_ros_sink.cpp)
ament_target_dependencies(gstreamer_encoder rclcpp cv_bridge sensor_msgs std_msgs OpenCV)
target_link_libraries(gstreamer_encoder ${OpenCV_LIBRARIES} ${GSTREAMER_LIBRARIES})

add_executable(gstreamer_decoder src/gstreamer_decoder.cpp)
ament_target_dependencies(gstreamer_decoder rclcpp cv_bridge sensor_msgs std_msgs OpenCV rosidl_default_runtime)
target_link_libraries(gstreamer_decoder ${GSTREAMER_LIBRARIES})
target_link_libraries(gstreamer_decoder "${cpp_typesupport_target}")

if(MEDIA_SDK_INCLUDE_DIR AND MEDIA_SDK_LIBRARY)
  get_filename_component(MEDIA_SDK_LIB_DIR "${MEDIA_SDK_LIBRARY}" DIRECTORY)
  message(STATUS "Insta360 MediaSDK: ${MEDIA_SDK_LIBRARY} (headers: ${MEDIA_SDK_INCLUDE_DIR})")
  add_executable(media_sdk_stitcher src/media_sdk_stitcher.cpp)
  ament_target_dependencies(media_sdk_stitcher
    rclcpp sensor_msgs geometry_msgs cv_bridge rosidl_default_runtime)
  target_include_directories(media_sdk_stitcher PRIVATE ${MEDIA_SDK_INCLUDE_DIR})
  target_compile_definitions(media_sdk_stitcher PRIVATE
    MEDIA_SDK_MODEL_DIR="${MEDIA_SDK_LIB_DIR}/../bin/models/")
  # CMAKE_DL_LIBS: dlsym/dladdr1 in src/stabilization_patch.hpp
  target_link_libraries(media_sdk_stitcher
    ${MEDIA_SDK_LIBRARY} pthread ${CMAKE_DL_LIBS} "${cpp_typesupport_target}")
  # libMediaSDK.so resolves its bundled dependencies through its own $ORIGIN RPATH.
  set_target_properties(media_sdk_stitcher PROPERTIES INSTALL_RPATH "${MEDIA_SDK_LIB_DIR}")
  install(TARGETS media_sdk_stitcher DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "Insta360 MediaSDK not found: media_sdk_stitcher will not be built "
    "(install the MediaSDK .deb and copy its headers to include/media_sdk, see README)")
endif()

add_executable(webcam_equirectangular src/webcam_equirectangular.cpp)
ament_target_dependencies(webcam_equirectangular rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(webcam_equirectangular ${OpenCV_LIBRARIES})

# Add C++ image conversion nodes
add_executable(dual_fisheye2equirectangular_cpp src/dual_fisheye2equirectangular.cpp)
ament_target_dependencies(dual_fisheye2equirectangular_cpp
  rclcpp
  rcl_interfaces
  cv_bridge
  sensor_msgs
  std_msgs
  OpenCV
)
add_executable(dual_fisheye2perspective_cpp src/dual_fisheye2perspective.cpp)
ament_target_dependencies(dual_fisheye2perspective_cpp
  rclcpp
  rcl_interfaces
  cv_bridge
  sensor_msgs
  std_msgs
  OpenCV
  tf2_geometry_msgs
  geometry_msgs
)
add_executable(equirectangular2perspective_cpp src/equirectangular2perspective.cpp)
ament_target_dependencies(equirectangular2perspective_cpp
  rclcpp
  rcl_interfaces
  cv_bridge
  sensor_msgs
  std_msgs
  OpenCV
  tf2_geometry_msgs
  geometry_msgs
)
target_link_libraries(dual_fisheye2equirectangular_cpp ${OpenCV_LIBRARIES})
target_link_libraries(dual_fisheye2perspective_cpp ${OpenCV_LIBRARIES} Eigen3::Eigen)
target_link_libraries(equirectangular2perspective_cpp ${OpenCV_LIBRARIES} Eigen3::Eigen)

install(TARGETS
  dual_fisheye2equirectangular_cpp
  dual_fisheye2perspective_cpp
  equirectangular2perspective_cpp
  gstreamer_encoder
  gstreamer_decoder
  webcam_equirectangular
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/equirectangular.py
  scripts/calibrate.py
  scripts/calibrate_cpp.py
  scripts/find_stabilizer_offset.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
