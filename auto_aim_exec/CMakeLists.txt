cmake_minimum_required(VERSION 3.12)
project(auto_aim_exec)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#######################
## Find dependencies ##
#######################
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
add_compile_definitions(PROJECT_DIR="${CMAKE_MODULE_PATH}") #到ros2_ws的路径
message(${CMAKE_CURRENT_SOURCE_DIR})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_transport_plugins REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(CUDA REQUIRED)
find_package(TensorRT REQUIRED)
find_package(Eigen3 REQUIRED)

# User self-defined interface
find_package(auto_aim_interface REQUIRED)

# User set
set(YAML_INCLUDE /usr/local/include/yaml-cpp)
file(GLOB YAML_LIBS "/usr/local/lib/libyaml-cpp.a")
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.4)
file(GLOB CUDA_LIBS "/usr/local/cuda-11.4/lib64/lib*.so")
###########
## Build ##
###########
include_directories(include)

# mindvision_camera
set(MINDVISION mindvision)

add_library(${MINDVISION}_main SHARED
  src/mindvision_camera/mv_camera_node.cpp
)

target_include_directories(${MINDVISION}_main PUBLIC include mvsdk/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${MINDVISION}_main PUBLIC mvsdk/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(${MINDVISION}_main PUBLIC mvsdk/lib/arm64)
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

ament_target_dependencies(${MINDVISION}_main
  rclcpp sensor_msgs image_transport image_transport_plugins camera_info_manager OpenCV cv_bridge)

target_link_libraries(${MINDVISION}_main MVSDK)

rclcpp_components_register_node(${MINDVISION}_main
  PLUGIN mindvision_camera::MVCameraNode
  EXECUTABLE ${MINDVISION}_node
)
# end mindvision_camera

# target_detect
set(TARGET_DETECT target_detect)

add_library(${TARGET_DETECT}_tools SHARED src/target_detect/tools.cpp)
target_include_directories(${TARGET_DETECT}_tools PUBLIC include)
target_include_directories(${TARGET_DETECT}_tools PRIVATE ${CUDA_INCLUDE_DIRS})

add_library(${TARGET_DETECT}_main SHARED
  src/target_detect/target_detect_node.cpp
)
target_include_directories(${TARGET_DETECT}_main PRIVATE ${CUDA_INCLUDE_DIRS})
ament_target_dependencies(${TARGET_DETECT}_main
  rclcpp sensor_msgs image_transport auto_aim_interface image_transport_plugins camera_info_manager TensorRT CUDA cv_bridge)
target_link_libraries(${TARGET_DETECT}_main
  ${OpenCV_LIBS} 
  ${CUDA_LIBS} 
  ${TensorRT_LIBS} 
  ${TARGET_DETECT}_tools
)
rclcpp_components_register_node(${TARGET_DETECT}_main
  PLUGIN target_detect::TargetDetectNode
  EXECUTABLE ${TARGET_DETECT}_node
)
# end target_detect
# kalman filter
set(KALMAN kalman)
add_library(${KALMAN}_lib SHARED
  src/kalman/predict_kalman.cpp
)
target_include_directories(${KALMAN}_lib PUBLIC include)
target_include_directories(${KALMAN}_lib PRIVATE ${YAML_INCLUDE})
ament_target_dependencies(${KALMAN}_lib
  Eigen3
)
target_link_libraries(${KALMAN}_lib 
  ${OpenCV_LIBS}
  ${YAML_LIBS}
)
# end kalman filter
# post_process
set(POST_PROCESS post_process)

add_library(${POST_PROCESS}_utils SHARED 
  src/post_process/utils.cpp
)
target_include_directories(${POST_PROCESS}_utils PUBLIC include)
target_include_directories(${POST_PROCESS}_utils PRIVATE ${CUDA_INCLUDE_DIRS})
target_link_libraries(${POST_PROCESS}_utils
  ${OpenCV_LIBS}
)

add_library(${POST_PROCESS}_main SHARED 
  src/post_process/post_process.cpp
)
target_include_directories(${POST_PROCESS}_main PRIVATE ${CUDA_INCLUDE_DIRS} ${YAML_INCLUDE})
target_include_directories(${POST_PROCESS}_main PUBLIC include)

ament_target_dependencies(${POST_PROCESS}_main
  rclcpp sensor_msgs image_transport auto_aim_interface image_transport_plugins camera_info_manager CUDA cv_bridge
  Eigen3)

target_link_libraries(${POST_PROCESS}_main
  ${OpenCV_LIBS} 
  ${CUDA_LIBS} 
  ${YAML_LIBS}
  ${POST_PROCESS}_utils
  ${KALMAN}_lib
)

rclcpp_components_register_node(${POST_PROCESS}_main
  PLUGIN post_process::PostProcessNode
  EXECUTABLE ${POST_PROCESS}_node
)
# end post_process

# serial
set(SERIAL serial)

if(APPLE)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
endif()
if(UNIX AND NOT APPLE)
    # If Linux, add rt and pthread
    set(rt_LIBRARIES rt)
    set(pthread_LIBRARIES pthread)
endif()
## Sources
set(serial_SRCS
    src/serial/serial.cpp
    include/serial/serial.hpp
    include/serial/v8stdint.hpp
)
if(APPLE)
    # If OSX
    list(APPEND serial_SRCS src/serial/impl/unix.cpp)
    list(APPEND serial_SRCS src/serial/impl/list_ports/list_ports_osx.cpp)
elseif(UNIX)
    # If unix
    list(APPEND serial_SRCS src/serial/impl/unix.cpp)
    list(APPEND serial_SRCS src/serial/impl/list_ports/list_ports_linux.cpp)
else()
    # If windows
    list(APPEND serial_SRCS src/serial/impl/win.cpp)
    list(APPEND serial_SRCS src/serial/impl/list_ports/list_ports_win.cpp)
endif()

add_library(${SERIAL}_lib SHARED ${serial_SRCS})
if(APPLE)
    target_link_libraries(${SERIAL}_lib ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
    target_link_libraries(${SERIAL}_lib rt pthread)
else()
    target_link_libraries(${SERIAL}_lib setupapi)
endif()
target_include_directories(${SERIAL}_lib PUBLIC include)

add_library(${SERIAL}_main SHARED
  src/serial/Serial_node.cpp
)
target_include_directories(${SERIAL}_main PUBLIC include)
target_include_directories(${SERIAL}_main PRIVATE ${CUDA_INCLUDE_DIRS})
ament_target_dependencies(${SERIAL}_main
  rclcpp auto_aim_interface CUDA)
target_link_libraries(${SERIAL}_main
  ${SERIAL}_lib
  ${CUDA_LIBS} 
)
rclcpp_components_register_node(${SERIAL}_main
  PLUGIN serial::SerialNode
  EXECUTABLE ${SERIAL}_node
)
# end serial

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
  ament_cpplint(EXCLUDE ${_linter_excludes})
endif()

# ############
# # Install ##
# ############

install(TARGETS
  ${MINDVISION}_main
  ${POST_PROCESS}_utils
  ${POST_PROCESS}_main
  ${TARGET_DETECT}_tools
  ${TARGET_DETECT}_main
  ${SERIAL}_lib
  ${SERIAL}_main
  ${KALMAN}_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()