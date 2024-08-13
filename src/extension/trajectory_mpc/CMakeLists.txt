cmake_minimum_required(VERSION 3.5)
project(trajectory_mpc)
set(CMAKE_BUILD_TYPE debug)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(car_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# 查找 Protocol Buffers 库
find_package(Protobuf REQUIRED)



include_directories(
  include
  proto
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

File(GLOB MAIN_PROTO ${CMAKE_CURRENT_SOURCE_DIR}/proto/*)

add_executable(${PROJECT_NAME}
  include/trajectory_mpc.hpp
  src/main.cpp
  src/trajectory_mpc.cpp
  ${MAIN_PROTO})

# 添加重要的头文件、库以及相关依赖
ament_target_dependencies(trajectory_mpc
  car_interfaces
  rclcpp)

target_link_libraries(trajectory_mpc ${catkin_LIBRARIES} ipopt ${YAML_CPP_LIBRARIES} ${PROTOBUF_LIBRARIES})


install(TARGETS
trajectory_mpc
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION 
  share/${PROJECT_NAME}
)

ament_package()
