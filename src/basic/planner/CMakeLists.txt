cmake_minimum_required(VERSION 3.5)
project(planner)
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
find_package(car_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OsqpEigen)
find_package(Eigen3)

set(PLANNER_DEPS
  rclcpp
  car_interfaces
)

include_directories(
  include
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


add_executable(${PROJECT_NAME}
  include/planner.hpp
  include/planner_tools.hpp
  src/main.cpp
  src/planner.cpp
  src/planner_tools.cpp)

# add_executable(${PROJECT_NAME}
#   src/test_osqp.cpp)

ament_target_dependencies(planner
  car_interfaces
  ${PLANNER_DEPS})

target_link_libraries(planner ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)


install(TARGETS
  planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION 
  share/${PROJECT_NAME}
)

ament_package()
