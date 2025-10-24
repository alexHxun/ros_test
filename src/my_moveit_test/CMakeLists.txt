cmake_minimum_required(VERSION 3.8)
project(my_moveit_test)

# ==================== 编译选项 ====================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ==================== 依赖 ====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# --- MoveIt 核心组件 ---
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# --- ROS 消息 ---
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- 其他依赖 ---
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io surface filters features kdtree)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  /opt/ros/humble/include/moveit
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ==================== 可执行文件 ====================
add_executable(my_moveit_test
  src/main.cpp
  src/auto_planner.cpp
  src/rd_waypoints.cpp
)

ament_target_dependencies(my_moveit_test
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  shape_msgs
  trajectory_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(my_moveit_test
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# ==================== 安装 ====================
install(TARGETS my_moveit_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ==================== 测试支持 ====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
