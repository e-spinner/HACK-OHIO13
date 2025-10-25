cmake_minimum_required(VERSION 3.8)
project(hack13)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --------------------
# Message generation
# --------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Angle.msg"
  "msg/Velocity.msg"
  "msg/AckState.msg"
  "msg/DiffState.msg"
  DEPENDENCIES builtin_interfaces
)

# ----------------
# C++ nodes
# ----------------

# Example
# add_executable(NAME src/PATH.cpp)
# ament_target_dependencies(
#   NAME
#   DEPS
# )


# Make executables depend on generated interfaces
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# target_link_libraries(NAME ${cpp_typesupport_target})


# --------------------
# Headers
# --------------------
# include_directories(
#   include
# )

# --------------------
# Install Nodes
# --------------------
# install(
#   TARGETS
#   DESTINATION lib/${PROJECT_NAME}
# )

# --------------------
# Install Extra Folders
# --------------------
# install(
#   DIRECTORY launch description
#   DESTINATION share/${PROJECT_NAME}
# )

# --------------------
# Python nodes
# --------------------
# find_package(ament_cmake_python REQUIRED)

# Install Python module
# ament_python_install_package(hack13_py)

# Install Python executables
install(PROGRAMS
  hack13_py/ack_calc.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()