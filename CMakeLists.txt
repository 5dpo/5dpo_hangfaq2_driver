cmake_minimum_required(VERSION 3.5)
project(sdpo_hangfaq2_driver)



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



# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sdpo_drivers_interfaces REQUIRED)
find_package(sdpo_serial_port REQUIRED)
find_package(serial_communication_channels REQUIRED)



# Node Executable
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}/SdpoHangfaQ2.cpp
  src/${PROJECT_NAME}/SdpoHangfaQ2DriverROS.cpp
  src/${PROJECT_NAME}/SerialChannelsConfig.cpp
  src/main.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sdpo_drivers_interfaces
  sdpo_serial_port
  serial_communication_channels
)



# Node Executable Installation Rules
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)



# Launch Files Installation Rules
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)



# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
