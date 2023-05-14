cmake_minimum_required(VERSION 3.8)
project(umx_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)


###########
## Build ##
###########

include_directories(
  include
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Um7Reset.srv"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(um7_driver src/um7_driver.cpp)
target_compile_features(um7_driver PUBLIC c_std_99 cxx_std_14)  # Require C99 and C++14

target_sources(um7_driver
  PRIVATE
  src/um7_registers.cpp
  src/um7_comms.cpp
)

ament_target_dependencies(um7_driver
  geometry_msgs
  rclcpp
  sensor_msgs
  serial
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(um7_driver "${cpp_typesupport_target}")

#############
## Install ##
#############

install(TARGETS um7_driver
  DESTINATION lib/${PROJECT_NAME})

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
