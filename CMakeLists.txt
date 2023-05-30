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
  "srv/Um6Reset.srv"
  "srv/Um7Reset.srv"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(um6_driver src/um6_driver.cpp)
target_compile_features(um6_driver PUBLIC c_std_99 cxx_std_14)  # Require C99 and C++14

target_sources(um6_driver
  PRIVATE
  src/um6_registers.cpp
  src/um6_comms.cpp
)

ament_target_dependencies(um6_driver
  geometry_msgs
  rclcpp
  sensor_msgs
  serial
  std_msgs
)

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
target_link_libraries(um6_driver "${cpp_typesupport_target}")
target_link_libraries(um7_driver "${cpp_typesupport_target}")

#############
## Install ##
#############

install(TARGETS um6_driver
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS um7_driver
  DESTINATION lib/${PROJECT_NAME})

#############
## Testing ##
#############


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(um6_comms_test
      test/um6_test_comms.cpp
  )

  target_sources(um6_comms_test
    PRIVATE
    src/um6_registers.cpp
    src/um6_comms.cpp
  )

  ament_target_dependencies(um6_comms_test
    rclcpp
    serial
  )

  ament_add_gtest(um6_register_test
      test/um6_test_registers.cpp
  )

  target_sources(um6_register_test
    PRIVATE
    src/um6_registers.cpp
  )

  ament_target_dependencies(um6_register_test
    rclcpp
  )

  ament_add_gtest(um7_comms_test
      test/um7_test_comms.cpp
  )

  target_sources(um7_comms_test
    PRIVATE
    src/um7_registers.cpp
    src/um7_comms.cpp
  )

  ament_target_dependencies(um7_comms_test
    rclcpp
    serial
  )

  ament_add_gtest(um7_register_test
      test/um7_test_registers.cpp
  )

  target_sources(um7_register_test
    PRIVATE
    src/um7_registers.cpp
  )

  ament_target_dependencies(um7_register_test
    rclcpp
  )

  install(TARGETS um6_comms_test
    DESTINATION test/${PROJECT_NAME})

  install(TARGETS um6_register_test
    DESTINATION test/${PROJECT_NAME})

  install(TARGETS um7_comms_test
    DESTINATION test/${PROJECT_NAME})

  install(TARGETS um7_register_test
    DESTINATION test/${PROJECT_NAME})

endif()

ament_package()
