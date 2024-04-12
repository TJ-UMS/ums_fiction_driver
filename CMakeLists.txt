cmake_minimum_required(VERSION 3.8)
project(ums_fiction_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/src/serial)


# 指定共享库的位置
# 检查目标CPU架构
if(CMAKE_SYSTEM_PROCESSOR MATCHES "amd64|x86_64|AMD64")
  # 如果为amd64
  message(STATUS "Configuring for amd64")
  link_directories(lib/x86)
  set(UMS_SDK ${CMAKE_SOURCE_DIR}/lib/x86/libums_sdk.so)
  set(SERIAL ${CMAKE_SOURCE_DIR}/lib/x86/libserial.so)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|ARM64|aarch64")
  # 如果为arm64
  message(STATUS "Configuring for arm64")
  link_directories(lib/arm64)
  set(UMS_SDK ${CMAKE_SOURCE_DIR}/lib/arm64/libums_sdk.so)
  set(SERIAL ${CMAKE_SOURCE_DIR}/lib/arm64/libserial.so)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# 其他CMake配置...

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
add_executable(ums_fiction_driver_node src/ums_fiction_driver_node.cpp)
ament_target_dependencies(ums_fiction_driver_node rclcpp geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
target_include_directories(ums_fiction_driver_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
# 添加链接库
target_link_libraries(ums_fiction_driver_node
        ${UMS_SDK}
        ${SERIAL}
)
install(TARGETS ums_fiction_driver_node
        DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ${UMS_SDK} DESTINATION lib)
install(PROGRAMS ${SERIAL} DESTINATION lib)
install(DIRECTORY launch param script
        DESTINATION share/${PROJECT_NAME})

ament_package()
