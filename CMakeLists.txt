cmake_minimum_required(VERSION 3.5)
project(xwiimote_controller)

set (THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  wiimote_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
)

ament_export_dependencies(
	rclcpp rclcpp_lifecycle std_msgs std_srvs sensor_msgs geometry_msgs wiimote_msgs xwiimote
)

## Declare a C++ executable
add_executable(${PROJECT_NAME} src/xwiimote_controller.cpp)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

find_package(PkgConfig)
pkg_check_modules(LIBXWIIMOTE REQUIRED libxwiimote)

find_library(XWIIMOTE_LIB xwiimote)
if(XWIIMOTE_LIB)
  message(STATUS "Found xwiimote library. ${XWIIMOTE_LIB}")
else()
  message(FATAL_ERROR "xwiimote library not found.")
endif()


find_path(XWIIMOTE_INCLUDE_DIR xwiimote.h)
if(XWIIMOTE_INCLUDE_DIR)
  message(STATUS "Found xwiimote header. ${XWIIMOTE_INCLUDE_DIR}")
else()
  message(FATAL_ERROR "xwiimote header not found.")
endif()

# Not 100% sure if this is the correct way, but I just want it to link and then its good enough
target_link_libraries(${PROJECT_NAME} ${XWIIMOTE_LIB})
target_include_directories(${PROJECT_NAME} INTERFACE ${XWIIMOTE_INCLUDE_DIR})


#############
## Install ##
#############

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
#install(
#  DIRECTORY launch/
#  DESTINATION share/${PROJECT_NAME}/launch
#)

# Install config files
#install(
#  DIRECTORY config/
#  DESTINATION share/${PROJECT_NAME}/config
#)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
