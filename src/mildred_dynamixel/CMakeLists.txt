cmake_minimum_required(VERSION 3.5)
project(mildred_dynamixel)

include(ExternalProject)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Dynamixel SDK
set(DYNAMIXEL_SDK_SUBMODULE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/dynamixel_sdk)
set(DYNAMIXEL_SDK_INCLUDE_DIRS ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/include)
if (UNIX AND NOT APPLE)
    set(DYNAMIXEL_SDK_SOURCES
        ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/src/dynamixel_sdk/packet_handler.cpp
        ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/src/dynamixel_sdk/protocol1_packet_handler.cpp
        ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/src/dynamixel_sdk/protocol2_packet_handler.cpp
        ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/src/dynamixel_sdk/group_sync_read.cpp
        ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/src/dynamixel_sdk/group_sync_write.cpp
        ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/src/dynamixel_sdk/group_bulk_read.cpp
        ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/src/dynamixel_sdk/group_bulk_write.cpp
        ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/src/dynamixel_sdk/port_handler.cpp
        ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/src/dynamixel_sdk/port_handler_linux.cpp
        )
elseif(UNIX AND APPLE)
    set(DYNAMIXEL_SDK_SOURCES
        ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/src/dynamixel_sdk/packet_handler.cpp
        ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/src/dynamixel_sdk/protocol1_packet_handler.cpp
        ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/src/dynamixel_sdk/protocol2_packet_handler.cpp
        ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/src/dynamixel_sdk/group_sync_read.cpp
        ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/src/dynamixel_sdk/group_sync_write.cpp
        ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/src/dynamixel_sdk/group_bulk_read.cpp
        ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/src/dynamixel_sdk/group_bulk_write.cpp
        ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/src/dynamixel_sdk/port_handler.cpp
        ${DYNAMIXEL_SDK_SUBMODULE_DIR}/c++/src/dynamixel_sdk/port_handler_mac.cpp
        )
endif()

## Dynamixel Workbench
set(DYNAMIXEL_WORKBENCH_SUBMODULE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/dynamixel_workbench)
set(DYNAMIXEL_WORKBENCH_INCLUDE_DIRS ${DYNAMIXEL_WORKBENCH_SUBMODULE_DIR}/dynamixel_workbench_toolbox/include)
set(DYNAMIXEL_WORKBENCH_SOURCES
    ${DYNAMIXEL_WORKBENCH_SUBMODULE_DIR}/dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox/dynamixel_item.cpp
    ${DYNAMIXEL_WORKBENCH_SUBMODULE_DIR}/dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox/dynamixel_driver.cpp
    ${DYNAMIXEL_WORKBENCH_SUBMODULE_DIR}/dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox/dynamixel_tool.cpp
    ${DYNAMIXEL_WORKBENCH_SUBMODULE_DIR}/dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox/dynamixel_workbench.cpp
    )

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
    NAMES yaml_cpp.h
    PATHS ${YAML_CPP_INCLUDE_DIRS}
    )
find_library(YAML_CPP_LIBRARY
    NAMES YAML_CPP
    PATHS ${YAML_CPP_LIBRARY_DIRS}
    )
link_directories(${YAML_CPP_LIBRARY_DIRS})

if (NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
    add_definitions(-DHAVE_NEW_YAMLCPP)
endif (NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

# Controller
add_executable(dynamixel_controller
    ${DYNAMIXEL_SDK_SOURCES}
    ${DYNAMIXEL_WORKBENCH_SOURCES}
    src/dynamixel_controller.cpp
    )

add_dependencies(dynamixel_controller
    ${PROJECT_NAME}
    )

target_include_directories(dynamixel_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${DYNAMIXEL_SDK_INCLUDE_DIRS}
    ${DYNAMIXEL_SDK_INCLUDE_DIRS}/dynamixel_sdk
    ${DYNAMIXEL_WORKBENCH_INCLUDE_DIRS}
    )

message(${DYNAMIXEL_SDK_INCLUDE_DIRS})

target_link_libraries(dynamixel_controller
    ${Eigen3_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    )

ament_target_dependencies(dynamixel_controller
    "rclcpp"
    "sensor_msgs"
    )

# Messages
rosidl_generate_interfaces(${PROJECT_NAME}
    msg/DynamixelInfo.msg
    msg/DynamixelState.msg
    msg/DynamixelStateList.msg
    msg/DynamixelLoadInfo.msg
    srv/DynamixelCommand.srv
    srv/GetDynamixelInfo.srv
    )

ament_export_dependencies(rosidl_default_runtime)

#rosidl_target_interfaces(dynamixel_controller ${PROJECT_NAME} "rosidl_typesupport_cpp")
get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)
get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

foreach (typesupport_impl ${typesupport_impls})
    rosidl_target_interfaces(dynamixel_controller ${PROJECT_NAME} ${typesupport_impl})
endforeach ()

# Install
install(TARGETS dynamixel_controller
    EXPORT export_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

install(FILES ${DYNAMIXEL_SDK_LIBRARIES} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    config
    launch
    DESTINATION share/${PROJECT_NAME}/
    )

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
