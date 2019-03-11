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

#if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#    add_compile_options(-Wall -Wextra -Wpedantic)
#endif ()

# Dynamixel SDK
set(DYNAMIXEL_SDK_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/dynamixel_sdk/src/DynamixelSDK/c++/include)
set(DYNAMIXEL_SDK_LIBRARY_DIRS ${CMAKE_CURRENT_BINARY_DIR}/dynamixel_sdk/src/DynamixelSDK/c++/build/linux64/)
set(DYNAMIXEL_SDK_LIBRARIES ${CMAKE_CURRENT_BINARY_DIR}/dynamixel_sdk/src/DynamixelSDK/c++/build/linux64/libdxl_x64_cpp.so)
externalproject_add(DynamixelSDK
    GIT_REPOSITORY https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/dynamixel_sdk
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/dynamixel_sdk/src/DynamixelSDK/c++/build/linux64
    CONFIGURE_COMMAND ""
    BUILD_COMMAND make
    INSTALL_COMMAND ""
    BUILD_BYPRODUCTS ${DYNAMIXEL_SDK_LIBRARIES}
    LOG_DOWNLOAD 1
    LOG_BUILD 1
    )

## Dynamixel Workbench
set(DYNAMIXEL_WORKBENCH_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/lib/dynamixel_workbench/dynamixel_workbench_toolbox/include)
set(DYNAMIXEL_WORKBENCH_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/dynamixel_workbench/dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox/dynamixel_item.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/dynamixel_workbench/dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox/dynamixel_driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/dynamixel_workbench/dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox/dynamixel_tool.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/dynamixel_workbench/dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox/dynamixel_workbench.cpp
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
    ${DYNAMIXEL_WORKBENCH_SOURCES}
    src/dynamixel_controller.cpp
    )
add_dependencies(dynamixel_controller
    DynamixelSDK
    ${PROJECT_NAME}
    )

target_include_directories(dynamixel_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${DYNAMIXEL_SDK_INCLUDE_DIRS}
    ${DYNAMIXEL_WORKBENCH_INCLUDE_DIRS}
    )

target_link_libraries(dynamixel_controller
    ${Eigen3_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${DYNAMIXEL_SDK_LIBRARIES}
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
