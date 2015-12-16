cmake_minimum_required(VERSION 2.8.3)
project(vicon_bridge)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBT_USE_DOUBLE_PRECISION -Wall")
# Unused warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter")
# Additional warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body -Wlogical-op")
#Suppress deprecated strings and unused result warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings -Wno-unused-result")

#set(CMAKE_PREFIX_PATH "/usr/lib/)
#set(CMAKE_PREFIX_PATH /usr/lib/x86_64-linux-gnu)
#set(CMAKE_PREFIX_PATH /usr/local/lib)

OPTION(ENABLE_CXX11 "Enable C++11 support" ON)

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()


find_package(catkin REQUIRED COMPONENTS
    message_generation
    dynamic_reconfigure
    geometry_msgs
    roscpp
    tf
    diagnostic_updater
)

find_package(Boost REQUIRED COMPONENTS system thread)

# Generate messages and services
add_message_files(FILES
    Marker.msg
    Markers.msg
    TfDistortInfo.msg
)

add_service_files(FILES
    viconCalibrateSegment.srv
    viconGrabPose.srv
)

generate_messages(DEPENDENCIES geometry_msgs)

# Generate dynamic parameters
generate_dynamic_reconfigure_options(
  cfg/tf_distort.cfg
)

catkin_package(CATKIN_DEPENDS 
    dynamic_reconfigure
    geometry_msgs
    message_runtime 
    roscpp 
)

# check for platform and use the right Vicon SDK
if(CMAKE_SIZEOF_VOID_P MATCHES "8")
    set(VICON_SDK             vicon_sdk/Vicon_SDK_1.3_Linux/64-bit/)
    set(VICON_SDK_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/${VICON_SDK})
    set(VICON_SDK_LINK_DIR    ${PROJECT_SOURCE_DIR}/${VICON_SDK})
elseif(CMAKE_SIZEOF_VOID_P MATCHES "4")
    set(VICON_SDK             vicon_sdk/Vicon_SDK_1.3_Linux/32-bit/)
    set(VICON_SDK_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/${VICON_SDK})
    set(VICON_SDK_LINK_DIR    ${PROJECT_SOURCE_DIR}/${VICON_SDK})
else(CMAKE_SIZEOF_VOID_P MATCHES "8")
    message(FATAL_ERROR "unable to detect platform")
endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
find_library(VICON_SDK_LIBRARY ViconDataStreamSDK_CPP PATHS ${VICON_SDK_LINK_DIR})

include_directories(src ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${VICON_SDK_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR})

add_library(msvc_bridge src/msvc_bridge.cpp)

add_executable(vicon_bridge src/vicon_bridge.cpp)
target_link_libraries(vicon_bridge
    msvc_bridge
    ${VICON_SDK_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(vicon_bridge ${PROJECT_NAME}_gencpp)

add_executable(calibrate src/calibrate_segment.cpp)
target_link_libraries(calibrate
   ${catkin_LIBRARIES}
)
add_dependencies(calibrate ${PROJECT_NAME}_gencpp)

add_executable(tf_distort src/tf_distort.cpp)
target_link_libraries(tf_distort
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_dependencies(tf_distort ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

add_executable(grabpose src/grabpose_client.cpp)
target_link_libraries(grabpose
                                msvc_bridge
                                ${catkin_LIBRARIES}
                                ${VICON_SDK_LIBRARY}
                                ${Boost_LIBRARIES}
                      )

add_dependencies(grabpose ${PROJECT_NAME}_gencpp)

add_executable(testclient src/ViconDataStreamSDK_CPPTest.cpp)
target_link_libraries(testclient 
    ${VICON_SDK_LIBRARY}
    ${Boost_LIBRARIES}
)
