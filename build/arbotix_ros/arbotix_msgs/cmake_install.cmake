# Install script for directory: /home/wxd/muti_uls_lidar_ws/src/arbotix_ros/arbotix_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/wxd/muti_uls_lidar_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arbotix_msgs/msg" TYPE FILE FILES
    "/home/wxd/muti_uls_lidar_ws/src/arbotix_ros/arbotix_msgs/msg/Analog.msg"
    "/home/wxd/muti_uls_lidar_ws/src/arbotix_ros/arbotix_msgs/msg/Digital.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arbotix_msgs/srv" TYPE FILE FILES
    "/home/wxd/muti_uls_lidar_ws/src/arbotix_ros/arbotix_msgs/srv/Enable.srv"
    "/home/wxd/muti_uls_lidar_ws/src/arbotix_ros/arbotix_msgs/srv/Relax.srv"
    "/home/wxd/muti_uls_lidar_ws/src/arbotix_ros/arbotix_msgs/srv/SetupChannel.srv"
    "/home/wxd/muti_uls_lidar_ws/src/arbotix_ros/arbotix_msgs/srv/SetSpeed.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arbotix_msgs/cmake" TYPE FILE FILES "/home/wxd/muti_uls_lidar_ws/build/arbotix_ros/arbotix_msgs/catkin_generated/installspace/arbotix_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/wxd/muti_uls_lidar_ws/devel/include/arbotix_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/wxd/muti_uls_lidar_ws/devel/share/roseus/ros/arbotix_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/wxd/muti_uls_lidar_ws/devel/share/common-lisp/ros/arbotix_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/wxd/muti_uls_lidar_ws/devel/share/gennodejs/ros/arbotix_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/wxd/muti_uls_lidar_ws/devel/lib/python3/dist-packages/arbotix_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/wxd/muti_uls_lidar_ws/devel/lib/python3/dist-packages/arbotix_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/wxd/muti_uls_lidar_ws/build/arbotix_ros/arbotix_msgs/catkin_generated/installspace/arbotix_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arbotix_msgs/cmake" TYPE FILE FILES "/home/wxd/muti_uls_lidar_ws/build/arbotix_ros/arbotix_msgs/catkin_generated/installspace/arbotix_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arbotix_msgs/cmake" TYPE FILE FILES
    "/home/wxd/muti_uls_lidar_ws/build/arbotix_ros/arbotix_msgs/catkin_generated/installspace/arbotix_msgsConfig.cmake"
    "/home/wxd/muti_uls_lidar_ws/build/arbotix_ros/arbotix_msgs/catkin_generated/installspace/arbotix_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arbotix_msgs" TYPE FILE FILES "/home/wxd/muti_uls_lidar_ws/src/arbotix_ros/arbotix_msgs/package.xml")
endif()

