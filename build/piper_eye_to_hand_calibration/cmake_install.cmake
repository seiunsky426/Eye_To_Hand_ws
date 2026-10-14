# Install script for directory: /home/zhengsihze/Eye_To_Hand_ws/src/piper_eye_to_hand_calibration

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zhengsihze/Eye_To_Hand_ws/install")
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
  include("/home/zhengsihze/Eye_To_Hand_ws/build/piper_eye_to_hand_calibration/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zhengsihze/Eye_To_Hand_ws/build/piper_eye_to_hand_calibration/catkin_generated/installspace/piper_eye_to_hand_calibration.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/piper_eye_to_hand_calibration/cmake" TYPE FILE FILES
    "/home/zhengsihze/Eye_To_Hand_ws/build/piper_eye_to_hand_calibration/catkin_generated/installspace/piper_eye_to_hand_calibrationConfig.cmake"
    "/home/zhengsihze/Eye_To_Hand_ws/build/piper_eye_to_hand_calibration/catkin_generated/installspace/piper_eye_to_hand_calibrationConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/piper_eye_to_hand_calibration" TYPE FILE FILES "/home/zhengsihze/Eye_To_Hand_ws/src/piper_eye_to_hand_calibration/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/piper_eye_to_hand_calibration" TYPE PROGRAM FILES "/home/zhengsihze/Eye_To_Hand_ws/build/piper_eye_to_hand_calibration/catkin_generated/installspace/aruco_detector_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/piper_eye_to_hand_calibration" TYPE PROGRAM FILES "/home/zhengsihze/Eye_To_Hand_ws/build/piper_eye_to_hand_calibration/catkin_generated/installspace/calibration_cli.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/piper_eye_to_hand_calibration" TYPE PROGRAM FILES "/home/zhengsihze/Eye_To_Hand_ws/build/piper_eye_to_hand_calibration/catkin_generated/installspace/calibration_server_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/piper_eye_to_hand_calibration" TYPE PROGRAM FILES "/home/zhengsihze/Eye_To_Hand_ws/build/piper_eye_to_hand_calibration/catkin_generated/installspace/static_tf_publisher_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/piper_eye_to_hand_calibration" TYPE PROGRAM FILES "/home/zhengsihze/Eye_To_Hand_ws/build/piper_eye_to_hand_calibration/catkin_generated/installspace/import_intrinsics.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/piper_eye_to_hand_calibration" TYPE PROGRAM FILES "/home/zhengsihze/Eye_To_Hand_ws/build/piper_eye_to_hand_calibration/catkin_generated/installspace/stereo_calibration_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/piper_eye_to_hand_calibration" TYPE PROGRAM FILES "/home/zhengsihze/Eye_To_Hand_ws/build/piper_eye_to_hand_calibration/catkin_generated/installspace/compose_results.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/piper_eye_to_hand_calibration" TYPE DIRECTORY FILES
    "/home/zhengsihze/Eye_To_Hand_ws/src/piper_eye_to_hand_calibration/config"
    "/home/zhengsihze/Eye_To_Hand_ws/src/piper_eye_to_hand_calibration/launch"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/piper_eye_to_hand_calibration" TYPE FILE FILES "/home/zhengsihze/Eye_To_Hand_ws/src/piper_eye_to_hand_calibration/README.md")
endif()

