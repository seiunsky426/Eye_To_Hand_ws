cmake_minimum_required(VERSION 3.0.2)
project(piper_eye_to_hand_calibration)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_filters
  rospy
  sensor_msgs
  std_srvs
  tf
  tf2_ros
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs message_filters rospy sensor_msgs std_srvs tf tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/aruco_detector_node.py
  scripts/calibration_cli.py
  scripts/calibration_server_node.py
  scripts/static_tf_publisher_node.py
  scripts/import_intrinsics.py
  scripts/stereo_calibration_node.py
  scripts/compose_results.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_aruco_support.py)
  catkin_add_nosetests(test/test_calibration_math.py)
  catkin_add_nosetests(test/test_intrinsics.py)
  catkin_add_nosetests(test/test_stereo.py)
  catkin_add_nosetests(test/test_composition.py)
  catkin_add_nosetests(test/test_capture.py)
  catkin_add_nosetests(test/test_handeye.py)
endif()
