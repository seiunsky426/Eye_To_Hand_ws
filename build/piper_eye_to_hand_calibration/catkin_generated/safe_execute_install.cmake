execute_process(COMMAND "/home/zhengsihze/Eye_To_Hand_ws/build/piper_eye_to_hand_calibration/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/zhengsihze/Eye_To_Hand_ws/build/piper_eye_to_hand_calibration/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
