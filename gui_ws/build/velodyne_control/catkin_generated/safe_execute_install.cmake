execute_process(COMMAND "/home/ameyasu/QTProjects/scanner-gui/gui_ws/build/velodyne_control/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ameyasu/QTProjects/scanner-gui/gui_ws/build/velodyne_control/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
