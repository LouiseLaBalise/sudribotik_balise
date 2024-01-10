execute_process(COMMAND "/home/student/Desktop/Test/build/balise/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/student/Desktop/Test/build/balise/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
