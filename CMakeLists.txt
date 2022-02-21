cmake_minimum_required(VERSION 3.7.2)
project(deepracer_track_geometry)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS rospy
)

#############
## Install ##
#############
install(DIRECTORY /opt/amazon/src/deepracer_track_geometry/deepracer_track_geometry/routes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/routes
)

## Tests

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(pytest.launch)
endif()
