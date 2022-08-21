## BEGIN_TUTORIAL
## This CMakeLists.txt file for rviz_plugin_tutorials builds both the
## TeleopPanel tutorial and the ImuDisplay tutorial.
##
## First start with some standard catkin stuff.
cmake_minimum_required(VERSION 3.0.2)
project(librviz_tutorial)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
list(APPEND CMAKE_PREFIX_PATH "~/Qt/5.15.2/gcc_64")
find_package(catkin REQUIRED COMPONENTS rviz roscpp)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})



  find_package(Qt5  REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/myviz.cpp
  src/main.cpp
)

## Add the "myviz" executable and specify the list of source files we
## collected above in ``${SRC_FILES}``.
add_executable(myviz ${SRC_FILES})

## Link the myviz executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, or by the
## ``set(QT_LIBRARIES Qt5::Widgets)``, and with whatever libraries
## catkin has included.
target_link_libraries(myviz ${QT_LIBRARIES} ${catkin_LIBRARIES})
## END_TUTORIAL

## Install
install(TARGETS myviz DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
