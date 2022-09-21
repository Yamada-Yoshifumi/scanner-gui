FROM tiryoh/ros-desktop-vnc:noetic

RUN apt-get update && apt-get install -y git\
 && rosdep update

ADD /gui /home/ubuntu/catkin_ws/src/scanner-gui/gui
ADD /ros_srv /home/ubuntu/catkin_ws/src/scanner-gui/ros_srv
ADD /velodyne_control /home/ubuntu/catkin_ws/src/scanner-gui/velodyne_control
ADD /velodyne_description /home/ubuntu/catkin_ws/src/scanner-gui/velodyne_description
ADD /velodyne_gazebo_plugins /home/ubuntu/catkin_ws/src/scanner-gui/velodyne_gazebo_plugins
ADD /velodyne_simulator /home/ubuntu/catkin_ws/src/scanner-gui/velodyne_simulator

RUN /bin/bash -c 'sudo apt-get update\
 && sudo apt-get -y install qt5-default\
 && sudo apt-get -y install qml-module-qtquick2\
 && sudo apt-get -y install qml-module-qtquick-controls2\
 && sudo apt-get -y install qml-module-qtquick-window2\
 && sudo apt-get -y install qml-module-qtquick-localstorage\
 && sudo apt-get -y install libqt5multimedia5-plugins qml-module-qtmultimedia\
 && sudo apt-get -y install qml-module-qtquick-layouts\
 && sudo apt-get -y install qml-module-qtgraphicaleffects\
 && sudo apt-get -y install qml-module-qtmultimedia\
 && sudo apt-get -y install qml-module-qt-labs-qmlmodels'
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash\
 && cd /home/ubuntu/catkin_ws \
 && catkin_make --only-pkg-with-deps ros_srv'
RUN /bin/bash -c 'cd /home/ubuntu/catkin_ws\
 && source /opt/ros/noetic/setup.bash\
 && catkin_make --only-pkg-with-deps gui velodyne_control velodyne_description velodyne_gazebo_plugins velodyne_simulator\
 && source devel/setup.bash \
 && echo "source /home/ubuntu/catkin_ws/devel/setup.bash" >> ~/.bashrc\
 && echo "user=root" >> /root/.config'