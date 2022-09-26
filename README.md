# scanner-gui

This LiDAR scanner user interface is for interfacing with a high-precision Velodyne LiDAR package with IMU and camera on board.

<h1>Docker</h1>

A Dockerfile is provided in the gui branch. To build the ROS Noetic version of the gui as a docker Image, do:

<code>git clone -b ros-noetic https://github.com/Yamada-Yoshifumi/scanner-gui.git</code>

or using ssh:

<code>git clone -b ros-noetic git@github.com:Yamada-Yoshifumi/scanner-gui.git</code>

In order for the Dockerfile to compile, we need to reset two environment variables:

<code>export DOCKER_BUILDKIT=0</code>

<code>export COMPOSE_DOCKER_CLI_BUILD=0</code>

Then build the Image:

<code>cd scanner-gui</code>

<code>docker build . -t \<your tag\></code>

<code>docker run -p 6080:80 \<your tag\></code>

<h1>Build</h1>

Create a catkin workspace:

<code>mkdir catkin_ws/src</code>

<code>cd catkin_ws</code>

<code>catkin_make</code>

<code>cd src</code>

Clone the code:

<code>git clone -b ros-noetic https://github.com/Yamada-Yoshifumi/scanner-gui.git</code>

or using ssh:

<code>git clone -b ros-noetic git@github.com:Yamada-Yoshifumi/scanner-gui.git</code>

Build:

<code>cd ..</code>

<code>catkin_make</code>

If Build fails due to an error from package ros_srv, try catkin_make again.

<h2>Dependencies</h2>

You may or may not have the required dependencies installed, depending on your Qt5 environment setup and your ROS setup. If build fails, try install any of the following manually. If you do not have the full-desktop version of ROS Noetic installed, you may want to have a look at the compiler logs and install the missing packages.

```
sudo apt-get -y install qt5-default
sudo apt-get -y install qml-module-qtquick2
sudo apt-get -y install qml-module-qtquick-controls2
sudo apt-get -y install qml-module-qtquick-window2
sudo apt-get -y install qml-module-qtquick-localstorage
sudo apt-get -y install libqt5multimedia5-plugins qml-module-qtmultimedia
sudo apt-get -y install qml-module-qtquick-layouts
sudo apt-get -y install qml-module-qtgraphicaleffects
sudo apt-get -y install qml-module-qtmultimedia
sudo apt-get -y install qml-module-qt-labs-qmlmodels
sudo apt-get -y ros-noetic-cv-bridge
```

<h1>Use</h1>

<code>roslaunch gui gui.launch</code>

The GUI provides the following interactions with the back-end packages:

- Power Button in the status panel, sends a service request to the velodyne_description package which then starts back-end ROS nodes.
- Start Recording and Start Scanning buttons which send service requests to the back-end. Start Recording is only clickable when Scanning is activated.
- Settings panel which stores all user preferences in an Sqlite3 database.

Other features that only live in the front-end:

- Touch pad which controls the view camera to the RViz render panel.
- Fullscreen button for render panel.
- Colour pattern drop-down list for PointCloud2 display.
