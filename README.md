# scanner-gui

This is the ROS2 Humble version of the LiDAR scanner user interface.

<h1>Build</h1>

Create a colcon workspace:
```
mkdir -p ~/workspace/src     # Make a workspace directory with a src subdirectory
cd ~/workspace               # Change directory to the workspace root

colcon list                     # List all packages in the workspace
colcon graph                    # List all packages in the workspace in topological order
                                  # and visualize their dependencies
colcon build
```
Clone the code:

<code>cd src</code>

<code>git clone -b ros-humble https://github.com/Yamada-Yoshifumi/scanner-gui.git</code>

or using ssh:

<code>git clone -b ros-humble git@github.com:Yamada-Yoshifumi/scanner-gui.git</code>

Build:

<code>cd ..</code>

<code>colcon build --select-packages ros_srv</code>

<code>colcon build --symlink-install</code>

<h2>Dependencies</h2>

You may or may not have the required dependencies installed, depending on your Qt5 environment setup and your ROS setup. If build fails, try install any of the following manually. If you do not have the full-desktop version of ROS Humble installed, you may want to have a look at the compiler logs and install the missing packages.

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
```

<h1>Use</h1>

<code>source install/setup.bash</code>

<code>ros2 run rviz_embed_test rviz_embed_test</code>


The ROS2 Humble version has basically the same features as the ROS Noetic version. However, many of its features have not been properly tested.

The Velodyne simulation packages under velodyne folder are not in working state. They were built for ROS2 Foxy, not Humble.
