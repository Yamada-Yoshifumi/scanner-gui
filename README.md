# scanner-gui

<h1>Docker</h1>

A Dockerfile is provided in the gui branch. To build the ROS Noetic version of the gui as a docker Image, do:

<code>git clone -b gui https://github.com/Yamada-Yoshifumi/scanner-gui.git</code>

or using ssh:

<code>git clone -b gui git@github.com:Yamada-Yoshifumi/scanner-gui.git</code>

In order for the Dockerfile to compile, we need to reset two environment variables:

<code>export DOCKER_BUILDKIT=0</code>

<code>export COMPOSE_DOCKER_CLI_BUILD=0</code>

Then build the Image:

<code>cd scanner-gui</code>

<code>docker build . -t \<your tag\></code>

<code>docker run -p 6080:80 \<your tag\></code>

<h1>Build</h1>

<h2>ROS Noetic</h2>

Create a catkin workspace:

<code>mkdir catkin_ws/src</code>

<code>cd catkin_ws</code>

<code>catkin_make</code>

<code>cd src</code>

Clone the code:

<code>git clone -b gui https://github.com/Yamada-Yoshifumi/scanner-gui.git</code>

or using ssh:

<code>git clone -b gui git@github.com:Yamada-Yoshifumi/scanner-gui.git</code>

Build:

<code>cd ..</code>

<code>catkin_make</code>

If Build fails due to an error from package ros_srv, try catkin_make again.

<h2>ROS2 Humble</h2>
