# scanner-gui

<h1>Docker</h1>

A Dockerfile is provided in the gui branch. To build the ROS Noetic version of the gui as a docker Image, do:

<code>git clone -b gui https://github.com/Yamada-Yoshifumi/scanner-gui.git</code>

or using ssh:

<code>git clone -b gui git@github.com:Yamada-Yoshifumi/scanner-gui.git</code>

<code>export DOCKER_BUILDKIT=0</code>

<code>export COMPOSE_DOCKER_CLI_BUILD=0</code>

<code>cd scanner-gui</code>

<code>docker build . -t \<your tag\></code>

<code>docker run -p 6080:80 \<your tag\></code>
