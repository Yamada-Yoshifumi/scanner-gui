#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/ameyasu/QTprojects/scanner-gui/gui_ws/src/velodyne_control"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ameyasu/QTprojects/scanner-gui/gui_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ameyasu/QTprojects/scanner-gui/gui_ws/install/lib/python3/dist-packages:/home/ameyasu/QTprojects/scanner-gui/gui_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ameyasu/QTprojects/scanner-gui/gui_ws/build" \
    "/usr/bin/python3" \
    "/home/ameyasu/QTprojects/scanner-gui/gui_ws/src/velodyne_control/setup.py" \
     \
    build --build-base "/home/ameyasu/QTprojects/scanner-gui/gui_ws/build/velodyne_control" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ameyasu/QTprojects/scanner-gui/gui_ws/install" --install-scripts="/home/ameyasu/QTprojects/scanner-gui/gui_ws/install/bin"