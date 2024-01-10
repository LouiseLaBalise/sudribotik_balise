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

echo_and_run cd "/home/student/Desktop/Test/src/balise"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/student/Desktop/Test/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/student/Desktop/Test/install/lib/python3/dist-packages:/home/student/Desktop/Test/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/student/Desktop/Test/build" \
    "/usr/bin/python3" \
    "/home/student/Desktop/Test/src/balise/setup.py" \
     \
    build --build-base "/home/student/Desktop/Test/build/balise" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/student/Desktop/Test/install" --install-scripts="/home/student/Desktop/Test/install/bin"
