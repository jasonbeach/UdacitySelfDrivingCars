#!/bin/bash

if [ "$1" = "-r" ]; then
  shift
  pushd /devel/install/bin > /dev/null
    ./FindLaneProject $@
  popd > /dev/null

else
  docker run --device=/dev/video0:/dev/video0 -v /tmp/.X11-unix:/tmp/.X11-unix --user 1000 -v $(pwd):/devel -e DISPLAY=$DISPLAY -p 5000:5000 -p 8888:8888 -it udacity_opencv_cpp /devel/find_lanes.sh -r $@
fi
