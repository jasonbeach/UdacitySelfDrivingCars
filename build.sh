#!/bin/bash

image_name=udacity_opencv_cpp

eigen_version=3.3.5
echo "build script for Jason's project 1"
echo "Eigen Version for image: ${eigen_version}"
echo "docker image name: ${image_name}"

main()
{
  check_image
  if [[ $build_image == true ]]; then
    echo "building docker image"
    build_docker_image
  fi

  build_project
}

check_image()
{
  build_image=false
  if [[ "$(docker images -q $image_name 2> /dev/null)" == "" ]]; then
    build_image=true
  else
    read -p "docker image $image_name found, do you want to rebuild it [n]? " input
    if [[ ${input^^} == "Y" ]] || [[ ${input^^} == "YES" ]]; then
      build_image=true
    fi
  fi
}

build_docker_image()
{
  echo building $image_name image
  docker build -t $image_name \
    --build-arg eigen_version=$eigen_version \
    -f Dockerfile .
}

build_project()
{
  echo "running docker container to build project"
  docker run --device=/dev/video0:/dev/video0 -v /tmp/.X11-unix:/tmp/.X11-unix --user 1000:1000 -v $(pwd):/devel -e DISPLAY=$DISPLAY -p 5000:5000 -p 8888:8888 -it udacity_opencv_cpp /devel/build.sh -r
}

do_build()
{
  mkdir -p /devel/build
  pushd /devel/build > /dev/null
    echo "running cmake..."
    cmake -DCMAKE_BUILD_TYPE=Release ..
    echo "building..."
    cmake --build . --parallel
    echo "installing..."
    cmake --build . --target install
  popd >/dev/null
  echo "Done"
}

script_dir="$(dirname "$(readlink -f "$0")")"
pushd $script_dir > /dev/null
if [ "$1" = "-r" ]; then
  echo "doing build"
  do_build
else
  echo "checking docker image"
  main
fi
popd > /dev/null

