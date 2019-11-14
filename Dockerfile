
# image for configuring, building, and testing my Udacity projects

FROM ubuntu:18.04 AS udacity-dev

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        sudo \
        pkg-config \
        git \
        build-essential \
        ssh \
        rsync \
        wget \
        ca-certificates \
        libopencv-dev \
        libfltk1.3-dev \
        fluid \
        libgl1-mesa-dev \
        doxygen \
        vim \
        less \
        gettext-base \
        netbase \
        iputils-ping \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# install a newer version of cmake
RUN mkdir /cmake \
    && cd /cmake \
    && wget https://github.com/Kitware/CMake/releases/download/v3.15.5/cmake-3.15.5-Linux-x86_64.sh \
    && chmod +x cmake-3.15.5-Linux-x86_64.sh \
    && ./cmake-3.15.5-Linux-x86_64.sh --prefix=/usr/local --skip-license 


# install a specific version of eigen
ARG eigen_version
RUN git clone --branch $eigen_version https://github.com/eigenteam/eigen-git-mirror.git /eigen \
    && mkdir /eigen/build \
    && cd /eigen/build \
    && cmake .. \
    && make install \
    && rm -rf /eigen
