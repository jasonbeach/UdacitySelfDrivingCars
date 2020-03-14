#! /bin/bash
export DEBIAN_FRONTEND="noninteractive"
apt update && apt install -y --no-install-recommends tzdata

ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime 
#    apt-get install -y tzdata && \

dpkg-reconfigure --frontend noninteractive tzdata