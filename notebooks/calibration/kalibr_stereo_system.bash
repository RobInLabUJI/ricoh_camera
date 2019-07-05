#!/bin/bash

xhost +

mkdir -p $1

docker run --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v `pwd`:/data:rw ros-kalibr \
  bash -c "cd /data && \
           kalibr_calibrate_cameras --target checker_8x5.yaml --bag $2 \
             --models $1 $1 $1 $1 --topics /bottom/back/image_raw /bottom/front/image_raw \
             /top/back/image_raw /top/front/image_raw --show-extraction && \
           chown $UID.${GROUPS[0]} camchain-*.yaml results-*.txt report-*.pdf"

mv camchain-*.yaml results-*.txt report-*.pdf $1

xhost -