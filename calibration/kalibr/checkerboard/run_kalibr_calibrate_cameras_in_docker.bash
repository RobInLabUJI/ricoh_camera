#!/bin/bash

mkdir -p $1

docker run --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v `pwd`:/data:rw ros-kalibr \
  bash -c "cd /data && \
           kalibr_calibrate_cameras --target checker_8x5.yaml --bag $3_camera.bag \
             --models $1 --topics /$2/image_$3_raw --show-extraction && \
           chown $UID.${GROUPS[0]} camchain-*.yaml results-*.txt report-*.pdf"

mv camchain-*.yaml results-*.txt report-*.pdf $1

