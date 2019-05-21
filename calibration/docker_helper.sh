#!/bin/bash

# initialize global variables
containerName=camodocal
containerTag=1.0
GREEN='\033[1;32m'
BLUE='\e[34m'
NC='\033[0m' # no color
user=`id -u -n`
userid=`id -u`
group=`id -g -n`
groupid=`id -g`
myhostname=hostcamodocal
no_proc=`nproc`

if [ $1 = "help" ];then
	echo -e "${GREEN}>>> Possible commands:\n ${NC}"
	echo -e "${BLUE}build --- Build an image based on DockerFile in current dir\n"
	echo -e "${BLUE}run --- Create and run container from image${NC}\n"
fi

if [ "$1" = "build" ]; then
	echo -e "${GREEN}>>> Building dexrov-${space} image ...${NC}"
	docker build -t ${user}/${containerName}:${containerTag} .
fi

if [ "$1" = "run" ]; then

	echo -e "${GREEN}>>> Initializing "${containerName}" container...${NC}"
		if [ -d /sys/module/nvidia ]; then

		    NVIDIA_ARGS=""
		    for f in `ls /dev | grep nvidia`; do
		        NVIDIA_ARGS="$NVIDIA_ARGS --volume=/dev/${f}:/dev/${f}:rw"
		    done

		    NVIDIA_ARGS="$NVIDIA_ARGS --privileged"
		elif [ -d /dev/dri ]; then

		    DRI_ARGS=""
		    for f in `ls /dev/dri/*`; do
		        DRI_ARGS="$DRI_ARGS --device=$f"
		    done

		    DRI_ARGS="$DRI_ARGS --privileged"
		fi

	docker run --rm --runtime=nvidia -it \
	    $DRI_ARGS \
	    --name="${containerName}" \
	    --hostname="${myhostname}" \
	    --net=default \
	    --env="DISPLAY" \
	    --env="QT_X11_NO_MITSHM=1" \
	    --workdir="/root" \
	    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	    --volume=`pwd`/input_data:/root/input_data:rw \
	    --volume=`pwd`/output_data:/root/output_data:rw \
	    ${user}/${containerName}:${containerTag} /bin/bash -c "cd /root/camodocal/build/bin && ./$2 && chmod a+rw /root/output_data/*"
	    rc=$?; if [[ $rc != 0 && $rc != 1 ]]; then exit $rc; fi


fi
