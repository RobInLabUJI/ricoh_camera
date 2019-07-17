# ricoh_camera

ROS package for the [Ricoh Theta S omnidirectional camera](https://theta360.com/en/about/theta/s.html).

Tested with [Ubuntu 16.04](http://releases.ubuntu.com/16.04/) and [ROS Kinetic](http://wiki.ros.org/kinetic).

The [user guide](https://support.theta360.com/en/manual/s/index.html) provides more detailed explanations 
on how to use this camera.

## Requisites

* Ubuntu 16.04
* ROS Kinetic
* Python Pip
```
    sudo apt install python-pip
``` 
## Installation

    python -m pip install --upgrade pip
    python -m pip install --user jupyter matplotlib==2.2.4 numpy==1.16.3 bash_kernel
    python -m bash_kernel.install
   
    cd Desktop
    mkdir -p RicohTheta_ws/src
    cd RicohTheta_ws/src/
    git clone https://github.com/RobInLabUJI/ricoh_camera.git
    cd ..
    catkin_make
   
    echo "source $HOME/Desktop/RicohTheta_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

## Notebooks with examples

    jupyter notebook ~/Desktop/RicohTheta_ws/src/ricoh_camera/notebooks/index.ipynb
   
