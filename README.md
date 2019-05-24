# ricoh_camera

ROS package for the [Ricoh Theta S omnidirectional camera](https://theta360.com/en/about/theta/s.html).

Tested with [ROS Kinetic](http://wiki.ros.org/kinetic).

The [user guide](https://support.theta360.com/en/manual/s/index.html) provides more detailed explanations 
on how to use this camera.

## Calibration

See the [wiki](https://github.com/RobInLabUJI/ricoh_camera/wiki/Calibration).

## Testing

Connect the camera to the computer in [Live Streaming](https://support.theta360.com/en/manual/s/content/streaming/streaming_01.html) mode and run:
```
roslaunch ricoh_camera capture.launch
```

A window will display the images of both fish-eye cameras (front and back).

Default configuration:
* Input device: `/dev/video0`
* Resolution: 1280 x 720
* Frequency: 30 Hz

You can also [test the package with Docker](https://github.com/RobInLabUJI/ricoh_camera/wiki/Use-with-Docker).

Notes:
* With USB connection the frequency is limited to 15 Hz
* With HDMI to USB3 connection the resolution can be set to 1920 x 1080 at 30 Hz

### Visualization with rviz

This package can be used for visualization on a skysphere with rviz: [https://github.com/RobInLabUJI/rviz_textured_sphere](https://github.com/RobInLabUJI/rviz_textured_sphere)

Clone and build the package in your workspace:
```
cd <YOUR_WORKSPACE>/src
git clone https://github.com/RobInLabUJI/rviz_textured_sphere.git
cd ..
catkin_make
```

Launch the camera streaming and visualization with:
```
roslaunch ricoh_camera capture.launch
roslaunch rviz_textured_sphere demo.launch
```

Note: the images are projected without any calibration.

## Equirectangular projection
Test with live images:
```
roslaunch ricoh_camera test_eqrect_python_live.launch
```

Test with a bag file (set the path in the launch file):
```
roslaunch ricoh_camera test_eqrect_python_bag.launch
```

Calibration files in:
* `config/front_camera_calib.yaml`
* `config/back_camera_calib.yaml`

### Example notebook
Requisites:
* [`jupyter notebook`](https://jupyter.org), install with:
```
python -m pip install --upgrade pip
python -m pip install jupyter
```

* `matplotlib>2.0.0`, install with:
```
python -m pip install --upgrade --user matplotlib
```

Run the notebook with:
```
jupyter notebook `rospack find ricoh_camera`/notebooks/Equirectangular_Projection.ipynb
```
