# ricoh_camera

ROS package for the [Ricoh Theta S omnidirectional camera](https://theta360.com/en/about/theta/s.html).

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

Notes:
* With USB connection the frequency is limited to 15 Hz
* With HDMI to USB3 connection the resolution can be set to 1920 x 1080 at 30 Hz

## Equirectangular projection
```
roslaunch ricoh_camera test_eqrect_python_live.launch
```
```
roslaunch ricoh_camera test_eqrect_python_bag.launch
```
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
