
## DATA RECORDING

Running realsense:

```
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

Rosbag recording:

```
rosbag record -o robot.bag /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /camera/color/camera_info /tf_static
```

## Offline processing

```
roslaunch gap_detector gap_localization.launch rviz:=1 rosbag:=1
```

## Online operation

```
roslaunch gap_detector gap_localization.launch rviz:=1
```


The included costmap_2d folder is from [ros_navigation](https://github.com/ros-planning/navigation/tree/noetic-devel) package. The example_params.yaml file was adjusted.