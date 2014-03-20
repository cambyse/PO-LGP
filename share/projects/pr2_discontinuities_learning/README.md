# Using discontinuities for Learning

- topic of the force/torque sensor: /ft/

## Plotting the ft topic
```
rxplot /ft/r_gripper_motor/wrench/force/x:y:z /ft/r_gripper_motor/wrench/torque/x:y:z
```

## Arm and Gripper control


## Record data
```
rosbag record /ft/r_gripper_motor/ /ft/l_gripper_motor/ /kinect_head/rgb/image_mono/compressed /l_forearm_cam/image_mono/compressed /r_forearm_cam/image_mono/compressed /tf
```

## Replay
```
rosparam set use_sim_time true
rqt
rosrun image_view image_view image:=/kinect_head/rgb/image_mono _image_transport:=compressed
```

# Record: minimal scenario

## cupboard
- lock -> unlock
- unlock -> unlock
- unlock -> lock
- lock -> lock

- close -> push
- close/locked-> pull
- close -> pull
- open -> pull
- open -> push

:vim: set ft=markdown
