# Using discontinuities for Learning

- topic of the force/torque sensor: /ft/

## Plotting the ft topic
```
rxplot /ft/r_gripper_motor/wrench/force/x:y:z /ft/r_gripper_motor/wrench/torque/x:y:z
```

## Arm and Gripper control


## Record data
Record a ros bag with the appropriate topics. The easiest way is to use our
record.sh:
```
sh record.sh
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
