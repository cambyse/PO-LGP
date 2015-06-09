MLR & ROS & the PR2
===================

This document describes how to setup everything to make the PR2 dance.

Note: we assume you installed the MLR code into `~/git/mlr`.


On your computer and on the PR2
-------------------------------

Create symbolic links for the robot model:
```
ln -s ~/git/mlr/share/projects/pr2_gamepadControl/model.kvg ~/.ros/model.kvg
ln -s ~/git/mlr/share/data/pr2_model ~/.ros/
```

[MT: Maybe we should link ~/.ros to ~/git/mlr/rospath ????]

Put the following in your `~.bashrc`
```
export ROS_DISTRO=indigo
export ROS_VERSION=${ROS_DISTRO}
source /opt/ros/${ROS_DISTRO}/setup.sh
export ROS_PACKAGE_PATH="${HOME}/git/mlr/rospath:${ROS_PACKAGE_PATH}"
```
Or if you normally use a catkin workspace source the setup.sh from there
```
export ROS_DISTRO=indigo
export ROS_VERSION=${ROS_DISTRO}
source ${HOME}/catkin_ws/devel/setup.sh
export ROS_PACKAGE_PATH="${HOME}/git/mlr/rospath:${ROS_PACKAGE_PATH}"
```

Now compile `src/pr2/`
```
cd ~/git/mlr/share/src/pr2/marc_controller_pkg/
make -f Makefile.gof  # create the c++ header from the msg
cd ..
make -f # actually compile roscom etc
```

[MT: usually the C++ headers from msg should be checked in]

Compile `marc_pr2_controller`. (This also builds python msgs so you can use
rostopic etc.)
```
cd ~/git/mlr/rospath/marc_controller_pkg
cmake .  # you have to do this once to create a Makefile
make  # this builds the controller and the messages
```

On your computer
----------------

Tell ROS your IP and where the master is (add to your .bashrc or execute every
time):
```
export ROS_MASTER_URI=http://bigbirdc1.informatik.uni-stuttgart.de:11311
export ROS_IP=`ifconfig eth0 | grep 'inet addr:'| grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1}'`
```


On the PR2
----------
You don't need to set ROS_MASTER_URI and ROS_IP on the PR2.

Start the robot:
```
robot claim
# RUNSTOP -> press green botton
robot start
```
Also start the PR2 dashboard.
```
rosrun pr2_dashboard pr2_dashboard
```

[MT: we should replace this with using a common script, like robStart (->mlr-start-robot.sh) that also starts the ft sensor recalib]

Start our realtime controller. In `~/git/mlr/share/bin` there is
`start_rt_controller.sh`


[MT: missing: info on downloading and compiling the ft sensor git stuff]


Troubleshooting
----------------

ROS has a tool for debugging the network setup called `roswtf`. In the same
spirit there is `mlrwtf.sh` in `~/git/mlr/share/bin/roswtf.sh`. Call it to
check all relevant environment variable.

When the controller is running check if you can receive msgs
```
rostopic echo -n 1 /marc_controller/jointState
```


Known issues
----------------

- `ROS_PACKAGE_PATH` not set when calling `robot start`: controller does not
  work; controller msgs can't be found.

- can't receive controller msgs on your computer/msgs can't be found:
  msgs build?

- `make` in marc_controller_pkg does not compile the ROS package:
  `bin/createMakeFileLinks.sh` created links to the `Makefile.gof` which is
  only generating the msgs.

- `source ~/catkin_ws/devel/setup.sh` overwrites the `ROS_PACKAGE_PATH`

- `cmake .` in `marc_controller_pkg` fails: remove all old cmake artifacts.

- If `cmake .` is not successful and you are using Ubuntu 14.04 & indigo:
  The `kdl` dependency is now called `orocos-kdl` -> change `kdl` to
  `orocos-kdl` in the `manifest.xml`.


TODO / Open Points
-------------------

- We use ROS_VERSION and ROS_DISTRO. Why?
