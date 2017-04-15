#Include = '../../data/baxter_model/baxter.ors'
#Delete shape visual

#Include = '../../data/reba-human/human.ors'
Include = 'data/man_model.ors'

### objs

#middle
body container_1 { X=<T t(.0 -1.0 1.23)> contact }
shape container_1_front(container_1)  { type=9 rel=<T t(0  0.15 0.2)> size=[0.3 0.01 0.4 0.01]   color=[1 0 0] contact }
shape container_1_left(container_1)   { type=9 rel=<T t(.15  0.0  0.2)> size=[0.01 0.3 0.4 0.01] color=[1 0 0] contact }
shape container_1_right(container_1)  { type=9 rel=<T t(-.15 0.0  0.2)> size=[0.01 0.3 0.4 0.01] color=[1 0 0] contact }
shape container_1_bottom(container_1) { type=9 rel=<T t(0  0.0 0)> size=[0.3 0.3 0.01 0.01]     color=[1 0 0] contact }

body target_1 { X=<T t(.0 -1.0 1.22)> }
shape target_1(target_1) { type=1 rel=<T t(0 0.0 0.05)> size=[0 0 0 .05] color=[0 1 0] }

#right
body container_2 { X=<T t(-.4 -0.6 1.23)> }
shape container_2_front(container_2)  { type=9 rel=<T t(0  0.15 0.15)> size=[0.3 0.01 0.3 0.01]   color=[1 0 0] }
shape container_2_left(container_2)   { type=9 rel=<T t(.15  0.0  0.15)> size=[0.01 0.3 0.3 0.01] color=[1 0 0] }
shape container_2_right(container_2)  { type=9 rel=<T t(-.15 0.0  0.15)> size=[0.01 0.3 0.3 0.01] color=[1 0 0] }
shape container_2_bottom(container_2) { type=9 rel=<T t(0  0.0 0)> size=[0.3 0.3 0.01 0.01]     color=[1 0 0] }

#Edit /human/base { X=<T t(1.5 0. 1.1) d(-180 0 0 1)> }

#body tableC{ type=9, X=<T t(.7 0 .8)>, size=[1. .8 .04 .02], color=[.3 .3 .5] fixed, contact }
#body tableL{ type=9, X=<T t(.2 .7 .8)>, size=[2. .6 .04 .02], color=[.3 .5 .3] fixed, contact }
body table{ type=9, X=<T t(0 -.7 1.2)>, size=[2. .7 .04 .02], color=[.3 .5 .3] fixed, contact }

## GRASP references

#body humanGraspRefR { type=5 size=[.1 0 0 0] color=[1 0 0] }
#body humanGraspRefL { type=5 size=[.1 0 0 0] color=[1 0 0] }
#joint humanGraspJointR(handR humanGraspRefR){ A=<T t(0 0 -.1)> type=11 ctrl_H=1e-4 }
#joint humanGraspJointL(handL humanGraspRefL){ A=<T t(0 0 -.1)> type=11 ctrl_H=1e-4 }
shape humanR (handR){ type=5 size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }
shape humanL (handL){ type=5 size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }

## Joints
joint (table container_1) { from=<T t(0 -0.3 0.02)> type=10 } #from=<T t(0 0 0) t(0 0 0)> to=<T >
#joint (table container_1) { from=<T t(0 -0.2 0.02) t(0 0 0)> to=<T > type=10 } #from=<T t(0 0 0) t(0 0 0)> to=<T >
#joint (target_1 occluding_object_1) { from=<T t(0 0 .04) t(0 0 .12)> to=<T > type=10 }
