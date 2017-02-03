#Include = '../../data/baxter_model/baxter.ors'
#Delete shape visual

#Include = '../../data/reba-human/human.ors'
Include = 'data/man_model.ors'

### objs

body target { X=<T t(1.0 0.0 1.9)> }
#body target { X=<T t(0.0 -0.4 1.7)> }
shape target(target) { type=1 size=[0 0 0 .05] color=[0 1 0] }

body occluding_object { X=<T t(.0 -0.6 1.9)> }
shape occluding_object(occluding_object) { type=9 size=[0.4 0.01 0.4 0.02] color=[1 0 0] }

body target2 { X=<T t(.0 -1.0 1.9)> }
shape target2(target2) { type=1 size=[0 0 0 .05] color=[1 0 0] }

#Edit /human/base { X=<T t(1.5 0. 1.1) d(-180 0 0 1)> }

#body tableC{ type=9, X=<T t(.7 0 .8)>, size=[1. .8 .04 .02], color=[.3 .3 .5] fixed, contact }
#body tableL{ type=9, X=<T t(.2 .7 .8)>, size=[2. .6 .04 .02], color=[.3 .5 .3] fixed, contact }
body tableR{ type=9, X=<T t(.2 -.7 .8)>, size=[2. .6 .04 .02], color=[.3 .5 .3] fixed, contact }

#shape humanL(/human/left_wrist){ rel=<T t(0 -.05 0)> type=5 color=[1 0 0] size=[.1 0 0 0] }
#shape humanR(/human/right_wrist){ rel=<T t(0 -.05 0)> type=5 color=[1 0 0] size=[.1 0 0 0] }
shape humanR (handR){ type=5 size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }
shape humanL (handL){ type=5 size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }

