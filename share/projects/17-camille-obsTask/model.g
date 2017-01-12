#Include = '../../data/baxter_model/baxter.ors'
#Delete shape visual

#Include = '../../data/reba-human/human.ors'
Include = '../../data/man_model.ors'

### objs

body target { X=<T t(.5 .5 1.3)> }
shape target(target) { type=1 size=[0 0 0 .05] color=[0 1 0] }

#Edit /human/base { X=<T t(1.5 0. 1.1) d(-180 0 0 1)> }

#body tableC{ type=9, X=<T t(.7 0 .8)>, size=[1. .8 .04 .02], color=[.3 .3 .5] fixed, contact }
#body tableL{ type=9, X=<T t(.2 .7 .8)>, size=[2. .6 .04 .02], color=[.3 .5 .3] fixed, contact }
body tableR{ type=9, X=<T t(.2 -.7 .8)>, size=[2. .6 .04 .02], color=[.3 .5 .3] fixed, contact }

#shape humanL(/human/left_wrist){ rel=<T t(0 -.05 0)> type=5 color=[1 0 0] size=[.1 0 0 0] }
#shape humanR(/human/right_wrist){ rel=<T t(0 -.05 0)> type=5 color=[1 0 0] size=[.1 0 0 0] }
shape humanR (handR){ type=5 size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }
shape humanL (handL){ type=5 size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }
