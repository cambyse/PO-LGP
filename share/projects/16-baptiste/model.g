Include = '../../data/baxter_model/baxter.ors'
Delete shape visual

Include = '../../data/reba-human/human.ors'

### objs

body target { X=<T t(.5 .5 1.3)> }
shape target(target) { type=1 size=[0 0 0 .05] color=[0 1 0] }

Edit /human/base { X=<T t(1.5 0. 1.1) d(-180 0 0 1)> }

body tableC{ type=9, X=<T t(.7 0 .8)>, size=[1. .8 .04 .02], color=[.3 .3 .5] fixed, contact }
body tableL{ type=9, X=<T t(.2 .7 .8)>, size=[2. .6 .04 .02], color=[.3 .5 .3] fixed, contact }
body tableR{ type=9, X=<T t(.2 -.7 .8)>, size=[2. .6 .04 .02], color=[.3 .5 .3] fixed, contact }

shape humanL(/human/left_wrist){ rel=<T t(0 -.05 0)> type=5 color=[1 0 0] size=[.1 0 0 0] }
shape humanR(/human/right_wrist){ rel=<T t(0 -.05 0)> type=5 color=[1 0 0] size=[.1 0 0 0] }
#shape humanR (handR){ type=5 size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }
#shape humanL (handL){ type=5 size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }

shape baxterR (right_wrist){ rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> type=5 size=[.1 0 0 0] color=[1 1 0] }
shape baxterL (left_wrist) { rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> type=5 size=[.1 0 0 0] color=[1 1 0] }


Include = '../../data/toolbox/toolbox.ors'

joint (tableC /toolbox/handle) { from=<T t(0 0 .04) t(0 0 .12)> to=<T > type=10 }
joint (tableC /toolbox/side_front) { from=<T t(0 0 .04) t(0 0 .12)> to=<T d(-90 0 0 1) t(0 -.212 0)> type=10 }
joint (tableC /toolbox/side_back) { from=<T t(0 0 .04) t(0 0 .12)> to=<T d(90 0 0 1) t(0. -.212 0)> type=10 }
joint (tableC /toolbox/side_left) { from=<T t(0 0 .04) t(0 0 .12)> to=<T t(.0 -.147 .0)> type=10 }
joint (tableC /toolbox/side_right) { from=<T t(0 0 .04) t(0 0 .12)> to=<T d(180 0 0 1) t(.0 -.147 0)> type=10 }
joint (tableC /toolbox/floor_left) { from=<T t(0 0 .04)> to=<T t(.0 .069 .004) d(90 1 0 0)> type=10 }
joint (tableC /toolbox/floor_right) { from=<T t(0 0 .04)> to=<T d(180 0 0 1) t(.0 .069 .004) d(90 1 0 0)> type=10 }


Include = '../../data/screwdriver/screwdriver.ors'
joint (tableR screwdriver) { from=<T t(0 0 .06)> to=<T t(-.5 .0 .0) > type=10 }
#joint (tableR screwdriver) { from=<T t(0 0 .06)> to=<T t(.5 .0 .0) > type=10 }
shape screwdriverHandle(screwdriver) { type=5 rel=<T d(90 0 0 1)> size=[.15 0 0 0] color=[1 1 0] }

body screwbox { type=9, size=[.05 .1 .04 .02], color=[.8 .3 .3] fixed, contact }
joint (tableL screwbox) { from=<T t(.8 0 .08)> type=10 }
