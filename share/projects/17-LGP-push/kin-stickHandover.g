#Include = 'arm7.kvg'
#Include = '../../data/pr2_model/pr2_model.ors'
Include = '../../data/baxter_model/baxter.ors'

body table1{ type=0, X=<T t(.7 0 .7)>, size=[1. 3. .04 .0], color=[.8 .5 .3] fixed,  }

body tableL{ type=9, X=<T t(.0 .7 .8)>, size=[.6 .6 .04 .02], color=[.3 .5 .3] fixed, contact }
body tableR{ type=9, X=<T t(.0 -.7 .8)>, size=[.6 .6 .04 .02], color=[.3 .5 .3] fixed, contact }

body slider1a { type=ST_box size=[.2 .1 .05 0] color=[0 0 0] }
body slider1b { type=ST_box size=[.2 .1 .05 0] color=[1 0 0] }
joint (slider1a slider1b){ type=JT_transX }
shape slider1Eff (slider1b){ rel=<T t(.1 0 0)> type=5 size=[.1 .1 .1] color=[0 1 0] }

body obj1 { size=[.1 .1 .2 .02] color=[0 1 0] type=9 contact }
joint (table1 obj1) { from=<T t(0 0 .02)> to=<T t(.20 1.3 .1)> type=JT_rigid }

shape target(table1){ type=5 rel=<T t(0 .3 .12)> size=[.3 0 0 0] color=[1 1 0]}

shape baxterR (right_wrist){ rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> type=5 size=[.1 0 0 0] color=[1 1 0] }
shape baxterL (left_wrist) { rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> type=5 size=[.1 0 0 0] color=[1 1 0] }

### tools


body stick {}
joint (table1 stick) { from=<T t(0 0 .02)> to=<T t(0 -.7 .02) d(90 0 0 1)> type=JT_rigid}
shape stick(stick) { type=9 size=[.8 .025 .04 .01] color=[1 0 0] contact }
shape stickTip (stick) { rel=<T t(-.4 0 0) d(-90 0 0 1)> type=9 size=[.2 .026 .04 0.01] color=[1 1 0] }
