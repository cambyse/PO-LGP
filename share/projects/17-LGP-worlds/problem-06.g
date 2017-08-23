Include = '../../data/baxter_model/baxter.ors'
Include = 'problem-shared.g'

body world { type=ST_marker size=[.2 .2 .2 .2] }

### tools

#body stick1 {}
#joint (table1 stick1) { from=<T t(0 0 .02)> to=<T t(-.2 .7 .02)> type=JT_rigid}
#shape (stick1) { type=9 size=[.8 .025 .04 .01] color=[.6 .3 0] contact }

### hook

body nostick  {}
joint (table1 nostick) { from=<T t(0 0 .02)> to=<T t(-.2 -.7 .02)> type=JT_rigid}
shape stick(nostick) { type=9 size=[.8 .025 .04 .01] color=[.6 .3 0] contact }
shape stickTip (nostick) { rel=<T t(.4 .1 0) d(90 0 0 1)> type=9 size=[.2 .026 .04 0.01] color=[.6 .3 0] }

### box

body box { type=9 size=[.3 .3 .4 .01] color=[.6 .3 0] contact }
joint (table1 box) { from=<T t(0 0 .02)> to=<T t(.3 0 .2)> type=JT_rigid}

### ball

body redBall { size=[.06 .06 .06 .02] color=[1 0 0] type=ST_ssBox contact }
joint (box redBall) { from=<T t(0 0 .2)> to=<T t(0 0 .03)> type=JT_rigid }

