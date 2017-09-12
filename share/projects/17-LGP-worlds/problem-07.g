Include = '../../data/baxter_model/baxter.ors'
Include = 'problem-shared.g'

### ball

body redBall { size=[.06 .06 .06 .02] color=[1 0 0] type=ST_ssBox contact }
joint (table1 redBall) { from=<T t(0 0 .02) t(.3 -.0 .03)> type=JT_rigid }

### tools

body stick1 {}
joint (table1 stick1) { from=<T t(0 0 .02) t(-.3 -.7 .02)> type=JT_rigid}
shape stick(stick1) { type=9 size=[.8 .025 .04 .01] color=[.6 .3 0] contact }

### paper

body paper { type=9 size=[1. .2 .004 .002] color=[.6 .3 0] contact }
joint (table1 paper) { from=<T t(0 0 .02) t(-.2 .7 .0)> type=JT_rigid}

