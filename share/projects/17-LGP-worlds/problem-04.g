Include = '../../data/baxter_model/baxter.ors'
Include = 'problem-shared.g'

### ball

body redBall { size=[.06 .06 .06 .02] color=[1 0 0] type=ST_ssBox contact }
joint (table1 redBall) { from=<T t(0 0 .02)> to=<T t(-.1 0 .03)> type=JT_rigid }

### block

body block {}
joint (table1 block) { from=<T t(0 0 .02)> to=<T t(-.3 0 .25)> type=JT_rigid}
shape (block) { type=9 size=[.05 .5 .5 .01] color=[.6 .3 0] contact }
shape (block) { type=9 rel=<T t(0 0 -.225)> size=[.2 .5 .05 .01] color=[.6 .3 0] contact }
