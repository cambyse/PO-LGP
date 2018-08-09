#Include = 'arm7.kvg'
Include = '../../data/pr2_model/pr2_model.ors'


body table{ type=0, X=<T t(.6 0 .7)>, size=[.8 .8 .025 .02], color=[.8 .5 .3] }

# Human: large blue, small green, small red

body S1 { size=[.06 .27 .04 0] type=0 color=[0.2 0.3 0.6] coloredBox percept }
joint (table S1) { from=<T t(.15 -.3 .02)> to=<T t(0 0 .02)> type=JT_free agent=1 } #JT_transXYPhi

body S2 { size=[.06 .06 .04 0] type=0 color=[0.3 0.4 0.2] coloredBox percept }
joint (table S2) { from=<T t(.15 .3 .02)> to=<T t(0 0 .02)> type=JT_free agent=1 }

body S3 { size=[.06 .06 .04 0] type=0 color=[0.4 0.2 0.2] coloredBox percept }
joint (table S3) { from=<T t(.15 0 .02)> to=<T t(0 0 .02)> type=JT_free agent=1 }

# Robot: medium red, small blue, small yellow

body S4 { size=[.06 .18 .04 0] type=0 color=[0.4 0.2 0.2] coloredBox percept }
joint (table S4) { from=<T t(-.3 -.3 .02)> to=<T t(0 0 .02)> type=JT_free agent=1 }

body S5 { size=[.06 .06 .04 0] type=0 color=[0.2 0.3 0.6] coloredBox percept }
joint (table S5) { from=<T t(-.3 0 .02)> to=<T t(0 0 .02)> type=JT_free agent=1 }

body S6 { size=[.06 .06 .04 0] type=0 color=[0.9 0.9 0.6] coloredBox percept }
joint (table S6) { from=<T t(-.3 .3 .02)> to=<T t(0 0 .02)> type=JT_free agent=1 }

# Targets

shape target1(table) { type=ST_marker rel=<T t(0. .12 0.) d(0 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }
shape target2(table) { type=ST_marker rel=<T t(0. .04 0.) d(0 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }
shape target3(table) { type=ST_marker rel=<T t(0. -.04 0.) d(0 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }
shape target4(table) { type=ST_marker rel=<T t(0. -.12 0.) d(0 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }

shape targetfront(table) { type=ST_marker rel=<T t(-.3 0. 0.) d(0 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }
shape targetfrontleft(table) { type=ST_marker rel=<T t(-.3 .2 0.) d(0 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }
shape targetfrontright(table) { type=ST_marker rel=<T t(-.3 .2 0.) d(0 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }
