#Include = 'arm7.kvg'
Include = '../../data/pr2_model/pr2_model.ors'

#square table on wheels
body table{ type=0, X=<T t(.6 0 .7)>, size=[.8 .8 .025 .02], color=[.8 .5 .3] }
#small drawers
#body table{ type=0, X=<T t(.55 0 .54)>, size=[.44 .59 .025 .02], color=[.8 .5 .3] }

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

shape targetbs(table) { type=ST_marker rel=<T t(0. .1 0.) d(90 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }
shape targetbl(table) { type=ST_marker rel=<T t(0. 0. .1) d(90 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }

shape targetrs(table) { type=ST_marker rel=<T t(0. -.1 0.) d(90 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }
shape target(table) { type=ST_marker rel=<T t(0. 0. 0.) d(0 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }
shape targetb(table) { type=ST_marker rel=<T t(0. .12 0.) d(0 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }
shape targety(table) { type=ST_marker rel=<T t(0. .04 0.) d(0 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }
#shape targetr(table) { type=ST_marker rel=<T t(0. -.04 0.) d(0 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }
#shape targetg(table) { type=ST_marker rel=<T t(0. -.12 0.) d(0 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }
shape targetg(table) { type=ST_marker rel=<T t(0. -.04 0.) d(0 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }
shape targetr(table) { type=ST_marker rel=<T t(0. -.12 0.) d(0 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }

shape targetg1(table) { type=ST_marker rel=<T t(0. 0.10 0.04) d(0 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }
shape targety1(table) { type=ST_marker rel=<T t(0. -0.10 0.04) d(0 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }
