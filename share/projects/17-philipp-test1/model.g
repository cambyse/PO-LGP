#Include = 'arm7.kvg'
Include = '../../data/pr2_model/pr2_model.ors'


body table{ type=0, X=<T t(.6 0 .658)>, size=[.8 .8 .025 .02], color=[.8 .5 .3] }


body S1 { size=[.06 .06 .04 0] type=0 color=[0.4 0.2 0.2] coloredBox percept }
joint (table S1) { from=<T t(-.2 -.2 .02)> to=<T t(0 0 .02)> type=JT_free agent=1 }

body S2 { size=[.06 .06 .04 0] type=0 color=[0.2 0.3 0.6] coloredBox percept }
joint (table S2) { from=<T t(0. .2 .02)> to=<T t(0 0 .02)> type=JT_free agent=1 }



#shape objTarget7(table2) { type=ST_marker rel=<T t(-.3 .4 .02) d(90 0 0 1)> size=[.3 0 0 0] color=[1 .5 0] }

#body ball2 { X=<T t(-1 -1.2 1.5)>, size=[.1 .1 .2 .02] type=9 }
