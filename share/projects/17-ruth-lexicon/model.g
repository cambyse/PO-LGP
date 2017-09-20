#Include = 'arm7.kvg'
Include = '../../data/pr2_model/pr2_model.ors'

#single object on small rectangular table width > length
#9 objects on small rectangular table width > length
#single object on small rectangular table width < length
#9 objects on small rectangular table width < length
#single object on large square table
#9 objects on large square table

#single object on small rectangular table width > length
#9 objects on small rectangular table width > length
body table{ type=0, X=<T t(.6 0 .54)>, size=[.44 .59 .025 .02], color=[.8 .5 .3] }
#body S1 { size=[.06 .06 .04 0] type=0 color=[0.4 0.2 0.2] coloredBox percept }
#joint (table S1) { from=<T t(-.125 0 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }
#body S2 { size=[.06 .06 .04 0] type=0 color=[0.2 0.3 0.6] coloredBox percept }
#joint (table S2) { from=<T t(.125 0 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 } #JT_transXYPhi
body S3 { size=[.06 .06 .04 0] type=0 color=[0.2 0.3 0.6] coloredBox percept }
joint (table S3) { from=<T t(0 -.175 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }
#body S4 { size=[.06 .06 .04 0] type=0 color=[0.3 0.4 0.2] coloredBox percept }
#joint (table S4) { from=<T t(0 .175 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }
#body S5 { size=[.06 .06 .04 0] type=0 color=[0.8 0.8 0.2] coloredBox percept }
#joint (table S5) { from=<T t(0 0. .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }
#body S6 { size=[.06 .06 .04 0] type=0 color=[0.4 0.2 0.2] coloredBox percept }
#joint (table S6) { from=<T t(-.125 -.175 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }
#body S7 { size=[.06 .06 .04 0] type=0 color=[0.2 0.3 0.6] coloredBox percept }
#joint (table S7) { from=<T t(.125 .175 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 } #JT_transXYPhi
#body S8 { size=[.06 .06 .04 0] type=0 color=[0.2 0.3 0.6] coloredBox percept }
#joint (table S8) { from=<T t(.125 -.175 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }
#body S9 { size=[.06 .06 .04 0] type=0 color=[0.3 0.4 0.2] coloredBox percept }
#joint (table S9) { from=<T t(-.125 .175 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }


#single object on small rectangular table width < length
#9 objects on small rectangular table width < length
#body table{ type=0, X=<T t(.6 0 .535)>, size=[.59 .44 .025 .02], color=[.8 .5 .3] }
#body S1 { size=[.06 .06 .04 0] type=0 color=[0.4 0.2 0.2] coloredBox percept }
#joint (table S1) { from=<T t(-.175 0 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }
#body S2 { size=[.06 .06 .04 0] type=0 color=[0.2 0.3 0.6] coloredBox percept }
#joint (table S2) { from=<T t(.175 0 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 } #JT_transXYPhi
#body S3 { size=[.06 .06 .04 0] type=0 color=[0.2 0.3 0.6] coloredBox percept }
#joint (table S3) { from=<T t(0 -.125 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }
#body S4 { size=[.06 .06 .04 0] type=0 color=[0.3 0.4 0.2] coloredBox percept }
#joint (table S4) { from=<T t(0 .125 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }
#body S5 { size=[.06 .06 .04 0] type=0 color=[0.8 0.8 0.2] coloredBox percept }
#joint (table S5) { from=<T t(0 0. .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }
#body S6 { size=[.06 .06 .04 0] type=0 color=[0.4 0.2 0.2] coloredBox percept }
#joint (table S6) { from=<T t(-.175 -.125 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }
#body S7 { size=[.06 .06 .04 0] type=0 color=[0.2 0.3 0.6] coloredBox percept }
#joint (table S7) { from=<T t(.175 .125 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 } #JT_transXYPhi
#body S8 { size=[.06 .06 .04 0] type=0 color=[0.2 0.3 0.6] coloredBox percept }
#joint (table S8) { from=<T t(.175 -.125 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }
#body S9 { size=[.06 .06 .04 0] type=0 color=[0.3 0.4 0.2] coloredBox percept }
#joint (table S9) { from=<T t(-.175 .125 .02)> to=<T t(0 0 .02)> type=JT_transXYPhi agent=1 }

#single object on large square table
#9 objects on large square table
#body table{ type=0, X=<T t(.6 0 .70)>, size=[.8 .8 .025 .02], color=[.8 .5 .3] }
#body table{ type=0, X=<T t(.6 0 .715)>, size=[.8 .8 .025 .02], color=[.8 .5 .3] }
#body S1 { size=[.06 .06 .04 0] type=0 color=[0.4 0.2 0.2] coloredBox percept }
#joint (table S1) { from=<T t(-.25 0 .02)> to=<T t(0 0 .02)> type=JT_free agent=1 }
#body S2 { size=[.06 .06 .04 0] type=0 color=[0.2 0.3 0.6] coloredBox percept }
#joint (table S2) { from=<T t(.25 0 .02)> to=<T t(0 0 .02)> type=JT_free agent=1 } #JT_transXYPhi
#body S3 { size=[.06 .06 .04 0] type=0 color=[0.2 0.3 0.6] coloredBox percept }
#joint (table S3) { from=<T t(0 -.25 .02)> to=<T t(0 0 .02)> type=JT_free agent=1 }
#body S4 { size=[.06 .06 .04 0] type=0 color=[0.3 0.4 0.2] coloredBox percept }
#joint (table S4) { from=<T t(0 .25 .02)> to=<T t(0 0 .02)> type=JT_free agent=1 }
#body S5 { size=[.06 .06 .04 0] type=0 color=[0.8 0.8 0.2] coloredBox percept }
#joint (table S5) { from=<T t(0 0. .02)> to=<T t(0 0 .02)> type=JT_free agent=1 }
#body S6 { size=[.06 .06 .04 0] type=0 color=[0.4 0.2 0.2] coloredBox percept }
#joint (table S6) { from=<T t(-.25 -.25 .02)> to=<T t(0 0 .02)> type=JT_free agent=1 }
#body S7 { size=[.06 .06 .04 0] type=0 color=[0.2 0.3 0.6] coloredBox percept }
#joint (table S7) { from=<T t(.25 .25 .02)> to=<T t(0 0 .02)> type=JT_free agent=1 } #JT_transXYPhi
#body S8 { size=[.06 .06 .04 0] type=0 color=[0.2 0.3 0.6] coloredBox percept }
#joint (table S8) { from=<T t(.25 -.25 .02)> to=<T t(0 0 .02)> type=JT_free agent=1 }
#body S9 { size=[.06 .06 .04 0] type=0 color=[0.3 0.4 0.2] coloredBox percept }
#joint (table S9) { from=<T t(-.25 .25 .02)> to=<T t(0 0 .02)> type=JT_free agent=1 }


