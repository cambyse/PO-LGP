Include='../../../share/data/drake_kuka/setup.g'

body obj1{ X=<T t(.5 .5 .5)> shape=ST_ssBox, size=[.05 .05 .1 .01], color=[1 1 0] }
body obj2{ X=<T t(.5 -.5 .5)> shape=ST_ssBox, size=[.05 .05 .1 .01], color=[1 1 0] }

body table1{ X=<T t(-.5 .5 .5)> shape=ST_ssBox, size=[.3 .3 .05 .01], color=[0 .5 0] }
body table2{ X=<T t(-.5 -.5 .5)> shape=ST_ssBox, size=[.3 .3 .05 .01], color=[0 .5 0] }
