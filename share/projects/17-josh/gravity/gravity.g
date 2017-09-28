body table1{ type=9, X=<T t(.5 0 .7)>, size=[1. 1. .04 .02], color=[.3 .3 .3] fixed, contact }

body world{ type=ST_marker size=[.2 .2 .2 .2]}

### ball

body block{ type=ST_ssBox, X=<T t(.1 0 .8)>, size=[.2 .2 .3 .02], color=[.9 .3 .3] fixed, contact }
joint (world block){ type=JT_free A=<T t(-.1 0 1.5)> }


