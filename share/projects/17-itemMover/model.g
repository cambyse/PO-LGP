Include = '../../data/nogit.itemMover/item_mover.g'

body target (world){ type=ST_ssBox size=[.2 .2 .05 .02] color=[0 1 0] }
joint (world target){ type=JT_rigid to=<T t(.2 .2 .9)> }


