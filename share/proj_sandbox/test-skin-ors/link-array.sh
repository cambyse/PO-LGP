#!/bin/sh

[ -z $xstart ] && xstart=.0090
[ -z $xoffset ] && xoffset=-.0036
[ -z $ystart ] && ystart=.0676
[ -z $yoffset ] && yoffset=-.0036

calc() { 
  bc<<.
  scale=5; 
  $@
.
}

echo "["
for i in `seq 0 13`; # y loop
do
	y=$( calc "$ystart + $yoffset * $i" )

	for j in `seq 0 5`; # x loop
	do
		x=$( calc "$xstart + $xoffset * $j")

		echo "<d(-90 1 0 0) d(180 0 0 1)  t($x $y -.015) >" 

	done # x

done # y
echo "]"
