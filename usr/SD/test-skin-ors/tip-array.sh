#!/bin/sh


[ -z $xstart ] && xstart=.0090
[ -z $xoffset ] && xoffset=-.0036
[ -z $ystart ] && ystart=-.0120
[ -z $yoffset ] && yoffset=.0036
[ -z $zstart ] && zstart=-.0150
[ -z $zoffset ] && zoffset=.0000
[ -z $astart ] && astart=0
[ -z $aoffset ] && aoffset=0

calc() { 
  bc<<.
  scale=5; 
  $@
.
}


a=0
echo "["
for i in `seq 0 12`; # y loop
do

  [ $i -eq 5 ] && yoffset=$( calc "$yoffset-.00002" )
  [ $i -gt 5 ] && yoffset=$( calc "$yoffset-.00001" )

  [ $i -eq 5 ] && zoffset=$( calc "$zoffset+.0002" )
  [ $i -gt 5 ] && zoffset=$( calc "$zoffset+.00015" )

  [ $i -eq 5 ] && a=8
  [ $i -gt 5 ] && a=$(calc $a+4)
  [ $i -gt 8 ] && a=$(calc "$a-($i-8)")

  y=$(calc "$ystart + $yoffset * $i")
  z=$(calc "$zstart + $zoffset * ($i-5)")

  for j in `seq 0 5`; # x loop
  do

    x=$(calc "$xstart + $xoffset * $j")

    echo "<d(-90 1 0 0) d(180 0 0 1) t($x $y $z) d($a 1 0 0)>"

  done # x

done # y
echo "]"
