#!/bin/sh

for number1 in 1 2 3 4 5 6 7 8 9 10 
do
for number2 in 1 2 3 4
do
echo -n "$number1 "
echo -n "$number2 \n"
~/TUB/mlr/share/robot/10-nik-graspdata/test.sh
done
done


###validate.txt should exist