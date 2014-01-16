#!/bin/sh

###############################################################################
# A really simple script to start and stop TCR (after a while).
#
# Make sure you source setup.sh before you run this!
###############################################################################

for i in {0..10}; do
  roslaunch the_curious_robot.launch &
  sleep 1200
  killall roslaunch
  sleep 60
done

for i in {0..10}; do
  roslaunch the_curious_robot.launch strategy_name:=entropy_mean &
  sleep 1200
  killall roslaunch
  sleep 60
done

echo "########################################################################"
echo "DONE"
echo "########################################################################"
