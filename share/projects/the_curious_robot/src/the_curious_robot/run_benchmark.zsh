#!/bin/zsh

###############################################################################
# A really simple script to start and stop TCR (after a while).
#
# Make sure you source setup.sh before you run this!
###############################################################################

for i in {1..5}; do
  echo "######################################################################"
  echo "Run $i"
  roslaunch the_curious_robot.launch &
  # let it run for a certain abount of time
  sleep 60
  # sleep 1200
  killall roslaunch
  echo "Waiting...."
  sleep 60
done

echo "########################################################################"
echo "DONE"
echo "########################################################################"
