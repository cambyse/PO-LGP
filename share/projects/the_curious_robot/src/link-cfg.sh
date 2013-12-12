#!/bin/sh

# A simple script to set enable different robot models in The Curious Robot.
# You can switch between a "flying ball robot" and the "PR2".

# GUARD: abot if usage is incorrect
if [ "$#" -ne 1 ] || ( ! [ "$1" = "ball" ] && ! [ "$1" = "pr2" ] ) ; then
  echo "Usage: $0 (ball|pr2)"
  exit
fi

DEST=~/.ros/MT.cfg
# rm old link of DEST if it exists
if [ -L $DEST ] ; then
  echo "rm link ~/.ros/MT.cfg"
  rm $DEST
else
  echo "negative" $DEST
fi

echo "Linking..."
ln -s $(pwd)/MT-$1.cfg ~/.ros/MT.cfg
echo "DONE"
