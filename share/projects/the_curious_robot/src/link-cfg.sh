#!/bin/sh

if [ "$#" -ne 1 ] || ( ! [ "$1" = "ball" ] && ! [ "$1" = "pr2" ] ) ; then
  echo "Usage: $0 (ball|pr2)"
else
  ln -s MT-$1.cfg ~/.ros/MT.cfg
fi
