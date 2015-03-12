#!/bin/bash

if [ $# -gt 0 ]; then
    list=$1
else
    list=./*
fi

echo "Executing doxygen in:"
for f in $list; do
    echo "    $f"
done


read -p "OK? (Y/n)" ok
if [ "$ok" = "n" -o "$ok" = "N" ]; then
    exit
else
    for f in $list; do
	if [ -d $f ]; then
	    cd $f
	    doxygen
	    cd ..
	fi
    done
fi
