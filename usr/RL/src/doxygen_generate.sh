#!/bin/bash

## get list as arguments or use all subfolders and ./
if [ $# -gt 0 ]; then
    list=$1
else
    list=./*
    list="$list ."
fi

## print list and ask for OK
echo "Executing doxygen in:"
for f in $list; do
    echo "    $f"
done
read -p "OK? (Y/n)" ok

## exit if not OK or proceed...
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
