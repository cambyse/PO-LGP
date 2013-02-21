#!/bin/bash

# This script is used by jenkins and automates the execution
# of the usual "tests" to make sure that the MLR project is 
# functional, i.e. that the project can be checked out from
# sratch and can be build without any manual effort.

# build everythin under share
cd share
rm -r lib/*
make cleanAll
make 2> ../gcc_warnings.log

# test the basic building blocks
cd test/array
make
./x.exe
