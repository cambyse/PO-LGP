#! /bin/bash
#
# updates the "sources to compile" include
#
# NOTE: changed to just use GLOB, works just fine
# 

incname=.sources.cmake
tmpname=${incname}.tmp

basedir=$(pwd)

# only process directories that contain CMakeLists.txt...
find . -name CMakeLists.txt | while read filename
do
  # ... with a source include
  if grep -q "include($incname)" "$filename"
  then
    dir=$(dirname "$filename")
    cd "$dir"
    # clear out tmp file
    echo -n > $tmpname
    # now just uses glob
    echo "file(GLOB CANDIDATE_SOURCES *.cpp *.c src/*.cpp)" >> $tmpname
    echo 'string(REGEX REPLACE main\\.[a-zA-Z0-9_.-]+.cpp "" SOURCES "${CANDIDATE_SOURCES}")' >> $tmpname
    if echo $dir | grep -q -E '(examples|teaching)'
    then
	echo 'set(SOURCES ${SOURCES} main.cpp)' >> $tmpname
    fi
    echo '#message(STATUS ${CMAKE_CURRENT_SOURCE_DIR} ${SOURCES})' >> $tmpname
    # only update include if content changed, to prevent unnecessary cmake calls
    if cmp $incname $tmpname >/dev/null 2>&1
    then
      rm -f $tmpname
    else
      echo "Updating source includes in $dir"
      mv -f $tmpname $incname
    fi
    cd "$basedir"
  fi
done
