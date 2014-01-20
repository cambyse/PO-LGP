#! /bin/bash
#
# updates the "sources to compile" include
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
    # look for C++ source
    CPP_SOURCES=$(find . -maxdepth 1 -type f -name '*.cpp' -a ! -name 'main.*.cpp' -o -name '*.h' -o -name '*.c')
    # check source dir, too
    if [ -d src ]
    then
    	CPP_SOURCES+=$(find src -maxdepth 1 -type f -name '*.cpp' -a ! -name 'main.*.cpp' -o -name '*.h' -o -name '*.c')
    fi
    if [ ! -z "$CPP_SOURCES" ]
    then
      echo "set(SOURCES ${CPP_SOURCES})" >> $tmpname
    fi
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
