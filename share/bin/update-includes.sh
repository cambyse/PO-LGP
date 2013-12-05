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
    SOURCES=$(find . -maxdepth 1 -type f -name '*.cpp' -a ! -name 'main.*.cpp' -o -name '*.h')
    echo "set(SOURCES ${SOURCES})" > $tmpname
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
