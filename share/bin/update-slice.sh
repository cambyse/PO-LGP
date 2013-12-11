#! /bin/bash
#
# updates the "directories to include" file
#

incname=.slicedef.cmake
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
    # look for subdirs with CMakeLists in them
    subfiles=$(ls */CMakeLists.txt 2>/dev/null)   
    if [ ! -z "$subfiles" ]
    then
      dirs=$(for file in $subfiles; do dirname $file; done)
      echo -e "set(CURRENT_SLICE \n ${dirs}\n )" >> $tmpname
    else
      echo "unset(CURRENT_SLICE)" >> $tmpname
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
