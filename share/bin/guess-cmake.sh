#! /bin/bash
#
# guesses (internal) dependencies to create an appropriate CMakeLists.txt
#
# Synopsis: guess-cmake.sh [-f] dir1 [dir2 [dir3 [..]]]
# 

# see if we should force creation
force=0
if [ "$1" = '-f' ]; then force=1; shift; fi

NAME=CMakeLists.txt

# create the list of known dependencies by considering what is in "src"
BASEDIR=$(realpath -s $(dirname $0)/../src)
DEPS=$(for dir in $(find $BASEDIR -maxdepth 1 -type d|grep -v -E '(Hardware|extern)'); do basename $dir; done)
DEPS+=$(for dir in $(find $BASEDIR/Hardware -maxdepth 1 -type d); do echo Hardware_$(basename $dir); done)
DEPS+=$(for dir in $(find $BASEDIR/extern -maxdepth 1 -type d); do echo extern_$(basename $dir); done)

# walk over directories given on command-line
for dir in $*
do
  if [ ! -d "$dir" ]; then echo "Directory $dir not found, skipping"; continue; fi

  target="$dir/$NAME"
  # check for existing file
  if [ -f "$target" ]
  then
    echo -n "Found existing $NAME in $dir, "
    if [ $force = 1 ]
    then
      echo "but was forced, moving away"
      cp --backup=t -f "$target" "${target}.bak"
    else
      echo "not forced, skipping $dir"
      continue
    fi
  fi

  update_cmd=$(realpath $(dirname $0))/update-includes.sh
  ## Analyze target directory
  pushd $dir >/dev/null
    # create sources list
    echo "include(.sources.cmake)" > $NAME
    $update_cmd
    rm -f $NAME

    if [ ! -f ".sources.cmake" ];
    then
      echo "No sources found in $dir, skipped"
      popd
      continue
    fi

    # always add Core, then check includes for more
    LINK_LIBRARIES=$(echo -n "Core "; for include in $(grep -E '^.*#include' $(cat .sources.cmake|\
      sed 's/set(SOURCES//'|sed 's/)//') | \
       cut -d \< -f 2|cut -d \> -f 1)
    do
      # translate / to _ to match the library name format used in deps
      libref=$(dirname $include | sed 's/\//_/')
      if [ "$libref" = "." ]; then continue; fi
      # if we got a lib like that in src, add it to deps
      if echo $DEPS | grep -q "$libref"; then
        echo -n "$libref "
      fi
    done | sort -u) # weed out duplicates

  ##
  popd >/dev/null


  # write resulting CMakeLists.txt
  cat <<EOF > "$target"
get_filename_component(NAME \${CMAKE_CURRENT_SOURCE_DIR} NAME)
include(.sources.cmake)
add_executable(\${NAME} \${SOURCES})
target_link_libraries(\${NAME} $LINK_LIBRARIES)
add_test(\${NAME}_test \${NAME} --gtest_output=xml:\${NAME}-unit.xml)
EOF

done
