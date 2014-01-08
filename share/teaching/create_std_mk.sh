#! /bin/bash

if [ -f CMakeLists.txt ]; then rm -i CMakeLists.txt || exit 0; fi

for dir in $(find . -type d); do
        if ls $dir/*.cpp >/dev/null 2>&1
        then
        pushd $dir >/dev/null
        echo "get_filename_component(NAME \${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_CXX_FLAGS \"\${MLR_CXX_FLAGS} \${CMAKE_CXX_FLAGS}\")

add_executable(\${NAME} main.cpp)

target_link_libraries(\${NAME} $@)

add_test(\${NAME}_test \${NAME} --gtest_output=xml:\${NAME}-unit.xml)" > CMakeLists.txt
        popd >/dev/null
        echo "add_subdirectory($dir)" >> CMakeLists.txt
        fi        

done
