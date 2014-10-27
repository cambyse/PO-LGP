
libname=${PWD##*/}

echo "find_package(Qt5Core)" >> CMakeLists.txt
echo "" >> CMakeLists.txt
echo 'set(CMAKE_CXX_FLAGS "-std=c++0x")' >> CMakeLists.txt
echo "" >> CMakeLists.txt
echo "add_library($libname" >> CMakeLists.txt
for f in `ls | grep ".h$\|.cpp$"`; do
    echo "    $f" >> CMakeLists.txt
done
echo ")" >> CMakeLists.txt
echo "" >> CMakeLists.txt
echo "target_include_directories($libname"' PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})' >> CMakeLists.txt
echo "" >> CMakeLists.txt
echo "target_link_libraries($libname PUBLIC Qt5::Core)" >> CMakeLists.txt

