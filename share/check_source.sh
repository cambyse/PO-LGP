#! /bin/sh

find . -type d -not -iwholename '*retired*'|while read dir; do
        find $dir -maxdepth 1 -name '*.cpp' -type f|while read sourcename; do
                grep -q $(basename $sourcename) $dir/CMakeLists.txt 2>/dev/null || echo $sourcename
        done
done

