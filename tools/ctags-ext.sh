#!/bin/sh

# invoke from share;
# in vim: set tags+=,/PATH/To/SHARE/tags

for i in include src;
do
	find $i -type f -name '*.cpp'
	find $i -type f -name '*.h'
	find $i -type f -name '*.c'
done |\
	     xargs ctags  \
	--regex-C++="/FIELD\(.*, *([^)]*)\)/\1/" \
	--regex-C++="/FIELD\(.*, *([^)]*)\)/set_\1/" \
	--regex-C++="/FIELD\(.*, *([^)]*)\)/get_\1/" \
	--regex-C++="/FIELD\(.*, *([^)]*)\)/reg_\1/"

