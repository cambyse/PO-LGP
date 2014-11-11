headache -h insertIncludePathInCreatorProjects.header -c insertIncludePathInCreatorProjects.conf \
../examples/*/*/.*.includes \

#../projects/*/.*.includes \


sed 's/----\*\/\n\n/----\*\/\n/m' ../src/Core/array.h > ../src/Core/z.h

