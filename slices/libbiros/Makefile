NAME   = $(shell basename $(PWD))

ALL =	src/MT src/biros use_cases/vision/extern/libcolorseg/src

makeAll: $(ALL:%=make/%) useCases

cleanAll: $(ALL:%=clean/%) cleanUseCases

make/%::
	make -C $*

clean/%::
	make clean -C $*

useCases::
	-@find -L use_cases -maxdepth 1 -follow -type d -not -name 'use_cases' -not -name 'SHARE' -exec make -C {} \;

cleanUseCases::
	-@find -L use_cases -maxdepth 1 -follow -type d -not -name 'use_cases' -not -name 'SHARE' -exec make -C {} clean \;

doc::
	make -C doc guide doxy

zip::
	cd ..;  rm -f $(NAME).tgz;  tar cvzf $(NAME).tgz $(NAME) --dereference --exclude-vcs --exclude-from tar.exclude
