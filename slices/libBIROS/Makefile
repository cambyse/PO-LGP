NAME   = $(shell basename $(PWD))

ALL =	src/MT src/biros

makeAll: $(ALL:%=make/%)

cleanAll: $(ALL:%=clean/%) 

make/%::
	make -C $*

clean/%::
	make clean -C $*

doc::
	make -C doc guide doxy

zip::
	cd ..;  rm -f $(NAME).tgz;  tar cvzf $(NAME).tgz $(NAME) --dereference --exclude "SHARE" --exclude ".kdev*" --exclude "*.svn" --exclude ".git*" --exclude "cudaSDK/*" --exclude "opencv-2.1/*" --exclude "cuda/*"
