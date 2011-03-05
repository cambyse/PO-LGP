PACKAGE = libORS

ALL = 	SWIFT++_1.2/src \
	SWIFT++_1.2/decomposer/RAPID \
	SWIFT++_1.2/decomposer/src \
	ANN_1.1/src \
	libcolorseg/src

makeAll: $(ALL:%=make/%)

cleanAll: $(ALL:%=clean/%)

make/%::
	make -C $*

clean/%::
	make clean -C $*
