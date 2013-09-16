BASE   = $(shell realpath .)
NAME   = $(shell basename $(BASE))

ALL =	src/MT

makeAll: $(ALL:%=makePath/%) makeSubpaths/test makeSubpaths/course

cleanAll: $(ALL:%=cleanPath/%) cleanSubpaths/test cleanSubpaths/course

include make-generic