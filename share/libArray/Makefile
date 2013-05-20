BASE   = $(shell realpath .)
NAME   = $(shell basename $(BASE))

ALL =	src

makeAll: $(ALL:%=makePath/%) makeSubpaths/examples

cleanAll: $(ALL:%=cleanPath/%) cleanSubpaths/examples

include make-generic