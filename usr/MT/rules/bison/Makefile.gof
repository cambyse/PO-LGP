BASE = ..

include $(BASE)/build/generic.mk

grammar:
	bison rule_parser.y
	cc rule_parser.tab.c -lm -o rule
