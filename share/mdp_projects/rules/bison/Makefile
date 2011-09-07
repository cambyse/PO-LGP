BASE = ..

include $(BASE)/make-generic

grammar:
	bison rule_parser.y
	cc rule_parser.tab.c -lm -o rule
