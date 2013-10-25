#!/bin/bash

## check input and print help
if [ $# -lt 1 -o ! -f "$1" ]; then
    echo "Please give a valid data file to process, such as"
    echo
    echo "    $0 /path/to/data/file"
    echo
    exit 0
fi

## get output file name
out_file_name="$(echo "$1" | sed 's/\(.*\)\.\(.*\)/\1_R.\2/')"
echo "Writing output to file '$out_file_name'"

## get number of lines in file
number_of_lines="$(wc -l < "$1")"

## process data
current_line_count=0
found_header_line="false"
found_additional_columns="false"
while read line; do
    # increment line count and print progress message
    ((current_line_count++))
    echo -en "\r$(((current_line_count*100)/number_of_lines))% ($current_line_count of $number_of_lines lines)"
    # find first line of the form '# Episode ... ' and use as heading (without the leading '# ')
    if [ "$found_header_line" == "false" -a "$(echo "$line" | sed 's/^\(# Episode\)*.*/\1/')" != "" ]; then
	head_line="$line"
	head_line="$(echo "$head_line" | sed 's/#\s\+//')"
	found_header_line="true"
    fi
    # if heading was found find first line (after that) that does not start with
    # a '#' and extract additional columns
    if [ "$found_header_line" == "true" -a "$found_additional_columns" == "false" ]; then
	if [ "$(echo "$line" | sed 's/^\(#\)*.*/\1/')" == "" ]; then
	    add_cols="$line"
	    add_cols="$(echo "$add_cols" | sed 's/:\?\(\s\|^\)[-e0-9.]\+//g')"  # remove fields that contain only numbers
	    add_cols="$(echo "$add_cols" | sed 's/\s\+/\t/g')" 		      	# replace (multiple) white spaces with a single TAB
	    add_cols="$(echo "$add_cols" | sed 's/\s$//')" 	  	      	# remove trailing white space
	    add_cols="$(echo "$add_cols" | sed 's/^\s//')" 	  	      	# remove leading white space
	    echo "$head_line	$add_cols" > "$out_file_name" 	  	      	# write the whole heading to file
	    found_additional_columns="true"
	fi
    fi
    # if heading (including additional columns) is complete write data
    if [ "$found_header_line" == "true" -a "$found_additional_columns" == "true" ]; then
	# ignore empty lines ('read' returns empty lines also for lines with
	# only whitespace in it) and lines starting with a '#'
	if [ "$line" != "" -a "$(echo "$line" | sed 's/^\([#]\)*.*/\1/')" == "" ]; then
	    data_line="$line"
	    data_line="$(echo "$data_line" | sed 's/\(\(\s\|^\)[^-e0-9.][^[:space:]]\+\)/ /g')"  # remove fields that contain non-numbers
	    data_line="$(echo "$data_line" | sed 's/\s\+/\t/g')" 			  	 # replace (multiple) white spaces with a single TAB
	    data_line="$(echo "$data_line" | sed 's/\s$//')" 	  	       		  	 # remove trailing white space
	    data_line="$(echo "$data_line" | sed 's/^\s//')" 	  	       		  	 # remove leading white space
	    echo "$data_line" >> "$out_file_name"			       		  	 # write to file
	fi
    fi
done < "$1"
echo
