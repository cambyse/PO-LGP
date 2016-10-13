#!/bin/bash

for file in ball-throwing-data-????-??-??_??-??-??
do
    # Create gnuplot readable file
    if [[ ! -f ${file}.gnuplot ]]
    then
        rewards=$(cat ${file} | grep -v ^# | grep -v ^+ | sed -e s/^.*,//g)
        let num=0
        for reward in ${rewards}
        do
            let num=${num}+1
            echo "${num} ${reward}" >> ${file}.gnuplot
        done
    fi

    # Then plot it
    gnuplot -e "plot '${file}.gnuplot' using 1:2 w l" -p
done
