#!/bin/bash
for s in 0 1 2 3
do
    for m in normal pc
    do
        logfile="$m-$s.log"
        tmp=$logfile.tmp
        
        #count solved        
        solved=`cat $logfile | grep "Solved = 1" | wc -l`
        
        # stats time
        cat $logfile | grep "Solved = " | cut -f3 -d ' ' | stats > $tmp
        mean_t=`cat $tmp | grep "mean" | cut -f14 -d ' '`
        stddev_t=`cat $tmp | grep "std" | cut -f12 -d ' '`
        
         # stats path time
        cat $logfile | grep "Solved = " | cut -f17 -d ' ' | stats > $tmp
        mean_pt=`cat $tmp | grep "mean" | cut -f14 -d ' '`
        stddev_pt=`cat $tmp | grep "std" | cut -f12 -d ' '`

        # stats path length
        cat $logfile | grep "Solved = " | cut -f22 -d ' ' | stats > $tmp
        mean_pl=`cat $tmp | grep "mean" | cut -f14 -d ' '`
        stddev_pl=`cat $tmp | grep "std" | cut -f12 -d ' '`

        # stats node
        cat $logfile | grep "Solved = " | cut -f12 -d ' ' | stats > $tmp
        mean_n=`cat $tmp | grep "mean" | cut -f14 -d ' '`
        stddev_n=`cat $tmp | grep "std" | cut -f12 -d ' '`
        
        
        rm $tmp
        echo $s $m $solved $mean_t $stddev_t $mean_pt $stddev_pt $mean_pl $stddev_pl # $mean_n $stddev_n
    done
    wait
done
