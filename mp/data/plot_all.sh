#!/bin/bash

# execute command in parallel use at most $NUM_CORES threads

function nrwait() {
    local threads
    if [[ -z $1 ]] ; then
        threads=2
    else
        threads=$1
    fi
    
    while [[ $(jobs -p | wc -l) -ge $threads ]] ; do
    sleep 0.33;
    done
}

for f in *.traj; do
    ./plotCol.m $f 1 6 'X (m)' 'Time (s)' &
    ./plotCol.m $f 2 6 'Y (m)' 'Time (s)' &
    ./plotCol.m $f 3 6 'Theta (rad)' 'Time (s)' &
    ./plotCol.m $f 4 6 'Velocity (m/s)' 'Time (s)' &
    nrwait 4
done
wait

