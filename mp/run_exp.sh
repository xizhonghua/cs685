#!/bin/bash
for s in 0 1 2 3
do
    echo "Scene $s"

    ./bin/Planner "Scene$s.txt" -exp -m 2 > "normal-$s.log" &
    ./bin/Planner "Scene$s.txt" -exp -m 2 -p > "control-$s.log" &
    
    wait
done
