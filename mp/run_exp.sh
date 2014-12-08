#!/bin/bash

rm *.log
for s in 0 1 2 3
do
    echo "Scene $s"

    ./bin/Planner "Scene$s.txt" -exp -m 2 > "normal-$s.log" &
    #./bin/Planner "Scene$s.txt" -exp -m 2 -bc > "control-$s.log" &
    ./bin/Planner "Scene$s.txt" -exp -m 2 -p -bc > "pc-$s.log" &
    wait
done
