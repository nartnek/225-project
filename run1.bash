#!/bin/bash

for i in {01..10}
do
    echo "Running scramble $i..."
    java -cp out rubikscube.Solver testcases/scramble$i.txt testcases/sol$i.txt
done
