#!/bin/bash

timestamp() {
  date +"%Y-%m-%d :: %H:%M:%S"
}

rd="results/"
par="--use-spp true --sol-constructor greedy --ppsize 0.83 --prob-eval 0.57 --ges-max-iter 4000 --ges-ppsize 0.15 --lns-min-q 2 --lns-max-mult-q 0.2 --lns-max-iter 970 --lns-lsize 1540 --lns-weight-shaw 6 --lns-weight-random 3 --lns-min-k 1 --lns-max-k 4"   

echo "Running set of tests: "$(timestamp)
mkdir $rd
./runner.sh ../instances/pdp_600/ $rd 3 1 "$par"
echo "Done: "$(timestamp)
echo "----------"

echo "DONE"
