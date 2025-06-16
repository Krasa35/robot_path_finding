#!/bin/bash
iterations=5

rm -rf results
mkdir results
source .venv/bin/activate
cd RTB_toolbox
export PYTHONUNBUFFERED=1

echo "Benchmarking RRT & RRT* in environment A has started."
python3 98_benchmark.py -f 12_findPathRRT.py -i $iterations -r ../results/output_RRT_A.txt -- -i restricted_areaA.csv -o pointsRRT_A.csv 2>&1 | tee ../results/log_RRT_A.txt
#python3 98_benchmark.py -f 13_findPathRRTstar.py -i $iterations -r ../results/output_RRTstar_A.txt -- -i restricted_areaA.csv -o pointsRRTstar_A.csv 2>&1 | tee ../results/log_RRTstar_A.txt

echo "Benchmarking RRT & RRT* in environment B has started."
#python3 98_benchmark.py -f 12_findPathRRT.py -i $iterations -r ../results/output_RRT_B.txt -- -i restricted_areaB.csv -o pointsRRT_B.csv 2>&1 | tee ../results/log_RRT_B.txt
#python3 98_benchmark.py -f 13_findPathRRTstar.py -i $iterations -r ../results/output_RRTstar_B.txt -- -i restricted_areaB.csv -o pointsRRTstar_B.csv 2>&1 | tee ../results/log_RRTstar_B.txt

echo "Benchmarking RRT & RRT* in environment C has started."
#python3 98_benchmark.py -f 12_findPathRRT.py -i $iterations -r ../results/output_RRT_C.txt -- -i restricted_areaC.csv -o pointsRRT_C.csv 2>&1 | tee ../results/log_RRT_C.txt
#python3 98_benchmark.py -f 13_findPathRRTstar.py -i $iterations -r ../results/output_RRTstar_C.txt -- -i restricted_areaC.csv -o pointsRRTstar_C.csv 2>&1 | tee ../results/log_RRTstar_C.txt

deactivate
