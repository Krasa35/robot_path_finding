#!/bin/bash
iterations=20

benchmark() {
	local algNo=$1
	local algName=$2
	local env=$3
	echo "Benchmarking ${algName} in environment ${env} has started."
	
	python3 98_benchmark.py -f ${algNo}_findPath${algName}.py -t 300 -o ../results/output_${algName}_${env}.txt -- -i restricted_area${env}.csv -o points${algName}_${env}.csv 2>&1 | tee -a ../results/log_${algName}_${env}.txt
}

rm -rf results
mkdir results
source .venv/bin/activate
cd RTB_toolbox
export PYTHONUNBUFFERED=1

for i in $(seq 1 $iterations); do
	echo "Iteration $i:"
	benchmark 12 RRT A
	benchmark 13 RRTstar A
	benchmark 12 RRT B
	benchmark 13 RRTstar B
	benchmark 12 RRT C
	benchmark 13 RRTstar C
done

deactivate


