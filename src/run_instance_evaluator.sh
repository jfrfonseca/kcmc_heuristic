#!/bin/sh

# Read the instances from the CSV file (format them as space-separated parameters)
# Pipe the formatted instances to the instance evaluator
# Pipe the evaluation results to the output file
sed -e 's/^KCKC//' \
    -e 's/END$//' \
    -e 's/;/ /g' \
    -e 's/[|()]//g' \
    -e 's/[KCMEND]/ /g' \
    -e 's/[[:space:]]\+/ /g' -e 's/^ //; s/ $//' \
    /data/instances.csv \
    | xargs -r -L1 /app/instance_evaluator \
    > /data/instances_evaluated.txt
