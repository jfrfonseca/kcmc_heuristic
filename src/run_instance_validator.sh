#!/bin/sh

rm -rf /data/isomorphism_validation.csv
rm -rf /data/instances_validated.csv
rm -rf /data/cx2
mkdir -p /data/cx2

# Run the isomorphism validator
/app/isomorphism_validator /data/instances.csv > /data/isomorphism_validation.csv

# Read the instances from the CSV file (format them as space-separated parameters)
# Pipe the formatted instances to the instance validator
# Pipe the validation results to the output file
sed -e 's/^KCKC//' \
    -e 's/END$//' \
    -e 's/;/ /g' \
    -e 's/[|()]//g' \
    -e 's/[KCMEND]/ /g' \
    -e 's/[[:space:]]\+/ /g' -e 's/^ //; s/ $//' \
    /data/instances.csv \
    | xargs -r -L1 /app/instance_validator \
    > /data/instances_validated.csv
