#!/bin/sh

# Generate the instances and the sacrifice line
parallel -a /data/instance_classes.csv --colsep ' ' --files /app/instance_generator

# Save the results, including the sacrifice line, sorted
FILE=/data/instances.csv
cat /tmp/*.par | sort > "$FILE"

# Remove the first line, that must be the sacrifice line
tail -n +2 "$FILE" > "$FILE.tmp" && mv "$FILE.tmp" "$FILE"

# Print for validation
echo "LINES COUNT"
sort "$FILE" | uniq --count
