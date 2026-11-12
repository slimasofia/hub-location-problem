#!/bin/bash
#
# Runs the 6 AP tests
# 2 instances (40L, 40T) x 3 revenues (20, 30, 50), alpha fixed at 0.75.
#
# Usage:
#   ./run_ap_exact.sh <executable>

ALPHA=0.75
INSTANCES=("40L.txt" "40T.txt")
REVENUES=(20 30 50)
LOG_DIR="logs_ap"

EXEC="$1"

if [ -z "$EXEC" ]; then
    echo "Usage: $0 <executable>"
    exit 1
fi

if [ ! -x "$EXEC" ]; then
    echo "Error: '$EXEC' not found or not executable."
    exit 1
fi

for instance in "${INSTANCES[@]}"; do
    if [ ! -f "$instance" ]; then
        echo "Error: instance file '$instance' not found in $(pwd)."
        exit 1
    fi
done

mkdir -p "$LOG_DIR"

total=$(( ${#INSTANCES[@]} * ${#REVENUES[@]} ))
count=0
failed=0

for instance in "${INSTANCES[@]}"; do
    for revenue in "${REVENUES[@]}"; do
        count=$((count + 1))
        name="${instance%.txt}"
        log="$LOG_DIR/${name}_alpha${ALPHA}_rev${revenue}.log"

        echo "[$count/$total] $EXEC $instance $ALPHA $revenue"
        start=$(date +%s)

        "$EXEC" "$instance" "$ALPHA" "$revenue" 2>&1 | tee "$log"
        status=${PIPESTATUS[0]}

        end=$(date +%s)
        echo "  -> finished in $((end - start)) s (log: $log)"

        if [ $status -ne 0 ]; then
            echo "  -> failed with exit code $status"
            failed=$((failed + 1))
        elif grep -q "^Error:" "$log"; then
            echo "  -> CPLEX reported an error (no results saved for this test)"
            failed=$((failed + 1))
        fi
        echo
    done
done

echo "Finished: $total tests, $failed failed."