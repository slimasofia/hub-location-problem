#!/bin/bash
#
# Runs the AP tests with the heuristic:
# 2 instances (40L, 40T) x 3 revenues (20, 30, 50), alpha fixed at 0.75,
# each combination repeated 10 times (60 runs in total).
#
# Usage:
#   ./run_ap_heuristic.sh <executable> [repetitions]
#
# A pause of 1 second is made between runs. The heuristic seeds its random
# generator with srand(time(NULL)), which only changes once per second, so runs
# started within the same second would repeat exactly the same result.

ALPHA=0.75
INSTANCES=("40L.txt" "40T.txt")
REVENUES=(20 30 50)

EXEC="$1"
REPS="${2:-10}"
SLEEP_BETWEEN="${SLEEP_BETWEEN:-1}"

if [ -z "$EXEC" ]; then
    echo "Usage: $0 <executable> [repetitions]"
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

total=$(( ${#INSTANCES[@]} * ${#REVENUES[@]} * REPS ))
count=0
failed=0

for instance in "${INSTANCES[@]}"; do
    for revenue in "${REVENUES[@]}"; do
        for (( rep = 1; rep <= REPS; rep++ )); do
            count=$((count + 1))
            echo "[$count/$total] $EXEC $instance $ALPHA $revenue (run $rep of $REPS)"

            "$EXEC" "$instance" "$ALPHA" "$revenue"
            status=$?

            if [ $status -ne 0 ]; then
                echo "  -> failed with exit code $status"
                failed=$((failed + 1))
            fi

            # Different seed for the next run
            if [ $count -lt $total ]; then
                sleep "$SLEEP_BETWEEN"
            fi
        done
    done
done

echo
echo "Finished: $total runs, $failed failed."
