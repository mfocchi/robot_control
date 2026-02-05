#!/bin/bash

# numb_ run the code
NUM_RUNS=3
name_folder="obstacels_A"

START_POINT="[0.5, 1.5, -1.5]"
GOAL_POINT="[0.5, 1.5, -5.5]"
for ((i=1; i<=NUM_RUNS; i++))
do
    #file name folder
    DIR_NAME="result/${name_folder}_${i}"

    CURRENT_MIN_PROB=$(python3 -c "print(0.01 * $i)")
    echo "=================================================="
    echo "=================================================="
    echo "RUN $i: Output -> $DIR_NAME"
    echo "  -> Min Prob: $CURRENT_MIN_PROB"
    echo "  -> Start:    $START_POINT"
    echo "  -> Goal:     $GOAL_POINT"
    echo "=================================================="
    echo "=================================================="

    # EXPERIMENT_DIR="$DIR_NAME" CEM_MIN_PROB="$CURRENT_MIN_PROB" P0_INIT_STR="$START_POINT" PF_INIT_STR="$GOAL_POINT" python3 Main_cemmulti_simple.py
    EXPERIMENT_DIR="$DIR_NAME" CEM_MIN_PROB="$CURRENT_MIN_PROB" P0_INIT_STR="$START_POINT" PF_INIT_STR="$GOAL_POINT" python3 Main_cemmulti_parabolic.py
    # EXPERIMENT_DIR="$DIR_NAME" CEM_MIN_PROB="$CURRENT_MIN_PROB" P0_INIT_STR="$START_POINT" PF_INIT_STR="$GOAL_POINT" python3 Main_cemmulti_patch.py



    echo "Run $i completed."
    echo ""
    sleep 2
done

echo "All $NUM_RUNS experiments have completed."