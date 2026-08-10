#!/bin/bash

name_folder="test_parabolic"

NUM_RUNS=5
export PARAMS_FILES="params"
TERRAIN_TYPE="rock" #"gaussian_bumps" | hemisphere | "rock"
PERC_MIN_PROB=0.1

START_POINT="[0.5, 8.51, -18.51 ]" 
GOAL_POINT="[0.5, 1.51, -5.31 ]"


echo ""
echo "=================================================="
echo "Starting experiments for terrain type: $TERRAIN_TYPE"
echo "=================================================="

for i in $(seq 1 ${NUM_RUNS});
do
    #file name folder
    DIR_NAME="result/${name_folder}_${i}_${TERRAIN_TYPE}"

    echo "=================================================="
    echo "=================================================="
    echo "RUN $i: Output -> $DIR_NAME"
    echo "  -> Min Prob Perc.: $PERC_MIN_PROB"
    echo "  -> Start         : $START_POINT"
    echo "  -> Goal          : $GOAL_POINT"
    echo "=================================================="
    echo "=================================================="

    EXPERIMENT_DIR="$DIR_NAME" PARAMS_FILES="$PARAMS_FILES" CEM_MIN_PROB="$PERC_MIN_PROB" P0_INIT_STR="$START_POINT" PF_INIT_STR="$GOAL_POINT" TERRAIN_TYPE="$TERRAIN_TYPE" python3 Main_cemmulti_parabolic.py
    # EXPERIMENT_DIR="$DIR_NAME" PARAMS_FILES="$PARAMS_FILES" CEM_MIN_PROB="$PERC_MIN_PROB" P0_INIT_STR="$START_POINT" PF_INIT_STR="$GOAL_POINT" TERRAIN_TYPE="$TERRAIN_TYPE" python3 Main_cemmulti_patch.py

    echo "Run $i completed."
    echo ""
    sleep 10
done

echo "All experiments completed."