#!/bin/bash

name_folder="test_patches"

NUM_RUNS=5
export PARAMS_FILES="params"
TERRAIN_TYPE="hemisphere"
PERC_MIN_PROB=0.8

if [ "$TERRAIN_TYPE" == "rock" ]; then
    START_POINT="[ 0.28, 2.5, -6.10104 ]" 
    GOAL_POINT="[ 0.28, 3.5, -16.3 ]"
elif [ "$TERRAIN_TYPE" == "gaussian_bumps" ]; then
    START_POINT="[ 0.5, 6.55, -18.49 ]" 
    GOAL_POINT="[ 0.5, 2.47, -6.3 ]"
elif [ "$TERRAIN_TYPE" == "hemisphere" ]; then
    START_POINT="[ 0.5, 6.51, -15.51 ]" 
    GOAL_POINT="[ 0.5, 2.51, -5.51 ]"
else
    echo "Invalid TERRAIN_TYPE: $TERRAIN_TYPE"
    exit 1
fi

# START_POINT="[0.5, 8.51, -18.51 ]" 
# GOAL_POINT="[0.5, 1.51, -5.31 ]"

# PATCH_SIZES_WIDTH=(20)
# PATCH_SIZES_HEIGHT=(20)
PATCH_SIZES_WIDTH=(10 10 20)
PATCH_SIZES_HEIGHT=(10 20 20)

echo ""
echo "=================================================="
echo "Starting experiments for terrain type: $TERRAIN_TYPE"
echo "=================================================="
for idx in "${!PATCH_SIZES_WIDTH[@]}"; do
    W="${PATCH_SIZES_WIDTH[$idx]}"
    H="${PATCH_SIZES_HEIGHT[$idx]}"

    for i in $(seq 1 ${NUM_RUNS});
    do
        #file name folder
        DIR_NAME="result/${name_folder}_W${W}_H${H}_run${i}_${TERRAIN_TYPE}"

        echo "=================================================="
        echo "=================================================="
        echo "RUN $i: Output -> $DIR_NAME"
        echo "  -> Min Prob Perc.: $PERC_MIN_PROB"
        echo "  -> Start         : $START_POINT"
        echo "  -> Goal          : $GOAL_POINT"
        echo "=================================================="
        echo "=================================================="

        # EXPERIMENT_DIR="$DIR_NAME" PARAMS_FILES="$PARAMS_FILES" CEM_MIN_PROB="$PERC_MIN_PROB" P0_INIT_STR="$START_POINT" PF_INIT_STR="$GOAL_POINT" TERRAIN_TYPE="$TERRAIN_TYPE" PATCH_WIDTH="$W" PATCH_HEIGHT="$H" python3 Main_cemmulti_parabolic.py
        EXPERIMENT_DIR="$DIR_NAME" PARAMS_FILES="$PARAMS_FILES" CEM_MIN_PROB="$PERC_MIN_PROB" P0_INIT_STR="$START_POINT" PF_INIT_STR="$GOAL_POINT" TERRAIN_TYPE="$TERRAIN_TYPE" PATCH_WIDTH="$W" PATCH_HEIGHT="$H" python3 Main_cemmulti_patch.py

        echo "Run $i completed."
        echo ""
        sleep 10
    done
done


echo "All experiments completed."