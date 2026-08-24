#!/bin/bash

name_folder="test_05"

NUM_RUNS=5
export PARAMS_FILES="params"
TERRAIN_TYPE="rock" #gaussian_bumps | hemisphere | rock
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

    # EXPERIMENT_DIR="$DIR_NAME" CEM_MIN_PROB="$CURRENT_MIN_PROB" P0_INIT_STR="$START_POINT" PF_INIT_STR="$GOAL_POINT" python3 Main_cemmulti_simple.py
    # EXPERIMENT_DIR="$DIR_NAME" P0_INIT_STR="$START_POINT" PF_INIT_STR="$GOAL_POINT" TERRAIN_TYPE="$TERRAIN_TYPE" python3 Main_cemmulti_parabolic.py
    EXPERIMENT_DIR="$DIR_NAME" PARAMS_FILES="$PARAMS_FILES" CEM_MIN_PROB="$PERC_MIN_PROB" P0_INIT_STR="$START_POINT" PF_INIT_STR="$GOAL_POINT" TERRAIN_TYPE="$TERRAIN_TYPE" python3 Main_cemmulti_patch.py

    echo "Run $i completed."
    echo ""
    sleep 10
done

echo "All experiments completed."

# # Plot results for each experiment
# echo ""
# echo "=================================================="
# echo "Starting visualization of results..."
# echo "=================================================="
#
# for TERRAIN_TYPE in "${TERRAIN_TYPES[@]}"
# do
#    echo ""
#    echo "=================================================="
#    echo "Plotting results for terrain type: $TERRAIN_TYPE"
#    echo "=================================================="
#
#    for ((i=1; i<=${#PERC_MIN_PROB[@]}; i++))
#    do
#        DIR_NAME="result/${name_folder}_${i}_${TERRAIN_TYPE}"
#
#        echo ""
#        echo "=================================================="
#        echo "PLOTTING RUN $i: $DIR_NAME"
#        echo "=================================================="
#
#        FOLDER_PLOT="\"$DIR_NAME\"" python3 Plot_result.py
#
#        echo "Plotting for Run $i completed."
#        sleep 2
#    done
# done
# echo ""
# echo "All visualizations have completed."
