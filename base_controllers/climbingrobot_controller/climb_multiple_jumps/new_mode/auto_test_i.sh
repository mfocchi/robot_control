#!/bin/bash

name_folder="test_i_E"

NUM_RUNS=10
export PARAMS_FILES="params_i"
TERRAIN_TYPE="gaussian_bumps" #"gaussian_bumps" | hemisphere
PERC_MIN_PROB=0.1

START_POINT="[0.5, 6.55, -18.49]" # Downward
GOAL_POINT="[0.5, 2.47, -6.3]"


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
