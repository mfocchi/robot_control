#!/bin/bash

# numb_ run the code
NUM_RUNS=5
name_folder="test_final_v1"

TERRAIN_TYPES=( "hemisphere") #"gaussian_bumps" | hemisphere

PERC_MIN_PROB=( 0.01 0.05 0.1 0.2 0.4 0.6 0.8)

for TERRAIN_TYPE in "${TERRAIN_TYPES[@]}"
do

    if [ "$TERRAIN_TYPE" == "hemisphere" ]; then
        START_POINT="[0.5, 2.5, -7.5]" # Upward
        GOAL_POINT="[0.5, 6.5,-2.5]"
    else
        START_POINT="[0.5, 5.5, -8.5]" # Downward
        GOAL_POINT="[0.5, 2.5, -1.5]"
    fi
    echo ""
    echo "=================================================="
    echo "Starting experiments for terrain type: $TERRAIN_TYPE"
    echo "=================================================="
        
    for ((i=1; i<=NUM_RUNS; i++))
    do
        #file name folder
        DIR_NAME="result/${name_folder}_${i}_${TERRAIN_TYPE}"

        CURRENT_MIN_PROB=${PERC_MIN_PROB[$((i-1))]}
        echo "=================================================="
        echo "=================================================="
        echo "RUN $i: Output -> $DIR_NAME"
        echo "  -> Min Prob: $CURRENT_MIN_PROB"
        echo "  -> Start:    $START_POINT"
        echo "  -> Goal:     $GOAL_POINT"
        echo "=================================================="
        echo "=================================================="

        # EXPERIMENT_DIR="$DIR_NAME" CEM_MIN_PROB="$CURRENT_MIN_PROB" P0_INIT_STR="$START_POINT" PF_INIT_STR="$GOAL_POINT" python3 Main_cemmulti_simple.py
        # EXPERIMENT_DIR="$DIR_NAME" P0_INIT_STR="$START_POINT" PF_INIT_STR="$GOAL_POINT" TERRAIN_TYPE="$TERRAIN_TYPE" python3 Main_cemmulti_parabolic.py
        EXPERIMENT_DIR="$DIR_NAME" CEM_MIN_PROB="$CURRENT_MIN_PROB" P0_INIT_STR="$START_POINT" PF_INIT_STR="$GOAL_POINT" TERRAIN_TYPE="$TERRAIN_TYPE" python3 Main_cemmulti_patch.py

        echo "Run $i completed."
        echo ""
        sleep 2
    done

done

echo "All experiments have completed."


# Plot results for each experiment
echo ""
echo "=================================================="
echo "Starting visualization of results..."
echo "=================================================="

for TERRAIN_TYPE in "${TERRAIN_TYPES[@]}"
do
    echo ""
    echo "=================================================="
    echo "Plotting results for terrain type: $TERRAIN_TYPE"
    echo "=================================================="
        
    for ((i=1; i<=NUM_RUNS; i++))
    do
        DIR_NAME="result/${name_folder}_${i}_${TERRAIN_TYPE}"
        
        echo ""
        echo "=================================================="
        echo "PLOTTING RUN $i: $DIR_NAME"
        echo "=================================================="
        
        FOLDER_PLOT="\"$DIR_NAME\"" python3 Plot_result.py
        
        echo "Plotting for Run $i completed."
        sleep 2
    done
done
echo ""
echo "All visualizations have completed."