# ALPINE Multi-Jump Optimization

## Overview

This code implements the multi-jump planner developed for the ALPINE climbing robot. The objective is to connect an initial position to a final target through a sequence of feasible jumps on a non-flat vertical terrain.

The planner follows a bi-level optimization structure:

1. The terrain is processed into a cost map and divided into landing patches.
2. The outer loop uses the Cross-Entropy Method (CEM) to select the number of intermediate jumps and the corresponding landing patches.
3. The inner loop solves an Optimal Control Problem (OCP) for each jump and checks dynamic feasibility, constraints, energy consumption, and the achieved landing position.
4. The CEM updates its sampling distribution using the best solutions until a final multi-jump sequence is obtained.

The inner optimization is executed through MATLAB/C++ MEX, while the high-level multi-jump logic is implemented in Python.

## Available Implementations

The repository contains **two main folders** corresponding to two versions of the multi-jump planner:

- `opti_patch/`: uses **landing-point optimization inside each selected patch**. The outer CEM selects the landing patch, while the inner optimization is allowed to search inside that patch for a better landing point. In this way, the optimizer is not forced to land exactly at the patch center and can select a point according to dynamic feasibility and terrain cost.

- `no_opti_inside_patch/`: runs the multi-jump planner **without optimizing the landing point inside the patch**. The selected patch is represented by a fixed target point, providing a simpler baseline that can be used to compare the effect of the additional landing-point optimization.

## Setup

The first requirement is to prepare the standard **Locosim environment**. No additional Python virtual environment is required for this project.


After setting up Locosim, the environment can be checked with:

```bash
python3 - <<'PY'
import matlab.engine
from base_controllers.components.terrain_manager import TerrainManager
print("Locosim and MATLAB Engine are available.")
PY
```

The optimization code also expects the compiled MATLAB/C++ solver in `codegen_mesh`. In the current implementation MATLAB adds it with:

```python
eng.addpath('../../codegen_mesh', nargout=0)
```

Therefore, keep the project inside the expected Locosim workspace structure or update this relative path if the directory layout is different.

## Running the Experiments

For the version with landing-point optimization inside each patch, move to `opti_patch/` and use:

```bash
cd opti_patch
chmod +x auto_test_dif_patch.sh
./auto_test_dif_patch.sh
```

The script can run repeated experiments on different terrain types. The available selections are:

- `rock`
- `gaussian`
- `hemisphere`

For example, the following commands select the Gaussian terrain and start the experiments:

```bash
cd opti_patch
sed -i 's/^TERRAIN_TYPE=.*/TERRAIN_TYPE="gaussian"/' auto_test_R_dif_patch.sh
chmod +x auto_test_dif_patch.sh
./auto_test_dif_patch.sh
```

To run the rock terrain instead:

```bash
sed -i 's/^TERRAIN_TYPE=.*/TERRAIN_TYPE="rock"/' auto_test_dif_patch.sh
./auto_test_dif_patch.sh
```

The main parameters that can be changed directly inside `auto_test_dif_patch.sh` are:

```bash
NUM_RUNS=5
TERRAIN_TYPE="rock"        # rock | gaussian | hemisphere
PERC_MIN_PROB=0.8

PATCH_SIZES_WIDTH=(10 10 20)
PATCH_SIZES_HEIGHT=(10 20 20)
```

Each width/height pair defines one patch discretization. With the values above, the script tests `10x10`, `10x20`, and `20x20` patch configurations, repeating each experiment `NUM_RUNS` times.

To run the version without landing-point optimization inside the patch:

```bash
cd no_opti_inside_patch
chmod +x auto_test_05.sh
./auto_test_05.sh
```

A single optimization can also be launched directly from either folder with:

```bash
python3 Main_cemmulti_patch.py
```

## Code Structure

The two folders share the same main multi-jump architecture. The most relevant files are:

- `Main_cemmulti_patch.py`: main entry point. It initializes the terrain and patches, runs the CEM iterations, evaluates the population, and stores the best solution.
- `params.py`: contains terrain, patch, CEM, OCP, fitness, threading, MATLAB Engine, and output parameters.
- `algo_patch.py`: implements CEM population sampling, elite selection, and probability distribution updates.
- `BilevelOpt.py`: evaluates each candidate multi-jump sequence by solving the OCPs sequentially.
- `auto_test_R_dif_patch.sh`: runs repeated experiments for different terrains and patch discretizations in `opti_patch/`.
- `auto_test_05.sh`: runs repeated experiments for the version in `no_opti_inside_patch/`.
- `Plot_result.py`: loads and visualizes the optimization results.

## Output

Each experiment is saved inside the `result/` directory of the selected implementation. When `auto_test_dif_patch.sh` is used, the output directory includes the patch dimensions, run number, and terrain type, for example:

```text
result/test_patches_W10_H10_run1_rock/
```

The main generated files include the processed terrain and patches, simulation parameters, iteration reports, CEM history, timing information, and `info_for_gazebo.json`, which contains the optimized target points and terrain information required by the Gazebo simulation.

## Some Notes:

1. The folder `/old_test_and_examples` contains all the legacy code and prototypes used during development.
These files are kept as a reference, backup, and documentation of previous functions and simple examples.
The current implementation is located in `/no_opti_inside_patch` and `/opti/patch`.
