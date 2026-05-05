# Quadrotor Slung-Load Offset Dynamics and Control

MATLAB and Maple project for modeling, controlling, simulating, and visualizing a quadrotor carrying a slung load with an offset suspension point.

This repository contains Maple worksheets for symbolic model/controller generation, MATLAB implementations of the generated dynamics, a QSFA outer-loop controller, a reduced-to-full command mapper, a geometric inner-loop controller, full nonlinear dynamics simulations, offset sensitivity studies, and 3D animation tools.

<p align="center">
  <img src="https://github.com/user-attachments/assets/d22b210f-6634-42d3-8962-979cf67d7480" width="70%" alt="Offset slung-load animation view 1" />
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/6c8f2e35-e032-4af3-9e70-c89803304921" width="49%" alt="Offset slung-load animation view 2" />
  <img src="https://github.com/user-attachments/assets/5468919f-e939-476c-ac75-fee4f3531a76" width="49%" alt="Offset slung-load animation view 3" />
</p>

## Overview

This project studies the dynamics and control of an offset quadrotor slung-load system. The suspended load is connected to the quadrotor through a cable attached at an offset point rather than directly at the vehicle center of mass. This offset introduces additional coupling between the quadrotor attitude, cable direction, load motion, thrust command, and control torque.

The main focus areas are:

- Symbolic derivation of reduced and full offset slung-load dynamics
- Maple-generated MATLAB model and controller files
- QSFA effective-force outer-loop control
- Mapping of reduced-model commands to full thrust and attitude commands
- Geometric inner-loop attitude control for the offset suspension-point system
- Full nonlinear offset slung-load dynamics simulation
- Offset sensitivity analysis
- Load trajectory tracking for multiple reference paths
- 3D animation and video export of the quadrotor, cable, and suspended load

## Repository Structure

```text
.
├── README.md
├── Offset_Dynamics.mw
├── Offset_QSFA_U.mw
│
└── matlab_core_offset/
    ├── model_offset.m
    ├── model_QSFA_U.m
    ├── controller_QSFA_U.m
    ├── mapper_QSFA_U_to_inner.m
    ├── inner_loop_offset.m
    │
    ├── run_outerloop.m
    ├── run_full_dynamics.m
    ├── run_loops_diagnosis.m
    ├── run_sensitivity_offset.m
    ├── animate_offset_slungload.m
    │
    └── simulationtools/
        ├── add_tracking_gains_from_poles.m
        ├── figureoptscall.m
        ├── ref_fig8.m
        ├── ref_fig8_zsin.m
        ├── ref_helix.m
        ├── ref_regulation_steps.m
        ├── ref_spiral.m
        ├── saveFigureAsPDF.m
        └── simulate_closed_loop.m
```

## Main Components

### Maple worksheets

The symbolic source files are located in the repository root:

- `Offset_Dynamics.mw`
- `Offset_QSFA_U.mw`

`Offset_Dynamics.mw` contains the symbolic construction of the full offset slung-load dynamic model and is used to generate:

```text
matlab_core_offset/model_offset.m
```

`Offset_QSFA_U.mw` contains the symbolic construction of the reduced QSFA effective-force model and controller and is used to generate:

```text
matlab_core_offset/model_QSFA_U.m
matlab_core_offset/controller_QSFA_U.m
```

The Maple worksheets reference auxiliary Maple scripts such as `MIMOTools.mpl` and `matlabsims.mpl`. These auxiliary scripts are only needed if the symbolic derivation or MATLAB-code generation is repeated. The MATLAB simulations can be run directly using the generated `.m` files included in `matlab_core_offset/`.

### MATLAB core folder

All executable MATLAB simulation files are stored in:

```text
matlab_core_offset/
```

Run MATLAB scripts from inside this folder, because the scripts use relative paths such as:

```matlab
addpath('simulationtools');
```

### Generated MATLAB model files

| File | Purpose |
|---|---|
| `matlab_core_offset/model_offset.m` | Full nonlinear offset slung-load plant model. |
| `matlab_core_offset/model_QSFA_U.m` | Reduced QSFA effective-force model. |
| `matlab_core_offset/controller_QSFA_U.m` | Generated QSFA outer-loop controller. |

### Control and mapping files

| File | Purpose |
|---|---|
| `matlab_core_offset/controller_QSFA_U.m` | Computes the reduced outer-loop effective-force command `U_d`. |
| `matlab_core_offset/mapper_QSFA_U_to_inner.m` | Maps the reduced-model command to full thrust, attitude, angular velocity, and angular acceleration references. |
| `matlab_core_offset/inner_loop_offset.m` | Geometric attitude inner-loop controller for the offset suspension-point system. |

The full-dynamics control architecture is:

```text
Load reference
     ↓
QSFA outer-loop controller
     ↓
Effective-force command U_d
     ↓
Mapper to thrust and attitude commands
     ↓
Geometric inner-loop controller
     ↓
Full nonlinear offset slung-load plant
```

## Simulation Scripts

| Script | Description |
|---|---|
| `matlab_core_offset/run_outerloop.m` | Simulates the reduced QSFA outer-loop model using the generated reduced model and controller. |
| `matlab_core_offset/run_full_dynamics.m` | Simulates the full nonlinear offset slung-load dynamics with the QSFA outer loop, mapper, and geometric inner loop. |
| `matlab_core_offset/run_loops_diagnosis.m` | Diagnostic script for checking consistency between the outer loop, mapper, inner loop, and full plant. |
| `matlab_core_offset/run_sensitivity_offset.m` | Runs a sensitivity study over multiple suspension-point offset vectors. |
| `matlab_core_offset/animate_offset_slungload.m` | Loads a saved full-dynamics simulation and exports a 3D animation/video of the quadrotor, cable, and load. |

## Reference Trajectories

Reference functions are stored in:

```text
matlab_core_offset/simulationtools/
```

| Function | Description |
|---|---|
| `ref_fig8.m` | Smooth figure-eight trajectory. |
| `ref_fig8_zsin.m` | Figure-eight trajectory with sinusoidal vertical motion. |
| `ref_helix.m` | Helical trajectory. |
| `ref_spiral.m` | Spiral trajectory. |
| `ref_regulation_steps.m` | Piecewise-constant regulation/step mission. |

The full-dynamics script also supports a built-in `hover` case.

## Requirements

The MATLAB simulations require:

- MATLAB

For symbolic regeneration from the worksheets:

- Maple
- Auxiliary Maple scripts referenced by the worksheets, such as `MIMOTools.mpl` and `matlabsims.mpl`

No external Python or C++ dependencies are required for the included MATLAB simulations.

## Getting Started

### 1. Clone the repository

```bash
git clone https://github.com/Hosnooo/Quadrotor-slungload-offset-Dynamics-and-control.git
cd Quadrotor-slungload-offset-Dynamics-and-control
```

### 2. Enter the MATLAB core folder

The runnable MATLAB scripts are inside `matlab_core_offset/`.

```bash
cd matlab_core_offset
```

### 3. Open MATLAB in `matlab_core_offset/`

Start MATLAB with the current folder set to:

```text
Quadrotor-slungload-offset-Dynamics-and-control/matlab_core_offset/
```

This is important because the scripts use relative paths to the `simulationtools/` folder.

### 4. Add the simulation tools folder

Most scripts already include:

```matlab
addpath('simulationtools');
```

You can also add it manually:

```matlab
addpath('simulationtools');
```

### 5. Run the reduced outer-loop simulation

```matlab
run_outerloop
```

This runs the reduced QSFA effective-force model and controller.

### 6. Run the full nonlinear dynamics simulation

```matlab
run_full_dynamics
```

This runs the full offset slung-load simulation using:

- QSFA outer-loop controller
- Reduced-to-full mapper
- Geometric inner-loop attitude controller
- Full nonlinear offset plant model

### 7. Animate the full-dynamics result

Before running the animation, make sure the `trajectory_name` in `animate_offset_slungload.m` matches the trajectory simulated by `run_full_dynamics.m`.

For example, if `run_full_dynamics.m` uses:

```matlab
trajectory_name = 'fig8_zsin';
```

then set the same value in `animate_offset_slungload.m`:

```matlab
trajectory_name = 'fig8_zsin';
```

Then run:

```matlab
animate_offset_slungload
```

This loads the saved simulation file and exports an animation/video.

## Recommended Main Workflow

From inside `matlab_core_offset/`, run:

```matlab
run_full_dynamics
animate_offset_slungload
```

The full-dynamics script saves results to:

```text
matlab_core_offset/results_offset_fulldynamics/<trajectory_name>/
```

For example:

```text
matlab_core_offset/results_offset_fulldynamics/fig8_zsin/
```

The animation script loads:

```text
matlab_core_offset/results_offset_fulldynamics/<trajectory_name>/sim_offset_QSFA_U.mat
```

and saves animation outputs under:

```text
matlab_core_offset/results_offset_fulldynamics/<trajectory_name>/animation/
```

## Changing the Reference Trajectory

In `matlab_core_offset/run_full_dynamics.m`, change:

```matlab
trajectory_name = 'fig8_zsin';
```

Supported trajectory names include:

```text
hover
fig8
fig8_zsin
helix
spiral
regulation_steps
```

In `matlab_core_offset/run_outerloop.m`, change:

```matlab
ref_fun = @ref_fig8;
```

to one of:

```matlab
ref_fun = @ref_fig8;
ref_fun = @ref_fig8_zsin;
ref_fun = @ref_helix;
ref_fun = @ref_spiral;
ref_fun = @ref_regulation_steps;
```

For animation, update the `trajectory_name` inside `matlab_core_offset/animate_offset_slungload.m` to match the saved simulation folder.

## Changing the Offset Vector

The offset vector is defined in the quadrotor body frame. In `matlab_core_offset/run_full_dynamics.m`, the nominal offset is:

```matlab
p.r = [0.25; 0; 0];
```

The corresponding scalar parameters used by the generated model are:

```matlab
p.r__1 = p.r(1);
p.r__2 = p.r(2);
p.r__3 = p.r(3);
```

To test another offset, modify `p.r` and update the scalar fields accordingly.

For batch comparison over multiple offsets, run:

```matlab
run_sensitivity_offset
```

The sensitivity script evaluates several offset cases and saves comparison plots and summary metrics.

## Output Files

Depending on the script, outputs may include:

- `.mat` simulation data
- `.txt` summary reports
- `.png` figures
- `.fig` MATLAB figures
- `.pdf` exported figures
- `.mp4` or `.avi` animation videos

Typical output folders include:

```text
matlab_core_offset/results_offset_outerloop/
matlab_core_offset/results_offset_fulldynamics/
matlab_core_offset/results_offset_sensitivity/
```

Full-dynamics outputs include files such as:

```text
sim_offset_QSFA_U.mat
summary.txt
traj3d.png
tracking_position.png
errors_position.png
tracking_velocity.png
inputs.png
outer_mapper_commands.png
attitude_errors.png
angular_rates.png
cable_states.png
mapper_consistency.png
```

Animation outputs include:

```text
offset_slung_load_animation.mp4
offset_slung_load_animation.avi
offset_slung_load_animation_final_frame.png
offset_slung_load_animation_final_frame.fig
```

## Typical Metrics

The scripts compute and save metrics such as:

- Position RMSE
- Maximum absolute position error
- Final absolute position error
- Maximum load-position error norm
- Final load-position error norm
- Attitude error norm
- Angular-velocity error norm
- Maximum thrust command
- Maximum control torque
- Cable unit-vector constraint error
- Cable angular-velocity orthogonality error
- Mapper attitude orthogonality and determinant errors

## Notes on the Dynamic Model

The full state used by the full offset model is:

```text
x = [x_L; q; R(:); v_L; omega; Omega]
```

where:

- `x_L` is the load position
- `q` is the cable direction unit vector
- `R` is the quadrotor attitude matrix
- `v_L` is the load velocity
- `omega` is the cable angular velocity
- `Omega` is the quadrotor body angular velocity

The full plant model has affine form:

```text
xdot = f(x,p) + G(x,p)u
```

where the plant input is:

```text
u = [u_t; tau_cm]
```

Here, `u_t` is the total thrust command and `tau_cm` is the torque command about the quadrotor center of mass.

## Notes on Figure Formatting

The repository uses:

```text
matlab_core_offset/simulationtools/figureoptscall.m
```

to apply consistent figure formatting and LaTeX-style plot labels.

Figures can be exported using:

```matlab
saveFigureAsPDF(figHandle, fileName)
```

## Suggested Workflow for Development

1. Open MATLAB in `matlab_core_offset/`.
2. Run `run_loops_diagnosis.m` to check consistency between the controller, mapper, inner loop, and full plant.
3. Run `run_outerloop.m` to verify the reduced QSFA controller.
4. Run `run_full_dynamics.m` to simulate the full nonlinear offset slung-load system.
5. Run `animate_offset_slungload.m` to create a 3D video of the saved simulation.
6. Run `run_sensitivity_offset.m` to compare tracking performance across multiple offset vectors.

## Citation

If this repository is used for academic work, cite the repository and any related report, thesis, or paper associated with the model derivation and control design.

```bibtex
@software{quadrotor_slungload_offset_dynamics_control,
  title  = {Quadrotor Slung-Load Offset Dynamics and Control},
  author = {Mohssen Elshaar},
  year   = {2026},
  url    = {https://github.com/Hosnooo/Quadrotor-slungload-offset-Dynamics-and-control},
  note   = {MATLAB and Maple implementation for offset slung-load dynamics, QSFA control, and full nonlinear simulation}
}
```
