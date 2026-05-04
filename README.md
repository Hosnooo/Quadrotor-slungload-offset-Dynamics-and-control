# Quadrotor Slung-Load Offset Dynamics and Control

MATLAB and Maple project for modeling, controlling, simulating, and visualizing a quadrotor carrying a slung load with an offset suspension point.

This repository contains Maple worksheets for symbolic model/controller generation, MATLAB implementations of the generated dynamics, a QSFA outer-loop controller, a reduced-to-full command mapper, a geometric inner-loop controller, full nonlinear dynamics simulations, offset sensitivity studies, and 3D animation tools.

<p align="center">
  <img src="https://github.com/user-attachments/assets/dd5fd2c1-538a-44d6-a1eb-833e176136b0" width="32%" alt="offset slung-load animation 1" />
  <img src="https://github.com/user-attachments/assets/1ed0d282-60f7-4816-a577-57bbb4857201" width="32%" alt="offset slung-load animation 2" />
  <img src="https://github.com/user-attachments/assets/0027f5ef-45de-488f-ae8c-34934cab431a" width="32%" alt="offset slung-load animation 3" />
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
├── Offset_Dynamics.mw
├── Offset_QSFA_U.mw
│
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

The symbolic source files are:

- `Offset_Dynamics.mw`
- `Offset_QSFA_U.mw`

`Offset_Dynamics.mw` contains the symbolic construction of the full offset slung-load dynamic model and is used to generate:

```text
model_offset.m
```

`Offset_QSFA_U.mw` contains the symbolic construction of the reduced QSFA effective-force model and controller and is used to generate:

```text
model_QSFA_U.m
controller_QSFA_U.m
```

The Maple worksheets reference auxiliary Maple scripts such as `MIMOTools.mpl` and `matlabsims.mpl`. These are only needed if the symbolic derivation or MATLAB-code generation is repeated. The MATLAB simulations can be run directly using the generated `.m` files included in the repository.

### Generated MATLAB model files

| File | Purpose |
|---|---|
| `model_offset.m` | Full nonlinear offset slung-load plant model. |
| `model_QSFA_U.m` | Reduced QSFA effective-force model. |
| `controller_QSFA_U.m` | Generated QSFA outer-loop controller. |

### Control and mapping files

| File | Purpose |
|---|---|
| `controller_QSFA_U.m` | Computes the reduced outer-loop effective-force command `U_d`. |
| `mapper_QSFA_U_to_inner.m` | Maps the reduced-model command to full thrust, attitude, angular velocity, and angular acceleration references. |
| `inner_loop_offset.m` | Geometric attitude inner-loop controller for the offset suspension-point system. |

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
| `run_outerloop.m` | Simulates the reduced QSFA outer-loop model using the generated reduced model and controller. |
| `run_full_dynamics.m` | Simulates the full nonlinear offset slung-load dynamics with the QSFA outer loop, mapper, and geometric inner loop. |
| `run_loops_diagnosis.m` | Diagnostic script for checking consistency between the outer loop, mapper, inner loop, and full plant. |
| `run_sensitivity_offset.m` | Runs a sensitivity study over multiple suspension-point offset vectors. |
| `animate_offset_slungload.m` | Loads a saved full-dynamics simulation and exports a 3D animation/video of the quadrotor, cable, and load. |

## Reference Trajectories

Reference functions are stored in `simulationtools/`.

| Function | Description |
|---|---|
| `ref_fig8.m` | Smooth figure-eight trajectory. |
| `ref_fig8_zsin.m` | Figure-eight trajectory with sinusoidal vertical motion. |
| `ref_helix.m` | Helical trajectory. |
| `ref_spiral.m` | Spiral trajectory. |
| `ref_regulation_steps.m` | Piecewise-constant regulation/step mission. |

The full-dynamics script also supports a `hover` case.

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

### 2. Open MATLAB in the repository folder

Start MATLAB and set the current folder to the repository root.

### 3. Add the simulation tools folder

Most scripts already include:

```matlab
addpath('simulationtools');
```

You can also add it manually:

```matlab
addpath('simulationtools');
```

### 4. Run the reduced outer-loop simulation

```matlab
run_outerloop
```

This runs the reduced QSFA effective-force model and controller.

### 5. Run the full nonlinear dynamics simulation

```matlab
run_full_dynamics
```

This runs the full offset slung-load simulation using:

- QSFA outer-loop controller
- Reduced-to-full mapper
- Geometric inner-loop attitude controller
- Full nonlinear offset plant model

### 6. Animate the full-dynamics result

After running `run_full_dynamics`, run:

```matlab
animate_offset_slungload
```

This loads the saved simulation file and exports an animation/video.

## Recommended Main Workflow

For a complete full-dynamics simulation and animation, run:

```matlab
run_full_dynamics
animate_offset_slungload
```

The full-dynamics script saves results to:

```text
results_offset_fulldynamics/<trajectory_name>/
```

For example:

```text
results_offset_fulldynamics/fig8_zsin/
```

The animation script loads:

```text
results_offset_fulldynamics/<trajectory_name>/sim_offset_QSFA_U.mat
```

and saves animation outputs under:

```text
results_offset_fulldynamics/<trajectory_name>/animation/
```

## Changing the Reference Trajectory

In `run_full_dynamics.m`, change:

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

In `run_outerloop.m`, change:

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

## Changing the Offset Vector

The offset vector is defined in the quadrotor body frame. In `run_full_dynamics.m`, the nominal offset is:

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
results_offset_outerloop/
results_offset_fulldynamics/
results_offset_sensitivity/
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
simulationtools/figureoptscall.m
```

to apply consistent figure formatting and LaTeX-style plot labels.

Figures can be exported using:

```matlab
saveFigureAsPDF(figHandle, fileName)
```

## Suggested Workflow for Development

1. Run `run_loops_diagnosis.m` to check consistency between the controller, mapper, inner loop, and full plant.
2. Run `run_outerloop.m` to verify the reduced QSFA controller.
3. Run `run_full_dynamics.m` to simulate the full nonlinear offset slung-load system.
4. Run `animate_offset_slungload.m` to create a 3D video of the saved simulation.
5. Run `run_sensitivity_offset.m` to compare tracking performance across multiple offset vectors.

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
