<div align="center">

<br/>

<!-- HERO BANNER -->
<img src="https://www.mathworks.com/help/examples/rl/win64/AutomaticParkingValetWithUnrealEngineSimulationExample_01.png" width="100%" alt="Autonomous Parking Valet — Unreal Engine 3D Simulation" style="border-radius:12px"/>

<br/><br/>

# 🚗 Autonomous Parking Valet
### Hybrid Control · Adaptive MPC + Deep Reinforcement Learning · Unreal Engine® Digital Twin

<br/>

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b+-FF6600?style=for-the-badge&logo=mathworks&logoColor=white)](https://www.mathworks.com/products/matlab.html)
[![Simulink](https://img.shields.io/badge/Simulink-Enabled-0076A8?style=for-the-badge&logo=mathworks&logoColor=white)](https://www.mathworks.com/products/simulink.html)
[![Unreal Engine](https://img.shields.io/badge/Unreal%20Engine-Enabled-black?style=for-the-badge&logo=unrealengine&logoColor=white)](https://www.unrealengine.com/)
[![License](https://img.shields.io/badge/License-MIT-22c55e?style=for-the-badge)](LICENSE)
[![Language](https://img.shields.io/badge/Language-MATLAB%20100%25-FF6600?style=for-the-badge)](https://github.com/FerXxk/Auto-Parking-RL)

<br/>

> *A production-grade digital twin that fuses classical optimal control with modern deep RL — the vehicle searches, decides, and parks itself, entirely autonomously.*

<br/>

---

</div>

## 📌 Table of Contents

- [Overview](#-overview)
- [How it works](#-how-it-works)
- [Architecture](#-architecture)
- [Sensor stack](#-sensor-stack)
- [Tracking Controllers](#-tracking-controllers)
- [Benchmarking & Evaluation](#-benchmarking--evaluation)
- [DRL agent](#-deep-reinforcement-learning-agent)
- [Reward design](#-reward-function-design)
- [Supported algorithms](#-supported-rl-algorithms)
- [Environment & parking lot](#-environment--parking-lot)
- [Quick start](#-quick-start)
- [Training from scratch](#-training-from-scratch)
- [Results](#-results)
- [File structure](#-file-structure)
- [Requirements](#-requirements)
- [References](#-references)

---

## 🎯 Overview

This project implements a **hybrid autonomous parking system** capable of navigating a photorealistic parking lot, finding a free spot, and executing a precise parking maneuver — all without human intervention.

The core innovation is a **seamless handoff** between two complementary control paradigms:

| Phase | Controller | Task |
|-------|-----------|------|
| 🔍 **Search** / Track | **Tracking Controllers**<br/>*(Adaptive MPC, Nonlinear MPC, LQR, Pure Pursuit, Stanley)* | Follow reference path at constant speed, scan for free spots |
| 🅿️ **Park** | **DRL Agent**<br/>*(DDPG / TD3 / SAC)* | Execute collision-free parking maneuver using Lidar feedback |

The entire system runs as a **digital twin** inside Unreal Engine®, co-simulated with MATLAB/Simulink — providing photorealistic sensor data and vehicle dynamics with zero real-world risk.

<br/>

<div align="center">
<img src="https://www.mathworks.com/help/examples/rl/win64/AutomaticParkingValetWithUnrealEngineSimulationExample_02.png" width="80%" alt="2D Parking Lot Visualization — free spots (green) and occupied spots (red)"/>
<br/><sub>2D parking lot map — green = free, red = occupied. The dashed pink line is the MPC reference path.</sub>
</div>

<br/>

---

## ⚙️ How It Works

The system operates in two distinct phases, triggered by a central **mode switch** signal (`isParking`):

```
┌────────────────────────────────────────────────────────────────────────┐
│                          HYBRID CONTROLLER                             │
│                                                                        │
│  ┌──────────────────────┐  spot found?     ┌──────────────────────┐    │
│  │ Tracking Controller  │  ──────────────► │      DRL Agent       │    │
│  │     (search mode)    │                  │   (parking mode)     │    │
│  │  (MPC/LQR/Pursuit)   │                  │  DDPG / TD3 / SAC    │    │
│  └──────────────────────┘                  └──────────────────────┘    │
│            ▲                                       ▲                   │
│            │         Vehicle Mode Selector         │                   │
│            └──────────── isParking ────────────────┘                   │
└────────────────────────────────────────────────────────────────────────┘
```

1. **Initialization** — the vehicle spawns at the southeast corner of the lot; the global path planner generates the reference trajectory.
2. **Search phase** — the selected Tracking Controller (Adaptive MPC by default, or choice of LQR, Nonlinear MPC, Pure Pursuit, or Stanley) tracks the reference path while the virtual camera and Lidar modules scan each parking row.
3. **Mode switch** — the `Vehicle Mode Selector` detects an empty spot and flips `isParking = true`, activating the RL agent subsystem and freezing the tracking controller.
4. **Parking phase** — the DRL agent reads the 3D Lidar point cloud and the relative pose error `[Δx, Δy, Δθ]`, then outputs a continuous steering angle and velocity command to slot the vehicle in.
5. **Done** — the episode ends when the vehicle is within tolerance of the target pose or a collision/timeout is detected.

---

## 🏗️ Architecture

```
rlAutoParkingValet3D.slx
│
├── Vehicle Dynamics                  ← Bicycle kinematic model (Ts = 0.1 s)
│
├── MPC Tracking Controller           ← Adaptive MPC (search mode)
│   ├── Lateral error minimisation
│   └── Speed regulation
│
├── RL Controller                     ← DRL agent (parking mode)
│   ├── Actor network  (continuous actions: δ, v)
│   └── Critic network (Q-value estimation)
│
├── Vehicle Mode Selector             ← isParking signal logic
│
└── Unreal Engine Visualization &     ← 3D render + sensor simulation
    Sensing Subsystem
    ├── Simulation 3D Lidar (360° FOV, 40° vertical)
    └── Camera occupancy algorithm
```

The `RL_Parking_And_Control.m` script is the **single entry point** — it wires together the environment, agents, and training options before opening the Simulink model.

---

## 📡 Sensor Stack

<div align="center">
<img src="https://www.mathworks.com/help/examples/rl/win64/AutomaticParkingValetWithUnrealEngineSimulationExample_01.png" width="75%" alt="Unreal Engine Lidar point cloud — vehicle navigating parking lot"/>
<br/><sub>Unreal Engine® renders a photorealistic environment; the roof-mounted Lidar generates a dense 3D point cloud in real time.</sub>
</div>

<br/>

| Sensor | Spec | Role |
|--------|------|------|
| **3D Lidar** | 360° H-FOV · 40° V-FOV · roof-mounted | Obstacle detection + point cloud observations for RL |
| **Virtual camera** | Parking-row scan algorithm | Spot occupancy detection (green/red indicator) |
| **Ego pose** | Ground-truth from Simulink | Relative pose error `[Δx, Δy, Δθ]` fed to both controllers |

---

## 🧭 Tracking Controllers

The project features multiple path tracking controllers to guide the vehicle along the global reference trajectory during the **Search Phase**. These controllers can be swapped and compared within the Simulink model.

### 1. Adaptive MPC
The default tracking controller. It solves a **constrained finite-horizon optimisation** at every timestep `Ts = 0.1 s`.
* **State vector:** $x = [X, Y, \theta, v]^T$ (position, heading, speed)
* **Control inputs:** $u = [\delta, a]^T$ (steering angle, acceleration)
* **Adaptation:** The prediction model updates its linearisation at each step based on the current velocity, avoiding full nonlinear MPC overhead while preserving accuracy.
* **Constraints:** Steering angle $|\delta| \le 0.5\text{ rad}$, speed $0 \le v \le 5\text{ m/s}$, steering rate $|\Delta\delta| \le 0.1\text{ rad/step}$.

### 2. Nonlinear MPC (NLMPC)
Implemented in [createNonLinearMPCForParking3D.m](AutomaticParkingValetWithUnrealEngineSimulationExample/createNonLinearMPCForParking3D.m).
* Uses a continuous-time nonlinear kinematic bicycle model (`vehicleStateFcn.m`).
* Tracks the reference path by directly solving the nonlinear optimization problem without local linearization.
* **States:** $x = [X, Y, \theta]^T$, **Inputs:** $u = [v, \delta]^T$.
* Enforces equivalent control weights and physical constraints to the Adaptive MPC, optimized via a custom solver setting (maximum 50 iterations per step).

### 3. LQR (Linear Quadratic Regulator)
Implemented in [createLQRForParking3D.m](AutomaticParkingValetWithUnrealEngineSimulationExample/createLQRForParking3D.m).
* A discrete-time infinite-horizon LQR designed around a neutral operating point (heading $\theta = 0$, looking straight ahead).
* State-space model matrices ($A_d, B_d$) are generated via `vehicleStateJacobianFcnDT.m` with nominal velocity $v_{\text{ref}} = 2.5\text{ m/s}$.
* Weighted tracking priority: uses state weights $Q = \text{diag}([1, 15, 5])$ to penalize lateral tracking error heavily, ensuring robust path alignment.

### 4. Pure Pursuit with Adaptive Lookahead
Implemented in [pure_pursuit_control.m](AutomaticParkingValetWithUnrealEngineSimulationExample/pure_pursuit_control.m).
* Projects a dynamic lookahead distance ($L_d$) along the path to calculate the target steering command.
* **Adaptive Lookahead:** In straight lines, lookahead extends up to $14.0\text{ m}$ to prevent steering oscillations; in curves, lookahead shrinks down to $1.5\text{ m}$ (with a minimum floor of $3.0\text{ m}$).
* Uses a nonlinear transition (`sqrt(curvatura_norm)`) to aggressively reduce the lookahead distance as soon as curvature is detected ahead, providing proactive steering response.

---

## 📊 Benchmarking & Evaluation

The profiling and evaluation framework in [analisis.m](AutomaticParkingValetWithUnrealEngineSimulationExample/analisis.m) facilitates comparison of the different controllers (Pure Pursuit, Stanley, LQR, Adaptive MPC, Nonlinear MPC).

### Key Performance Indicators (KPIs)
* **Tracking Precision:** Root Mean Square Error (RMSE) and Maximum/Average error for both lateral displacement and heading/orientation.
* **Control Smoothness:** Energy cost of the steering action ($\int \delta^2 \, dt$) and command smoothness/zigzagging ($\int (\dot{\delta})^2 \, dt$).
* **Computational Cost:** Profiling of block execution times via the Simulink Profiler, normalized as *cost-per-step* (in microseconds) for a hardware-agnostic comparison.

### Workflow
1. Select the desired tracking controller in Simulink.
2. Run the simulation through the script to collect log variables (`egoPose`, `reference`, `accel_steer`, `isParking`).
3. View and record the execution time from the Simulink Profiler report.
4. Store results in `resultados_controladores.mat` and call `compararTodos()` to plot overlay trajectories, comparative bar charts, and error distributions.

---

## 🧠 Deep Reinforcement Learning Agent

Once a free spot is found, the DRL agent takes full authority over the vehicle.

### Observation space

| Signal | Dimension | Description |
|--------|-----------|-------------|
| Lidar point cloud (processed) | 1 × N | Distances to nearby obstacles |
| Relative pose error | 1 × 3 | `[Δx, Δy, Δθ]` to target spot |

### Action space

| Action | Range | Description |
|--------|-------|-------------|
| Steering angle δ | `[-0.5, 0.5]` rad | Continuous, normalised |
| Longitudinal velocity v | `[-2, 2]` m/s | Allows forward and reverse |

### Network architecture

```
Actor:  observations → FC(256) → ReLU → FC(128) → ReLU → tanh → actions
Critic: [obs, act]   → FC(256) → ReLU → FC(128) → ReLU →       → Q-value
```

---

## 🏆 Reward Function Design

The shaped reward guides the agent towards clean, efficient parking:

```matlab
% Proximity reward (dense guidance)
r_pose  = -w1 * norm([Δx, Δy]) - w2 * abs(Δθ);

% Success bonus
r_done  = +100  (if within tolerance: Δpos < 0.3 m, Δθ < 0.1 rad);

% Collision penalty
r_coll  = -50   (if Lidar detects obstacle within safety radius);

% Time penalty (encourages speed)
r_time  = -0.1  (per timestep);

% Total
R = r_pose + r_done + r_coll + r_time;
```

---

## 🤖 Supported RL Algorithms

The project supports three off-policy actor-critic algorithms. Switch between them via the `agentType` variable in `RL_Parking_And_Control.m`:

| Algorithm | Key strength | Best for |
|-----------|-------------|----------|
| **DDPG** | Simple, fast convergence | Baseline, quick experiments |
| **TD3** | Overestimation fix, more stable | Default recommended |
| **SAC** | Entropy regularisation, robust exploration | Challenging spots, narrow spaces |

```matlab
agentType = "TD3";   % options: "DDPG" | "TD3" | "SAC"
```

---

## 🅿️ Environment & Parking Lot

The simulation uses a subsection of the MathWorks **Large Parking Lot** scene rendered in Unreal Engine®:

- **20+ numbered parking spots** — each with a green/red occupancy indicator
- **Static obstacle vehicles** fill all occupied spots as rigid bodies
- **Reference path** starts at the southeast corner, sweeps the lot west-to-east

```matlab
% Set which spot is free (all others filled with static vehicles)
freeSpotIndex = 18;
setupActorVehicles("rlAutoParkingValet3D", freeSpotIndex);
```

You can test any of the 20+ spots by changing `freeSpotIndex` and updating `egoTargetPose` accordingly.

---

## 🚀 Quick Start

### 1 — Clone

```bash
git clone https://github.com/FerXxk/Auto-Parking-RL.git
cd Auto-Parking-RL
```

### 2 — Open in MATLAB

Set the repo root as your MATLAB working directory, then open the entry-point script:

```matlab
open('RL_Parking_And_Control.m')
```

### 3 — Run a pre-trained simulation

```matlab
% Use pre-trained weights — no training required
doTraining = false;
```

Click **Run** in Simulink (`rlAutoParkingValet3D.slx`). The Unreal Engine 3D viewport will open and the vehicle will search, switch modes, and park autonomously.

### 4 — Try different spots

```matlab
% List the parking spots you want to simulate (valid indices: 1 to 23)
spotsToSimulate = [20, 6, 17, 1, 12]; 
```

### 5 — Run tracking controller benchmarking & comparison

The project supports benchmarking multiple path tracking controllers. To run the analysis and compare their performance:

1. Open the Simulink model `rlAutoParkingValet3D.slx`.
2. Inside the Simulink model, manually select the controller block you want to evaluate (e.g. **Pure Pursuit**, **Stanley**, **Adaptive MPC**, **Nonlinear MPC**, or **LQR**).
3. Open the evaluation script [analisis.m](AutomaticParkingValetWithUnrealEngineSimulationExample/analisis.m) in MATLAB.
4. Set the name of the active controller in the script under **PASO 2**:
   ```matlab
   nombreCtrl = 'Stanley'; % Change to matches the active controller: 'LQR', 'Pure Pursuit', etc.
   ```
5. Run the script step-by-step using **Run Section** (`Ctrl + Enter`) or run it in parts:
   * **PASO 1**: Simulates the model with the Simulink Profiler activated to collect the vehicle trajectory, control commands, and computation times.
   * **PASO 2**: Calculates precision metrics (RMSE, max/mean lateral and heading error) and control effort/smoothness. If you want to log computational cost, read the execution time from the generated Profiler report and set it in `total_time_profiler` before executing this section.
   * **PASO 3**: Saves the computed metrics to `resultados_controladores.mat`.
   * **PASO 4 & 5**: Executing these sections displays a comparative table in the command window and generates plots overlaying trajectories, comparing tracking precision (RMSE/Max/Mean), and comparing heading error side-by-side.

---

## 🏋️ Training From Scratch

To train a new agent from random initialisation:

```matlab
doTraining = true;
agentType   = "TD3";      % "DDPG" | "TD3" | "SAC"
```

Training options are configured inside `RL_Parking_And_Control.m` via `rlTrainingOptions`. Expect **several hours** on a modern GPU with Parallel Computing Toolbox enabled.

**Monitor training progress** in the Episode Manager window — watch cumulative reward climb and episode length shrink as the agent learns to park efficiently.


<div align="center">
  <img src="images/FinalEntrenamientoSAC.png" width="80%" alt="SAC training curve — cumulative reward per episode"/>
  <br/><sub>SAC training curve — cumulative reward per episode</sub>
</div>

---

## 📊 Results

<div align="center"  style="margin-bottom: 40px;">
  <img src="images/SAC_metricas.png" width="90%" alt="Results"/>
  <br/><sub>Results from 100 episodes simulation with SAC agent</sub>
</div>



<div align="center"   style="margin-bottom: 40px;">
  <img src="images/TD3_metricas.png" width="90%" alt="Results"/>
  <br/><sub>Results from 100 episodes simulation with TD3 agent</sub>
</div>



<div align="center">
  <img src="images/DDPG_metricas.png" width="90%" alt="Results"/>
  <br/><sub>Results from 100 episodes simulation with DDPG agent</sub>
</div>

---

## 📁 File Structure

```
Auto-Parking-RL/
│
├── RL_Parking_And_Control.m              ← Main entry point script
├── rlAutoParkingValet3D.slx              ← Core Simulink model
│
├── AutomaticParkingValetWithUnreal
│   EngineSimulationExample/
│   ├── ParkingLotManager.m               ← Environment state manager
│   ├── ParkingLotVisualizer.m            ← 2D visualisation utility
│   ├── setupActorVehicles.m              ← Populate lot with static vehicles
│   ├── createReferenceTrajectory.m       ← Global path generator
│   ├── createLQRForParking3D.m           ← LQR controller design
│   ├── createNonLinearMPCForParking3D.m  ← Nonlinear MPC controller design
│   ├── pure_pursuit_control.m            ← Pure Pursuit with adaptive lookahead
│   ├── analisis.m                        ← Benchmarking & evaluation framework
│   └── *.m / *.mat                       ← Agent networks, helper functions
│
├── images/                               ← Figures and screenshots
├── videos/                               ← Simulation recordings
└── .gitignore
```

---

## 🛠️ Requirements

### MATLAB Toolboxes

| Toolbox | Version |
|---------|---------|
| MATLAB | R2023b or newer |
| Simulink | included with MATLAB |
| Reinforcement Learning Toolbox | required |
| Model Predictive Control Toolbox | required |
| Automated Driving Toolbox | required |
| Robotics System Toolbox | required |
| Parallel Computing Toolbox | recommended (training) |

### System

- **OS:** Windows 10/11 (Unreal Engine co-simulation requires Windows)
- **RAM:** 16 GB minimum, 32 GB recommended
- **GPU:** CUDA-capable GPU recommended for training
- **Unreal Engine:** Installed via MathWorks Automated Driving Toolbox support package

---

## 📚 References

- MathWorks. *Automatic Parking Valet with Unreal Engine Simulation* — [docs.mathworks.com](https://www.mathworks.com/help/reinforcement-learning/ug/automatic-parking-valet-with-mpc-and-unreal-engine.html)
- Fujimoto, S. et al. *Addressing Function Approximation Error in Actor-Critic Methods (TD3)*, ICML 2018
- Haarnoja, T. et al. *Soft Actor-Critic: Off-Policy Maximum Entropy Deep Reinforcement Learning*, ICML 2018
- Lillicrap, T. et al. *Continuous Control with Deep Reinforcement Learning (DDPG)*, ICLR 2016
- Zhang, Z. et al. *Automated Parking Trajectory Generation Using Deep Reinforcement Learning (SAC)*, arXiv 2025

---

<div align="center">

<br/>

Made by  · [Fernando Román](https://github.com/FerXxk) ·  [Jose Antonio García](https://github.com/FerXxk) · [Andrés Martínez](https://github.com/FerXxk)

<br/>

[![Star this repo](https://img.shields.io/github/stars/FerXxk/Auto-Parking-RL?style=social)](https://github.com/FerXxk/Auto-Parking-RL)

</div>