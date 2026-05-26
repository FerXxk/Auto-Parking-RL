# Autonomous Parking Valet using Hybrid Control (Adaptive MPC + DRL)

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b%2B-orange.svg)](https://www.mathworks.com/products/matlab.html)
[![Simulink](https://img.shields.io/badge/Simulink-Enabled-blue.svg)](https://www.mathworks.com/products/simulink.html)
[![Unreal Engine](https://img.shields.io/badge/Unreal%20Engine-Enabled-black.svg)](https://www.unrealengine.com/)

---

## 🎯 Purpose of the Project

The core objective of this project is to design, evaluate, and simulate an advanced hybrid control system capable of navigating an autonomous vehicle through a complex parking lot environment, searching for an empty spot, and executing precise parking maneuvers. 

To achieve this, the architecture splits the operational responsibilities into two distinct methodologies:
* **Path Following & Searching:** An **Adaptive Model Predictive Controller (MPC)** keeps the vehicle at a steady, controlled speed along a global reference path while its virtual sensor modules scan the surroundings for available parking spots.
* **Collision-Free Parking:** Once a free spot is identified, control seamlessly switches to a **Deep Reinforcement Learning (DRL)** agent (supporting **DDPG**, **TD3**, or **SAC** frameworks). The agent processes real-time 3D Lidar point cloud feedback and relative pose errors to safely guide the vehicle into tight spaces, avoiding static vehicles and boundaries.

The entire environment, vehicle kinematics, sensor perception, and multi-agent control logic are simulated as a digital twin leveraging the real-time 3D rendering power of **Unreal Engine®**.

---

## 🚀 How to Use It

### 1. Clone the Repository
Open your terminal and clone the repository to your local machine:
```bash
git clone [https://github.com/FerXxk/Auto-Parking-RL.git](https://github.com/FerXxk/Auto-Parking-RL.git)
cd Auto-Parking-RL

2. Run a Pre-trained Simulation
To see the hybrid controller in action without waiting for a long training cycle to complete, follow these steps inside MATLAB:

Launch MATLAB and set the repository root directory as your current folder.

Open the primary entry-point script (RL_Parking_And_Control.m) or the core Simulink model (rlAutoParkingValet3D.slx).

In the MATLAB Workspace, ensure the training flag is set to false so the model uses the pre-trained weights:

Matlab
doTraining = false;
Set your target custom pose or index for the specific spot you want to test (e.g., Target Spot 17):

Matlab
egoTargetPose = [2.8875, -38.5600, -1.5708];
Click the Run button in Simulink. The Unreal Engine 3D viewport window will open, showcasing the vehicle tracking the path, switching modes, and parking autonomously.