# Reinforcement Learning-Based Quadcopter Stabilization Using CoppeliaSim

**Final Project** for Advanced Topics in Robotics (038785), Technion

**Authors:** Nir Manor & Gil Gur Arieh
**Date:** May 2024

---

## Project Overview

This project develops a **reinforcement learning (RL) model** to autonomously control a quadcopter to reach a goal position and stabilize, **without using explicit control models or prior knowledge of drone physics**.

### Key Features:
- **Learning-based control:** PPO and A2C agents
- **Realistic physics simulation:** Full quadcopter dynamics in CoppeliaSim
- **Custom RL environment:** 16D state, 4D action space
- **Synchronization framework:** Robust Python-CoppeliaSim bridge
- **Production-grade code:** Modular, documented implementation

---

## Solution Architecture

### System Components

#### 1. CoppeliaSim Physics Simulator

\![Drone Scene](presentation/scene.png)

#### 2. Gymnasium Custom Environment

16D state space (orientation, position, velocities) and 4D action space (propeller thrusts)

#### 3. RL Training Framework

PPO and A2C from Stable-Baselines3 with [128, 128] network architecture

#### 4. Python-CoppeliaSim Synchronization

Robust synchronization with blocking calls and paused communication

---

## Results & Performance

### Training Progress

\![Training Visualization](presentation/drone.gif)

### Architecture Diagram

\![Code Structure](presentation/code_structure.png)

### Quadcopter Physics

\![Drone Equations](presentation/drone_eqs.png)

### Training Metrics

| Metric | Value |
|--------|-------|
| Total timesteps | 350,000+ |
| Training time per 100k | ~9 hours |
| Mean reward | +15 to +25 |
| Algorithm | PPO (300k+) |

---

## Usage

### Prerequisites
```bash
conda env create -f environment.yml
```

### Setup CoppeliaSim
1. Install from https://coppeliarobotics.com/
2. Load drone_simplified_model.ttt in CoppeliaSim
3. Enable Remote API server

### Training
```python
from drone.DroneEnv import DroneEnv
from stable_baselines3 import PPO

env = DroneEnv(reward_type="mixed")
agent = PPO("MlpPolicy", env, gamma=0.95)
agent.learn(total_timesteps=100000)
agent.save("my_ppo_agent")
env.close()
```

---

## Authors

**Nir Manor** — Machine Learning & Autonomous Systems
- GitHub: https://github.com/NirManor

**Gil Gur Arieh** — Autonomous Systems & Robotics
- GitHub: https://github.com/gilgurarieh

---

**Last Updated:** May 2024
