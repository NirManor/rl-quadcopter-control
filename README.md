# Reinforcement Learning-Based Quadcopter Stabilization Using CoppeliaSim

**Final Project** for Advanced Topics in Robotics (038785), Technion

**Authors:** Nir Manor & Gil Gur Arieh
**Date:** May 2024

---

## Project Overview

This project develops a **reinforcement learning (RL) model** to autonomously control a quadcopter to reach a goal position and stabilize, **without using explicit control models or prior knowledge of drone physics**. The system integrates CoppeliaSim physics simulation with Gymnasium RL environment framework and Stable-Baselines3 algorithms.

### Key Features:
- **Learning-based control:** PPO and A2C agents without pre-programmed physics models
- **Realistic physics simulation:** Full quadcopter dynamics in CoppeliaSim
- **Custom RL environment:** Gymnasium-compatible with sophisticated reward shaping
- **Synchronization framework:** Python-CoppeliaSim bridge with robust timing control
- **Production-grade code:** Modular, documented implementation

## State Space & Action Space

**16D Observation:**
- Drone Orientation: Quaternion [q_w, q_x, q_y, q_z]
- Drone Position: [x, y, z]
- Linear Velocity: [v_x, v_y, v_z]
- Angular Velocity: [α, β, γ]
- Target Position: [x_t, y_t, z_t]

**4D Actions:** Propeller thrust [T₁, T₂, T₃, T₄] ∈ [0, 3.18825] N

## Multi-Objective Reward Function

```
r(s) = c₀ - c₁(|α| + |β|) - c₂|z_drone - z_target| - c₃√[(x_drone-x_target)² + (y_drone-y_target)²]
```

- **c₀ = 50:** Positive baseline (reward non-terminal steps)
- **c₁ = 5:** Pitch/roll penalty (encourage upright orientation)
- **c₂ = 10:** Vertical distance weight
- **c₃ = 5:** Horizontal distance weight

## Results & Training History

| Metric | Value |
|--------|-------|
| Total timesteps | 350,000+ |
| Training time per 100k steps | ~9 hours |
| Algorithms tested | A2C, PPO |
| Network architecture | [128, 128] neurons |
| Best reward achieved | +15 to +25 |

## Usage

### Setup
```bash
pip install -r requirements.txt
```

### Train a New Agent
```python
from DroneEnv import DroneEnv
from stable_baselines3 import PPO

env = DroneEnv(reward_type="mixed")
agent = PPO("MlpPolicy", env, gamma=0.95)
agent.learn(total_timesteps=100000)
agent.save("my_agent")
```

### Evaluate
```python
env = DroneEnv(reward_type="mixed")
agent = PPO.load("models/ppo_drone_agent", env=env)
obs, _ = env.reset()
for _ in range(1000):
    action, _ = agent.predict(obs, deterministic=True)
    obs, _, terminated, _, _ = env.step(action)
    if terminated:
        break
```

## Documentation

See **Advance_Robotics_Project.pdf** for full technical report including:
- Mathematical formulations
- Synchronization challenges and solutions
- Reward shaping evolution
- Detailed experimental results
- Future work recommendations

## Repository Structure

```
drone/
├── DroneEnv.py              # Gymnasium environment
├── Drone_model.py           # CoppeliaSim interface
├── rl_main.py               # Training script
├── drone_test.py            # Quick test
├── drone_simplified_model.ttt  # CoppeliaSim scene
├── models/                  # Trained agents
├── VREP_RemoteAPIs/         # CoppeliaSim API bindings
├── logs/                    # Training logs
├── requirements.txt         # Dependencies
├── README.md                # This file
└── LICENSE                  # MIT License
```

## Key Insights

1. **Reward design is critical** - Naive goal-only rewards failed; multi-objective weighting needed
2. **Synchronization matters** - Python-CoppeliaSim timing was the primary bottleneck
3. **Curriculum learning helps** - Progressive task complexity improved convergence
4. **Sample efficiency is limited** - RL requires 350k+ timesteps (~87 hours training)

## Future Work

- Parallel environments (10x speedup)
- Domain randomization for robustness
- Sim-to-real transfer
- Alternative algorithms (TD3, SAC)
- Formal safety guarantees

## Citation

```bibtex
@article{Manor2024QuadcopterRL,
  title={Reinforcement Learning-Based Quadcopter Stabilization Using CoppeliaSim},
  author={Manor, Nir and Gur Arieh, Gil},
  journal={Advanced Topics in Robotics (038785), Technion},
  year={2024}
}
```

## Authors

- **Nir Manor** - GitHub: https://github.com/NirManor
- **Gil Gur Arieh** - Technion

## License

MIT License - See LICENSE file

## Acknowledgments

- Dr. Avraham Cohen (Technion)
- Stable-Baselines3 team
- CoppeliaSim/V-REP developers
