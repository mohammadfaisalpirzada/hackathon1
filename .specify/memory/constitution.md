# Project Constitution — Physical AI & Humanoid Robotics Book (Docusaurus)

## Core principles

### 1) No fluff, only useful
We write for builders. Every section must explain “why” and “how”. Avoid marketing language.

### 2) Runnable by default (non-negotiable)
All labs and code must be runnable on Ubuntu 22.04 + ROS 2 Humble, unless explicitly labeled optional. Commands must be copy-paste friendly and include expected outputs or verification steps.

### 3) Chapter structure is fixed
Every chapter must include:
- Prerequisites
- Concepts
- Hands-on lab (step-by-step)
- Troubleshooting
- Short quiz (5–10 questions)

### 4) Prefer ROS 2 Python (rclpy)
Use `rclpy` first. If C++ is shown, provide a Python equivalent or justify why C++ is required.

### 5) Explicit hardware guidance
Whenever hardware is mentioned, clearly separate:
- Minimum (works, slower/limited)
- Recommended (smooth)
- “Will not work” constraints (e.g., Isaac Sim needs RTX)

## Scope (must cover)
- Physical AI / embodied intelligence + sensors
- ROS 2: nodes, topics, services, actions, launch, parameters
- URDF basics for humanoids
- Simulation: Gazebo (required) and Unity (optional/advanced)
- NVIDIA Isaac Sim + Isaac ROS: synthetic data, VSLAM, navigation
- Nav2 basics
- Vision-Language-Action: Whisper voice → LLM planning → ROS actions
- Capstone: voice command → plan → navigate → identify object → manipulate

## Repo + deployment constraints
- Docusaurus v3 site.
- Deploy to GitHub Pages with GitHub Actions.
- No broken internal links in build.
- Keep assumptions configurable (url/baseUrl) and clearly documented.

## Governance
- This constitution is the top rule. If any later doc conflicts, update that doc.
- Any change to scope/structure requires updating the spec + plan + tasks accordingly.

**Version**: 1.0.0 | **Ratified**: 2025-12-25 | **Last Amended**: 2025-12-25
