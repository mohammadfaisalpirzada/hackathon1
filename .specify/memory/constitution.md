# Project Constitution — Physical AI & Humanoid Robotics Book (Docusaurus)

## Purpose
Build a Docusaurus book site for the course **“Physical AI & Humanoid Robotics (Quarter 2)”**.
The book must be practical, runnable, and suitable for teaching: clear explanations + step-by-step labs + verification.

## Audience
Intermediate developers who know basic Python/ML but are new to ROS 2, simulation, and robotics systems engineering.

## Output format
- Docusaurus **Markdown/MDX only**.
- Clear headings, short paragraphs, bullets.
- Use Mermaid diagrams when they clarify pipelines (ROS graph, sim-to-real loop, VLA flow).
- Avoid fluff. Every section must help the learner build or debug something.

## Non-negotiable rules (MUST)

### 1) Fixed chapter structure
Every chapter MUST include:
- Prerequisites
- Concepts (the “why” and key mental models)
- Hands-on lab (step-by-step)
- Verification (how the student confirms it worked)
- Troubleshooting (common failures + fixes)
- Quiz

### 2) Quiz policy
- Each chapter MUST include a quiz with **minimum 5 questions**; **recommended 8–10**.
- Appendix/reference-only pages MAY use **3–5** questions.
- Questions must test understanding of both concepts and practical steps (commands, config, ROS graph, etc.).

### 3) Runnable labs (default environment)
- Labs and commands MUST be runnable on **Ubuntu 22.04 + ROS 2 Humble** unless clearly labeled “Optional”.
- Every lab MUST state:
  - expected deliverable (file/output),
  - exact run commands,
  - exact verification steps,
  - expected result (what success looks like).

### 4) Prefer ROS 2 Python (rclpy)
- Prefer `rclpy` examples and labs.
- If C++ is shown, provide a Python equivalent or clearly justify why C++ is required.

### 5) Simulation and hardware clarity
- When listing requirements, ALWAYS separate:
  - **Minimum** (works but slower/limited),
  - **Recommended** (smooth experience),
  - **Will not work** constraints (hard blockers).
- Any GPU/RTX requirement must be stated plainly (especially for Isaac Sim).

## Required scope (MUST cover)
The content MUST cover, at minimum:

1. **Physical AI / Embodied Intelligence**
   - Why physical laws, sensors, and control loops matter
   - Sensor overview: cameras, depth, LiDAR, IMU, force/torque

2. **Module 1 — ROS 2**
   - Nodes, topics, services, actions
   - `rclpy` packages, launch files, parameters
   - Bridging Python “agents” to ROS controllers
   - URDF basics for humanoids (robot description + joints/links)

3. **Module 2 — Digital Twin**
   - Gazebo setup, physics, collisions, gravity
   - Sensor simulation (depth camera, LiDAR, IMU)
   - Unity content is **optional/advanced** and must be clearly labeled if included

4. **Module 3 — NVIDIA Isaac**
   - Isaac Sim (photoreal sim, synthetic data)
   - Isaac ROS (hardware-accelerated VSLAM where applicable)
   - Nav2 navigation basics (path planning, obstacles)

5. **Module 4 — Vision-Language-Action (VLA)**
   - Whisper voice input → text
   - LLM planning: natural language → sequence of ROS 2 actions
   - Safety and reliability: confirmations, fallbacks, timeouts

6. **Capstone (minimum in simulation)**
   Capstone MUST implement this end-to-end pipeline:
   **voice command (Whisper) → LLM planning → ROS 2 actions → Nav2 navigation + obstacle avoidance → object identification (vision) → manipulation/pick-place**.

## Repo and deployment constraints
- Docusaurus v3 site structure with a clean sidebar-based ToC.
- GitHub Pages deployment via GitHub Actions MUST work on push to `main`.
- Site must build without broken internal links.

## Assumptions policy
If any detail is unknown (GitHub username/org, repo name, Pages baseUrl):
- state the assumption explicitly,
- choose a safe default,
- keep the values easy to change in config (avoid scattering hardcoded URLs).

## Governance
- This constitution is the highest priority rule-set for the project.
- If spec/plan/tasks conflict with this document, update spec/plan/tasks or revise this constitution intentionally.
- Any scope change requires updating the Table of Contents and the affected labs/quizzes.

**Version**: 1.1.0 | **Ratified**: 2025-12-25 | **Last Amended**: 2025-12-26
