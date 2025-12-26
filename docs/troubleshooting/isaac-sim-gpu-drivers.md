---
title: "Troubleshooting: Isaac Sim GPU/Driver Issues"
---

# Troubleshooting: Isaac Sim GPU/Driver Issues

Isaac Sim is sensitive to GPU drivers and graphics stack versions. When it fails, it often fails early and noisily.

## Symptom: Isaac Sim won’t start / crashes on launch

Common causes:

- driver version mismatch for the Isaac Sim release
- insufficient VRAM (8 GB minimum; 12–16 GB recommended)
- running via VM/remote desktop without proper GPU passthrough

Mitigation checklist:

- verify GPU and driver are detected
- verify available VRAM and system RAM
- use the official Isaac Sim “supported driver” guidance for the chosen release

## Symptom: containerized Isaac ROS can’t see the GPU

Common causes:

- NVIDIA container toolkit not installed/configured
- driver/toolkit mismatch

Diagnostics (high-level):

- confirm the container runtime can enumerate GPUs
- confirm the same container can run a minimal GPU check

## Ether lab considerations (cloud GPU)

When using a remote GPU instance:

- keep large assets/datasets on attached persistent storage
- budget time for first-run shader/cache compilation
- treat latency as a constraint (especially for interactive sim)

