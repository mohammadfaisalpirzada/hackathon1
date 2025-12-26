---
title: "On-Prem Lab BOM (Example)"
---

# On-Prem Lab BOM (Example)

This is an example bill of materials for a 10-seat cohort lab. Adjust quantities and pricing for your region and supplier.

## Workstation tiers (per seat)

| Tier | Target use | CPU | RAM | GPU | Storage | Target range (USD) |
|------|------------|-----|-----|-----|---------|--------------------|
| Minimum | ROS + RViz + basic Gazebo | 4 cores | 16 GB | integrated | 30–100 GB free | $700–$1,200 |
| Recommended | Smooth Gazebo + optional Isaac | 8+ cores | 32–64 GB | RTX-class 12–16 GB VRAM | 1 TB SSD | $1,500–$3,500 |
| Pro | Heavy Isaac + large scenes | 12+ cores | 64–128 GB | 24 GB+ VRAM | 2 TB SSD | $3,500–$6,000 |

## Shared lab infrastructure (10 seats)

| Item | Qty | Target range (USD) | Notes |
|------|-----|--------------------|-------|
| Gigabit (or faster) switch | 1 | $200–$1,500 | Size depends on seats + uplink |
| NAS / shared storage | 1 | $800–$3,000 | Datasets, cache, student submissions |
| UPS (optional) | 1 | $150–$800 | Protect NAS/switch |
| Cables/adapters | 10+ | $100–$400 | Ethernet, HDMI/DP, power |

## Notes

- Isaac Sim performance is typically gated by VRAM and driver compatibility.
- For students without GPUs, prefer the Ether lab architecture instead of forcing underpowered local installs.

