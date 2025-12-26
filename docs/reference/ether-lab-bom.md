---
title: "Ether Lab BOM / Cost Model (Example)"
---

# Ether Lab BOM / Cost Model (Example)

This is an example cost model for running GPU workloads remotely so students without GPUs can still complete Isaac-heavy labs.

## Components

| Component | Why it exists | What to budget |
|----------|---------------|----------------|
| GPU compute | Isaac Sim + accelerated perception | hourly/monthly GPU cost |
| Persistent storage | caches, assets, datasets, bags | GB-month storage |
| Egress/bandwidth | streaming UI + downloads | GB egress (region-dependent) |
| Remote access | RDP/SSH/VNC | operational time + tooling |

## Instance tiers (generic)

| Tier | Example GPU class | Target use | Budget guidance (USD per student-month) |
|------|-------------------|------------|----------------------------------------|
| Entry GPU | ~16 GB VRAM | light Isaac labs, small scenes | $150–$300 |
| Mid/High GPU | 24–48 GB VRAM | larger scenes, heavier perception | $300–$600 |

## Cost control levers

- enforce scheduled hours (lab windows)
- use pre-baked images to reduce setup time
- keep assets on persistent storage to reduce re-download and cache rebuilds
- record deliverables as small artifacts (logs/bags) rather than full VM images

