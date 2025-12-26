---
title: "Hardware + Lab Architectures (On-Prem vs Ether Lab)"
---

# Hardware + Lab Architectures (On-Prem vs Ether Lab)

This page defines two supported ways to run the course:

- **On-prem**: students use local workstations (recommended for best sim performance)
- **Ether lab**: students use remote GPU instances for Isaac-heavy content

## Decision guide

| Constraint | Prefer On-Prem | Prefer Ether Lab |
|-----------|-----------------|------------------|
| Students lack NVIDIA GPUs | ✗ | ✓ |
| Low-latency interactive sim | ✓ | ✗ |
| Cohort scale fluctuates | ✗ | ✓ |
| IT can manage images/hardware | ✓ | ✗ |

## Architecture (high level)

- **On-prem**: workstation runs ROS + sim + tooling locally.
- **Ether lab**: remote instance runs GPU-heavy workloads; student connects via remote desktop/SSH.

![Ether lab topology](/img/ether-lab-topology.svg)

## BOM summary (typical ranges, USD)

| Item | Quantity (example) | Target range (USD) | Notes |
|------|---------------------|--------------------|-------|
| Student workstation (recommended) | 1 per student | $1,500–$3,500 | Gazebo core path + optional Isaac |
| On-prem cohort lab (10 seats) | 10 | $15,000–$35,000 | Workstations only; excludes room/ops |
| Shared storage/networking (10 seats) | 1 | $2,000–$6,000 | NAS + switch/cabling |
| Ether lab (cloud GPU) | per active student-month | $150–$600 | Depends on utilization caps |

## Detailed BOM pages

- On-prem BOM: `docs/reference/on-prem-lab-bom.md`
- Ether lab BOM/cost model: `docs/reference/ether-lab-bom.md`
