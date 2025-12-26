---
title: "Authoring Guide (Chapters, Labs, Quizzes)"
---

# Authoring Guide (Chapters, Labs, Quizzes)

## Goals

- Keep every page runnable and verifiable.
- Keep interfaces stable (topic/action names, message types) as the course grows.
- Keep structure consistent across chapters to support learning and grading.

## Required sections (chapters)

Each chapter MUST include:

- Learning objectives
- Key terms
- Lab deliverable
- Assessment item (quiz/checkpoint)

Use the templates under `docs/templates/` when creating new pages.

## Link conventions

- Prefer relative links to other docs (e.g., `/labs/module-1-ros2-lab`).
- Reference images via `/img/<file>` (files live under `static/img/`).

## Mermaid conventions

- Keep diagrams small and readable.
- Prefer `flowchart` diagrams for pipelines and ROS graphs.
- Avoid embedding sensitive details (hostnames, IPs, API keys).

