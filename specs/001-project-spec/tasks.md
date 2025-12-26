---
description: "Task list for implementing the Docusaurus course book site"
---

# Tasks: Physical AI & Humanoid Robotics (Q2) Course Site

**Input**: Design documents from `C:\ai_cli\hackathon\hackathon1\hackathon1\specs\001-project-spec\`  
**Prerequisites**: `C:\ai_cli\hackathon\hackathon1\hackathon1\specs\001-project-spec\plan.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\specs\001-project-spec\spec.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\specs\001-project-spec\research.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\specs\001-project-spec\data-model.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\specs\001-project-spec\contracts\`

**Tests**: No automated tests required; verification is via local build + deploy smoke checks recorded in files.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Every task includes at least one exact file path it will touch

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Ensure the Docusaurus project is configured predictably for local dev and GitHub Pages, and establish the content scaffolding directories.

- [x] T001 Verify production URL/base path behavior and repo metadata in `C:\ai_cli\hackathon\hackathon1\hackathon1\docusaurus.config.ts` (url/baseUrl/organizationName/projectName)
- [x] T002 Align navbar + footer links to actual routes and ensure “Capstone” points to the correct doc route in `C:\ai_cli\hackathon\hackathon1\hackathon1\docusaurus.config.ts`
- [x] T003 Define stable sidebar categories for `Intro`, `Chapters`, `Labs`, `Troubleshooting`, `Reference` in `C:\ai_cli\hackathon\hackathon1\hackathon1\sidebars.ts`
- [x] T004 [P] Create templates index/landing page in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\templates\README.md`
- [x] T005 [P] Create chapter authoring template with required fields (objectives, key terms, lab deliverable, assessment) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\templates\chapter-template.md`
- [x] T006 [P] Create lab authoring template with required fields (module, prerequisites, steps, verification) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\templates\lab-template.md`
- [x] T007 [P] Create quiz authoring template with required fields (learning objectives mapped to questions) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\templates\quiz-template.md`
- [x] T008 [P] Add a “Build + Deploy Verification Log” page to record command outputs and smoke checks in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\reference\build-deploy-verification.md`

**Checkpoint (Phase 1)**: Local dev can run; docs templates exist; sidebar skeleton is defined (even if not fully populated yet).

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Create the cross-cutting pages and sidebar wiring needed by all stories (without requiring full chapter content yet).

- [x] T009 [P] Create labs index/landing page in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\labs\README.md`
- [x] T010 [P] Create troubleshooting index/landing page in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\troubleshooting\README.md`
- [x] T011 [P] Create reference index/landing page in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\reference\README.md`
- [x] T012 [P] Create glossary page stub and include key course terms in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\reference\glossary.md`
- [x] T013 [P] Create ROS 2 CLI cheatsheet page in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\reference\ros2-cheatsheet.md`
- [x] T014 Update sidebar to include the new `Labs`, `Troubleshooting`, and `Reference` categories (pointing to the landing pages) in `C:\ai_cli\hackathon\hackathon1\hackathon1\sidebars.ts`

**Checkpoint (Phase 2)**: Sidebars render without broken items and the non-chapter navigation sections exist.

---

## Phase 3: User Story 1 - Read the course book on GitHub Pages (Priority: P1) — MVP

**Goal**: A student can browse the site (at minimum: Intro + Setup + a few chapters) via sidebar navigation without 404s.

**Independent Test**: Build locally and confirm sidebar navigation reaches Intro + Setup + at least one chapter; record results in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\reference\build-deploy-verification.md`.

- [x] T015 [P] [US1] Ensure the homepage doc renders as the course landing page and matches the exact course title in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\intro.md`
- [x] T016 [P] [US1] Write student setup guidance (Ubuntu 22.04 + ROS 2 Humble + Gazebo baseline) and confirm it matches tooling requirements in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\course-setup.md`
- [x] T017 [US1] Create a “Start Here” navigation block (links to Setup + first module chapter) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\intro.md`
- [x] T018 [US1] Restrict the `Chapters` sidebar to currently-available chapter pages only (avoid listing pages not yet created) in `C:\ai_cli\hackathon\hackathon1\hackathon1\sidebars.ts`
- [x] T019 [US1] Record local build + navigation smoke-check results in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\reference\build-deploy-verification.md`

---

## Phase 4: User Story 2 - Maintain and publish content (Priority: P2)

**Goal**: A maintainer can merge changes and have the site deploy automatically to GitHub Pages.

**Independent Test**: Workflow runs on push and publishes the site; record the deployed URL and smoke-check results in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\reference\build-deploy-verification.md`.

- [x] T020 [US2] Create GitHub Pages deploy workflow (build + upload artifact + deploy) in `C:\ai_cli\hackathon\hackathon1\hackathon1\.github\workflows\deploy.yml`
- [x] T021 [US2] Ensure workflow passes required environment values for GitHub Pages URL/base path in `C:\ai_cli\hackathon\hackathon1\hackathon1\.github\workflows\deploy.yml`
- [x] T022 [P] [US2] Document deployment assumptions and environment overrides in `C:\ai_cli\hackathon\hackathon1\hackathon1\README.md`
- [x] T023 [US2] Record a “Deploy smoke-check” checklist run (intro + one chapter) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\reference\build-deploy-verification.md`

---

## Phase 5: User Story 3 - Follow runnable ROS 2 labs (Priority: P3)

**Goal**: Students can run at least one complete ROS 2 lab with explicit commands, file paths, and verification steps.

**Independent Test**: A student can follow the module lab and produce the specified artifact(s); the page includes verification commands and expected outputs.

- [x] T024 [P] [US3] Write a dedicated ROS install troubleshooting page (common failures + fixes) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\troubleshooting\ros2-install.md`
- [x] T025 [P] [US3] Write a Gazebo troubleshooting page (blank/slow, plugin issues, sim time, common errors) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\troubleshooting\gazebo-issues.md`
- [x] T026 [US3] Add Troubleshooting sidebar items for ROS + Gazebo pages in `C:\ai_cli\hackathon\hackathon1\hackathon1\sidebars.ts`
- [x] T027 [P] [US3] Create and write the Module 1 ROS 2 lab (single coherent deliverable + verification) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\labs\module-1-ros2-lab.md`
- [x] T028 [P] [US3] Create and write the Module 2 Gazebo lab (spawn + drive + odom verification) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\labs\module-2-gazebo-lab.md`
- [x] T029 [US3] Add Labs sidebar items for Module 1 and Module 2 labs in `C:\ai_cli\hackathon\hackathon1\hackathon1\sidebars.ts`

---

## Phase 6: User Story 4 - ToC + sidebars + chapter templates (Priority: P4)

**Goal**: The ToC clearly communicates Intro + Modules 1–4 + Capstone, and chapter structure is consistent.

**Independent Test**: A new chapter created from the template renders correctly and appears in the sidebar in the intended location.

- [x] T030 [P] [US4] Add a “How to Write Chapters/Labs/Quizzes” contributor guide in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\reference\authoring-guide.md`
- [x] T031 [US4] Update sidebar ordering to reflect module grouping (Intro → M1 → M2 → M3 → M4 → Capstone) in `C:\ai_cli\hackathon\hackathon1\hackathon1\sidebars.ts`
- [x] T032 [P] [US4] Ensure each existing chapter includes the four required fields (objectives, key terms, lab deliverable, assessment item) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\01-embodied-ai.md`
- [x] T033 [P] [US4] Ensure each existing chapter includes the four required fields (objectives, key terms, lab deliverable, assessment item) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\02-ros2-fundamentals.md`
- [x] T034 [P] [US4] Ensure each existing chapter includes the four required fields (objectives, key terms, lab deliverable, assessment item) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\03-rclpy-nodes.md`
- [x] T035 [P] [US4] Ensure each existing chapter includes the four required fields (objectives, key terms, lab deliverable, assessment item) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\04-urdf-tf2.md`
- [x] T036 [P] [US4] Ensure each existing chapter includes the four required fields (objectives, key terms, lab deliverable, assessment item) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\05-gazebo-simulation.md`
- [x] T037 [P] [US4] Ensure each existing chapter includes the four required fields (objectives, key terms, lab deliverable, assessment item) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\06-unity-simulation.md`

---

## Phase 7: User Story 5 - Modules 1-4 chapters + labs + quizzes (Priority: P5)

**Goal**: Modules 1–4 are complete, with chapters and one lab per module (plus assessments) in a consistent structure.

**Independent Test**: Sidebar provides access to all Module 1–4 pages; each module has at least one lab page; build passes with no broken links.

- [x] T038 [P] [US5] Create and write Chapter 7 (Isaac Sim synthetic data + Isaac ROS overview) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\07-isaac-sim-isaac-ros.md`
- [x] T039 [P] [US5] Create and write Chapter 8 (Isaac ROS VSLAM + Nav2 integration concepts + verification plan) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\08-nav2.md`
- [x] T040 [P] [US5] Create and write Chapter 9 (VLA: Whisper voice-to-action + LLM planning to ROS actions) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\09-vla-whisper-llm-planning.md`
- [x] T041 [US5] Update sidebar to add Chapters 7–9 in the correct module positions in `C:\ai_cli\hackathon\hackathon1\hackathon1\sidebars.ts`
- [x] T042 [P] [US5] Create and write Module 3 Isaac lab (synthetic data and/or VSLAM pipeline deliverable + verification) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\labs\module-3-isaac-lab.md`
- [x] T043 [P] [US5] Create and write Module 4 VLA lab (speech/text intent → validated plan → ROS action demo) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\labs\module-4-vla-lab.md`
- [x] T044 [US5] Add Labs sidebar items for Module 3 and Module 4 labs in `C:\ai_cli\hackathon\hackathon1\hackathon1\sidebars.ts`
- [x] T045 [P] [US5] Write Isaac Sim troubleshooting page (GPU/driver pitfalls, remote GPU notes, caches) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\troubleshooting\isaac-sim-gpu-drivers.md`
- [x] T046 [US5] Add Troubleshooting sidebar item for Isaac Sim GPU/driver page in `C:\ai_cli\hackathon\hackathon1\hackathon1\sidebars.ts`

---

## Phase 8: User Story 6 - Capstone + hardware/lab architecture chapters (Priority: P6)

**Goal**: Capstone is defined end-to-end and students/instructors have clear hardware + lab architecture guidance (on-prem vs Ether lab) with BOM tables.

**Independent Test**: The capstone and hardware pages are reachable from the sidebar and contain concrete deliverables, budgets, and BOM tables.

- [x] T047 [P] [US6] Create and write Chapter 10 (Capstone end-to-end pipeline and rubric gates) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\10-capstone.md`
- [x] T048 [P] [US6] Create and write Capstone lab (end-to-end bringup + evidence checklist) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\labs\capstone-lab.md`
- [x] T049 [P] [US6] Create and write hardware + lab architectures overview (on-prem vs Ether lab; tradeoffs; budgets) with BOM tables in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\reference\hardware-lab-architectures.md`
- [x] T050 [P] [US6] Create and write on-prem lab BOM detail page (parts list, quantities, pricing assumptions) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\reference\on-prem-lab-bom.md`
- [x] T051 [P] [US6] Create and write Ether lab BOM/cost model page (instance types, storage, utilization caps, per-student/month estimates) in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\reference\ether-lab-bom.md`
- [x] T052 [US6] Update sidebar to add Capstone chapter + capstone lab + hardware/BOM pages in `C:\ai_cli\hackathon\hackathon1\hackathon1\sidebars.ts`

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Ensure consistency, eliminate broken links, and document verification outcomes.

- [x] T053 Audit and fix internal links/cross-references in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\intro.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\course-setup.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\01-embodied-ai.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\02-ros2-fundamentals.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\03-rclpy-nodes.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\04-urdf-tf2.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\05-gazebo-simulation.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\06-unity-simulation.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\07-isaac-sim-isaac-ros.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\08-nav2.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\09-vla-whisper-llm-planning.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\10-capstone.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\labs\module-1-ros2-lab.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\labs\module-2-gazebo-lab.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\labs\module-3-isaac-lab.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\labs\module-4-vla-lab.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\labs\capstone-lab.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\troubleshooting\ros2-install.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\troubleshooting\gazebo-issues.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\troubleshooting\isaac-sim-gpu-drivers.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\reference\hardware-lab-architectures.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\reference\on-prem-lab-bom.md`, `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\reference\ether-lab-bom.md`
- [x] T054 [P] Create required diagram assets and reference them from the relevant pages in `C:\ai_cli\hackathon\hackathon1\hackathon1\static\img\capstone-pipeline.svg`, `C:\ai_cli\hackathon\hackathon1\hackathon1\static\img\ether-lab-topology.svg`, `C:\ai_cli\hackathon\hackathon1\hackathon1\static\img\ros2-sim-stack.svg`
- [x] T055 Update the build/deploy verification log with final local build results and deployed smoke-check notes in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\reference\build-deploy-verification.md`
- [x] T056 Update course setup page with “minimum vs recommended hardware” and links to hardware/BOM pages in `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\course-setup.md`

---

## Dependencies & Execution Order

### User Story Dependency Graph (recommended)

- Phase 1 → Phase 2 (blocks all stories)
- US1 (MVP browsing) depends on Phase 1–2
- US2 (deployment) depends on Phase 1 (and may proceed in parallel with US3–US4 after Phase 2)
- US3 (runnable labs) depends on Phase 2
- US4 (templates + consistent structure) depends on Phase 1–2
- US5 (Modules 1–4 completion) depends on US4 for consistency (recommended, not strictly required)
- US6 (capstone + hardware) depends on US5 for narrative continuity (recommended)

### Parallel Opportunities (high-level)

- After Phase 2, US2 (deploy) can proceed in parallel with US3 (labs) and US4 (chapter structure fixes).
- Within US4, chapter updates T032–T037 can run in parallel ([P] not marked to avoid concurrent edits if one person is doing all work; add [P] if multiple writers).
- Within US5, T038–T040 are parallelizable if different authors work on different files.
- Within US6, T047–T051 are parallelizable if different authors work on different files.

---

## Parallel Example: User Story 5

Run in parallel if different contributors own different files:

- Chapter author A: `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\07-isaac-sim-isaac-ros.md`
- Chapter author B: `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\08-nav2.md`
- Chapter author C: `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\chapters\09-vla-whisper-llm-planning.md`
- Lab author D: `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\labs\module-3-isaac-lab.md` and `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\labs\module-4-vla-lab.md`

---

## Task Completeness Self-Check

- US1: Intro + Setup + sidebar + recorded smoke check → T015–T019
- US2: Deploy workflow + README + recorded deploy smoke check → T020–T023
- US3: ROS/Gazebo troubleshooting + 2 module labs + sidebar wiring → T024–T029
- US4: Authoring guide + consistent chapter structure in existing chapters → T030–T037
- US5: Create chapters 7–9 + labs 3–4 + Isaac troubleshooting + sidebar wiring → T038–T046
- US6: Capstone chapter + capstone lab + hardware/ether BOM pages + sidebar wiring → T047–T052
