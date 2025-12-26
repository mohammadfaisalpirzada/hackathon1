# Feature Specification: Physical AI & Humanoid Robotics (Quarter 2) — Docusaurus Course Book Site

**Feature Branch**: `001-project-spec`  
**Created**: 2025-12-25  
**Status**: Draft  
**Input**: Create the full specification for this project:
1) book info and setup, 2) table of contents, 3) repo structure, 4) GitHub Pages deployment spec,
5) per-chapter objectives/terms/deliverables/assessments.

## Clarifications

### Session 2025-12-25

- Q: Course title (exact) → A: Physical AI & Humanoid Robotics (Quarter 2)
- Q: Audience → A: Intermediate developers (basic Python/ML) new to ROS 2 + simulation
- Q: Required tooling baseline → A: Ubuntu 22.04 + ROS 2 Humble + Gazebo (core path)
- Q: Optional tooling → A: Isaac Sim/Isaac ROS (advanced track), Unity (optional visualization track)
- Q: GitHub Pages path (root vs /repo/) → A: Project Pages at `/<repo>/` (baseUrl `/<repo>/`)
- Q: Docusaurus version choice → A: Docusaurus v3
- Q: Ether lab → A: Covered as a cloud lab architecture option (non-blocking)

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.
  
  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Read the course book on GitHub Pages (Priority: P1)

As a student, I want to browse the course content on a public website so I can follow the material
without installing anything.

**Why this priority**: The primary value is accessible, navigable course content.

**Independent Test**: Open the deployed GitHub Pages URL and navigate across modules using the
sidebar; confirm no 404s and Mermaid diagrams render.

**Acceptance Scenarios**:

1. **Given** the GitHub Pages site is deployed, **When** I visit the homepage, **Then** I see the
   title "Physical AI & Humanoid Robotics (Quarter 2)" and can navigate to at least one module.
2. **Given** I am on a chapter page, **When** I use sidebar navigation, **Then** the next page
   loads without 404s and the URL stays under `/<repo>/`.

---

### User Story 2 - Maintain and publish content (Priority: P2)

As an instructor/maintainer, I want to edit content in the repo and have it publish to GitHub Pages
so updates are easy and repeatable.

**Why this priority**: Without a reliable publish flow, content updates become manual and error-prone.

**Independent Test**: Make a small docs change, run a build locally, and verify the same change appears after a Pages deploy.

**Acceptance Scenarios**:

1. **Given** I change a Markdown file in the docs, **When** I run the site build, **Then** it completes successfully and the updated content appears in the output.
2. **Given** a commit is pushed to the default branch, **When** the GitHub Pages deploy workflow
   runs, **Then** the site updates under `/<repo>/`.

---

### User Story 3 - Follow runnable ROS 2 labs (Priority: P3)

As a student, I want labs with runnable commands and explicit file paths so I can complete the
course on Ubuntu 22.04 + ROS 2 Humble.

**Why this priority**: The course is practice-heavy; labs must be executable, not hand-wavy.

**Independent Test**: Follow the setup and at least one lab end-to-end, verifying commands and
packages exist and run on a clean Ubuntu 22.04 machine.

**Acceptance Scenarios**:

1. **Given** I follow the setup instructions, **When** I run the ROS 2 sanity checks, **Then**
   commands succeed and I can run a demo node.
2. **Given** I follow a lab chapter, **When** I execute each command in order, **Then** I can
   observe expected ROS topics/nodes/actions via CLI tools.

---

### User Story 4 - ToC + sidebars + chapter templates (Priority: P4)

As a reader, I want a clear table of contents (Intro + Modules + Capstone) and consistent chapter
structure so the site feels like a cohesive technical book.

**Why this priority**: Navigation + consistent structure is the baseline UX for book-style learning content.

**Independent Test**: Run the site locally and verify sidebar ordering and template sections render on a representative chapter.

**Acceptance Scenarios**:

1. **Given** the site is running locally, **When** I open the sidebar, **Then** I see modules and chapter items in the agreed order.
2. **Given** a chapter page, **When** it renders, **Then** it contains consistent template headings (learning objectives, prerequisites, summary/next steps).

---

### User Story 5 - Modules 1–4 chapters + labs + quizzes (Priority: P5)

As a learner, I want Modules 1–4 written with labs and quizzes so I can learn by reading, practicing, and self-checking.

**Why this priority**: This is the core “book content” value after the deploy + structure are in place.

**Independent Test**: Build the site and spot-check each module’s pages for reachability, rendering, and internal links.

**Acceptance Scenarios**:

1. **Given** Modules 1–4 are authored, **When** I click through the ToC, **Then** every chapter loads without broken links or missing assets.
2. **Given** a lab section, **When** I follow the steps, **Then** it includes verification criteria and expected outputs.

---

### User Story 6 - Capstone + hardware/lab architecture chapters (Priority: P6)

As a learner, I want capstone guidance and a hardware/lab architecture chapter so I can build and evaluate a final project with clear expectations.

**Why this priority**: These chapters tie the course/book together and reduce onboarding friction.

**Independent Test**: Build the site and confirm both chapters render, are linked, and include diagrams/checklists.

**Acceptance Scenarios**:

1. **Given** the capstone chapter exists, **When** I open it, **Then** it includes goals, deliverables, and evaluation criteria/rubric.
2. **Given** the hardware/lab architecture chapter exists, **When** I open it, **Then** it includes a reference architecture diagram and minimal/recommended setups.

---

### User Story 7 - Polish + final deploy verification (Priority: P7)

As a maintainer, I want a final polish pass (links, images, Mermaid) and a deploy verification so the shipped site is reliable.

**Why this priority**: Broken links/assets undermine credibility and create support burden.

**Independent Test**: Run local checks (`typecheck`, `build`), then do a deployed-site smoke check.

**Acceptance Scenarios**:

1. **Given** the site is ready to ship, **When** I run `npm run typecheck` and `npm run build`, **Then** both succeed.
2. **Given** GitHub Pages is deployed, **When** I open representative pages (intro, one chapter per module, capstone), **Then** images and Mermaid diagrams render and navigation works.

### Edge Cases

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right edge cases.
-->

- What happens when a page link is moved/renamed (avoid broken internal links / 404s)?
- How does the build/deploy surface broken Markdown links, missing images, or Mermaid errors?
- What happens when `baseUrl` is wrong for GitHub Pages (assets 404)?
- What happens when lab commands drift from ROS 2 Humble (pinning vs updates)?
- What happens when a student cannot run Isaac Sim (clear alternative path)?

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: The site MUST be built using Docusaurus v3.
- **FR-002**: The deployed site MUST be compatible with GitHub Pages project sites using `baseUrl`
  `/<repo>/`.
- **FR-003**: The repo MUST be structured so that content updates are made via Markdown (docs) and versioned in git.
- **FR-004**: The landing page MUST display the exact title
  "Physical AI & Humanoid Robotics (Quarter 2)".
- **FR-005**: The navigation (sidebar) MUST allow a reader to reach all primary chapters without broken links.
- **FR-006**: Ether lab (cloud) content MUST be included as a non-blocking lab architecture option.
- **FR-007**: Unity-related content MUST be optional and MUST NOT require Unity to build or deploy the docs site.
- **FR-008**: The navigation (sidebar) MUST support modules and chapters with a predictable ordering.
- **FR-009**: The content MUST define and follow templates for chapters, labs, and quizzes (minimum: headings + frontmatter conventions).
- **FR-010**: Mermaid diagrams MUST be supported in Markdown/MDX and render correctly.
- **FR-011**: Production builds MUST fail on broken links and broken markdown links.
- **FR-012**: Final verification MUST include local checks and a deployed-site smoke test.
- **FR-013**: The specification MUST define (a) book info, (b) table of contents, (c) repo structure,
  (d) GitHub Pages deployment spec, and (e) per-chapter objectives + assessment items.
- **FR-014**: The content MUST cover (at minimum): Physical AI/embodied intelligence, ROS 2, URDF,
  Gazebo + Unity (Unity optional), Isaac Sim/Isaac ROS, Nav2, VLA (Whisper + LLM planning), and a
  capstone.
- **FR-015**: All lab code and commands MUST be runnable on Ubuntu 22.04 + ROS 2 Humble, with file
  paths and verification steps.

### Acceptance Criteria (verification per requirement)

- **AC-001 (FR-001)**: `package.json` and site configuration indicate Docusaurus v3 usage, and a production build produces a static artifact.
- **AC-002 (FR-002)**: GitHub Pages deployment settings include a non-root base path (project pages), and deployed assets load without 404s under `/<repo>/`.
- **AC-003 (FR-003)**: All instructional content lives under `docs/` as Markdown, and edits are tracked via git.
- **AC-004 (FR-004)**: The deployed homepage renders the exact title string in the UI.
- **AC-005 (FR-005, FR-008)**: Sidebar navigation reaches every required chapter page in the specified order.
- **AC-006 (FR-006)**: A dedicated section describes the Ether lab topology, tradeoffs, and student workflow.
- **AC-007 (FR-007)**: Unity references are clearly labeled optional and do not block core labs.
- **AC-008 (FR-009)**: Each chapter page includes learning objectives, key terms, lab deliverable, and assessment item.
- **AC-009 (FR-010)**: At least one Mermaid diagram renders correctly in the deployed site.
- **AC-010 (FR-011, FR-012)**: Build/deploy fails on broken links and a post-deploy smoke test checklist is defined.
- **AC-011 (FR-013, FR-014)**: This specification includes all requested modules and topics (ROS 2, Gazebo, Unity optional, Isaac, Nav2, VLA, capstone).
- **AC-012 (FR-015)**: Each lab includes a verifiable outcome (expected topics/nodes/artifacts) and is runnable on the stated baseline environment.

### Book information (non-negotiable)

- **Title**: Physical AI & Humanoid Robotics (Quarter 2)
- **Subtitle**: ROS 2 Humble + Gazebo + Isaac + VLA Capstone for Intermediate Python/ML Developers
- **Audience**: intermediate developers who know basic Python/ML but are new to ROS 2 + simulation
- **Target outcomes**:
  - Build ROS 2 systems in Python (`rclpy`) with clear topic/service/action boundaries.
  - Model robots with URDF/Xacro and reason about TF2 frames.
  - Validate behavior in simulation (Gazebo core path; Unity optional visualization track).
  - Understand Isaac Sim vs Isaac ROS and integrate accelerated perception patterns.
  - Use Nav2 for navigation and integrate a VLA pipeline safely (schema validation before actions).
  - Deliver an end-to-end capstone pipeline and demonstrate it reliably.
- **Prerequisites**:
  - Comfortable with Linux shell basics and Python.
  - Familiar with ML concepts (inference loops, latency tradeoffs); no prior ROS required.
- **Tooling**:
  - Required: Ubuntu 22.04, ROS 2 Humble, Gazebo (Classic), Git, Node.js (site dev).
  - Covered: Isaac Sim + Isaac ROS (advanced/optional).
  - Optional: Unity (resources + patterns; not required for the core path).
- **Student setup**: the book MUST include setup instructions that install ROS 2 Humble, create a
  workspace, and run sanity-check demos.

### Table of contents (requested module structure)

- **Intro**: Physical AI + sensors
- **Module 1 (ROS 2)**: nodes/topics/services/actions, `rclpy` bridge/patterns, URDF basics + TF2
- **Module 2 (Digital Twin)**: Gazebo physics/sensors (core) + Unity visualization (optional)
- **Module 3 (Isaac)**: Isaac Sim synthetic data + Isaac ROS VSLAM + Nav2
- **Module 4 (VLA)**: Whisper voice-to-action + LLM planning to ROS actions
- **Capstone**: autonomous humanoid pipeline end-to-end
- **Hardware & lab architectures**: on-prem vs cloud "Ether lab" + budgets

### Weekly breakdown (default pacing, Quarter 2 = 10 weeks)

- **Week 1**: Intro + Setup (tooling, ROS CLI, talker/listener, workspace conventions)
- **Week 2–3**: Module 1 (ROS 2 foundations) — Chapters 1–4
- **Week 4–5**: Module 2 (Digital Twin) — Chapters 5–6 (Unity optional track can run in parallel)
- **Week 6–7**: Module 3 (Isaac + Navigation) — Chapters 7–8 (Ether lab path supported)
- **Week 8**: Module 4 (VLA) — Chapter 9 (voice-to-action + planner-to-actions, safety gates)
- **Week 9–10**: Capstone — Chapter 10 (integration, evaluation, demo readiness)

### Assessments (default policy + weights)

- **Quizzes (20%)**: Low-stakes concept checks (typically 3–5 questions) attached to most chapters.
- **Labs (40%)**: Verified artifacts (code + command outputs + evidence) for Chapters 1–9.
- **Capstone (40%)**: End-to-end demo + rubric (reliability, explainability, reproducibility).

### Docusaurus repo structure (exact paths)

- `docs/`
  - `docs/intro.md`
  - `docs/course-setup.md`
  - `docs/chapters/01-embodied-ai.md`
  - `docs/chapters/02-ros2-fundamentals.md`
  - `docs/chapters/03-rclpy-nodes.md`
  - `docs/chapters/04-urdf-tf2.md`
  - `docs/chapters/05-gazebo-simulation.md`
  - `docs/chapters/06-unity-simulation.md`
  - `docs/chapters/07-isaac-sim-isaac-ros.md`
  - `docs/chapters/08-nav2.md`
  - `docs/chapters/09-vla-whisper-llm-planning.md`
  - `docs/chapters/10-capstone.md`
  - `docs/reference/ros2-cheatsheet.md`
  - `docs/reference/glossary.md`
  - `docs/reference/hardware-lab-architectures.md`
- `static/`
  - `static/img/`
- `sidebars.ts`
- `docusaurus.config.ts`
- `src/css/custom.css`
- `.github/workflows/deploy.yml`
- **Docs versioning**: none initially (single current version). Enable later only if multiple
  quarters must be hosted simultaneously.

### GitHub Pages deployment specification

- `url`: `https://<owner>.github.io`
- `baseUrl`: `/<repo>/`
- `organizationName`: `<owner>`
- `projectName`: `<repo>`
- Workflow file: `.github/workflows/deploy.yml`

Workflow requirements (must be satisfied by `.github/workflows/deploy.yml`):

- Deploys on push to the default branch and on manual dispatch
- Uses Node.js 18+ and produces a production build artifact
- Publishes via GitHub Pages GitHub Actions (no external tokens required)
- Allows overriding `DOCUSAURUS_URL` and `DOCUSAURUS_BASE_URL` via repository variables for non-default deployments

Workflow steps (logical outline; exact YAML may vary):

- Checkout repository
- Set up Node.js and cache dependencies
- Install dependencies and build a production artifact
- Upload the build artifact for Pages
- Deploy artifact to GitHub Pages

### Per-chapter specification (objectives, terms, deliverable, assessment)

For each chapter, define:

- learning objectives
- key terms
- lab deliverable (verifiable artifact)
- assessment item

Chapters required (minimum set, may be expanded):

1. Intro: Physical AI & sensors (embodied loop + constraints)
2. Module 1: ROS 2 fundamentals (nodes/topics/services/actions)
3. Module 1: `rclpy` patterns + debugging
4. Module 1: URDF basics + TF2
5. Module 2: Gazebo physics + sensors
6. Module 2: Unity visualization (optional)
7. Module 3: Isaac Sim synthetic data (optional/advanced)
8. Module 3: Isaac ROS VSLAM + Nav2 integration
9. Module 4: Whisper voice-to-action (optional/advanced)
10. Module 4: LLM planning → validated schema → ROS 2 actions
11. Capstone: autonomous humanoid pipeline end-to-end
12. Hardware & lab architectures (on-prem vs Ether lab) + budgets

#### Chapter-by-chapter requirements (concrete)

Each row below MUST be implemented as a page (or an equivalent split across pages). Paths reflect
the expected Docusaurus repository layout.

| Chapter | Module | Page path | Learning objectives (must be measurable) | Key terms (minimum) | Lab deliverable | Assessment item |
|--------|--------|----------|------------------------------------------|---------------------|----------------|-----------------|
| Intro | Intro | `docs/intro.md` | Explain embodied loop constraints and why ROS 2 is used; identify the toolchain used in this course | embodied intelligence, latency, feedback, sim-to-real | Run ROS 2 demo node and verify with CLI | 3-question quiz |
| Setup | Intro | `docs/course-setup.md` | Install ROS 2 Humble; create `~/q2_ws`; validate Gazebo path | workspace, `colcon`, `rosdep`, sim time | Working `~/q2_ws` and successful `colcon build` | Setup checklist (pass/fail) |
| Physical AI / embodied | Intro | `docs/chapters/01-embodied-ai.md` | Implement a closed-loop controller using ROS topics; explain timing constraints | control loop, state estimate, policy, actuator command | `rclpy` node reaches a goal in `turtlesim` | Quiz + code check |
| ROS 2 primitives | M1 | `docs/chapters/02-ros2-fundamentals.md` | Choose between topics/services/actions; debug a running graph | topic, service, action, parameter | Publisher + service in `ament_python` package | Practical: demonstrate CLI inspection + service call |
| `rclpy` patterns | M1 | `docs/chapters/03-rclpy-nodes.md` | Explain and demonstrate QoS differences; use latched config safely | QoS, reliability, durability, transient local | QoS demo node + verification via `ros2 topic info` | Quiz + short written rationale |
| URDF + TF2 | M1 | `docs/chapters/04-urdf-tf2.md` | Build a minimal URDF/Xacro and visualize TF in RViz | URDF, Xacro, TF tree, `base_link` | Robot renders in RViz with correct fixed frame | Practical: TF verification output + screenshot |
| Gazebo digital twin | M2 | `docs/chapters/05-gazebo-simulation.md` | Spawn a robot in Gazebo; drive via `/cmd_vel`; verify `/odom` | plugin, odometry, `use_sim_time`, diff drive | Gazebo launch + teleop drive + odom topic evidence | Practical: demonstrate motion + odom evidence |
| Unity visualization (optional) | M2 | `docs/chapters/06-unity-simulation.md` | Explain ROS↔Unity bridge pattern; keep message contracts stable | bridge, topic contract, schema | ROS-side mirror node + message round-trip test | Optional quiz |
| Isaac Sim/ROS (optional) | M3 | `docs/chapters/07-isaac-sim-isaac-ros.md` | Distinguish Isaac Sim vs Isaac ROS; identify when acceleration matters | synthetic data, throughput, latency, pipeline | Baseline vision pipeline; optional accelerated swap | Short written comparison |
| Nav2 | M3 | `docs/chapters/08-nav2.md` | Launch Nav2; send goals; debug TF/costmaps | map/odom/base_link, AMCL, costmap, controller | TurtleBot3 reaches a goal in sim | Practical: goal success + explain one failure mode |
| VLA planning to actions | M4 | `docs/chapters/09-vla-whisper-llm-planning.md` | Enforce schema validation; connect plans to ROS actions safely | schema, validator, executor, action client | Text command → validated plan → Nav2 goal | Quiz + code check (reject unknown commands) |
| Capstone | Capstone | `docs/chapters/10-capstone.md` | Integrate model+sim+nav+VLA into reproducible bringup | bringup, launch, observability | One-command bringup (`ros2 launch ...`) | Capstone demo + rubric |
| Hardware + Ether lab | Reference | `docs/reference/hardware-lab-architectures.md` | Choose lab topology; budget hardware; explain tradeoffs | workstation, VRAM, remote GPU, tunnel/VPN | Documented lab plan for on-prem and Ether lab | Short written architecture justification |

### Hardware & lab architectures (explicit)

- On-prem (student workstation):
  - Minimum: 4 CPU cores, 16 GB RAM, 30 GB disk, integrated GPU acceptable for RViz/basic Gazebo.
  - Recommended: 8+ CPU cores, 32–64 GB RAM, 100 GB disk, NVIDIA RTX GPU with 12–16 GB VRAM for
    Isaac Sim experiments.
- Cloud “Ether lab” (remote GPU):
  - A GPU VM runs Isaac/compute-heavy nodes; student uses a local machine for browser + lightweight
    ROS tools; ROS graph access is provided via secure tunneling/VPN.
- Budgets (typical ranges; currency = USD):
  - **Student workstation (recommended tier)**: $1,500–$3,500 per seat (GPU workstation suitable for Gazebo + optional Isaac).
  - **On-prem cohort lab (10 seats)**: $15,000–$35,000 for workstations + $2,000–$6,000 for shared storage/networking.
  - **Ether lab (cloud GPU)**: $150–$600 per active student-month depending on GPU class and usage caps.

### Key Entities *(include if feature involves data)*

- **Course Book Site**: static site (docs + config + assets).
- **Chapter**: content unit with required sections and assessments.
- **Lab**: runnable commands/code with verifiable outcomes.
- **Assessment Item**: quiz/practical/rubric evidence tied to chapters.
- **Lab Architecture**: on-prem vs Ether lab environment definition.

## Success Criteria *(mandatory)*

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

- **SC-001**: A production build completes successfully and produces a deployable static site artifact.
- **SC-002**: GitHub Pages serves the site under `https://<owner>.github.io/<repo>/` without broken
  navigation links (no 404s via sidebar traversal).
- **SC-003**: Every chapter includes the required sections (prereqs/concepts/lab/troubleshooting/quiz).
- **SC-004**: A student can complete the setup and at least one ROS 2 lab end-to-end on Ubuntu 22.04
  + ROS 2 Humble, with verifiable outputs.
- **SC-005**: The Table of Contents matches the requested modules and clearly labels optional tracks
  (Unity, Isaac) without blocking the core path.
- **SC-006**: The spec includes hardware and lab architecture guidance with minimum vs recommended
  requirements and a concrete budget section.
- **SC-007**: Each chapter defines learning objectives, key terms, lab deliverable, and assessment item.

## Assumptions

The request references "weekly breakdown, hardware list, assessments" as source of truth, but those
details were not present in the repository content at time of writing. The defaults below are used
to complete this specification end-to-end.

## Question 1: Weekly breakdown (course pacing)

**Context**: The TOC is module-based, but weekly allocation is unspecified.

**Default decision**: Quarter 2 is **10 weeks**, mapped as: Week 1 Intro+Setup; Weeks 2–3 M1; Weeks 4–5 M2; Weeks 6–7 M3; Week 8 M4; Weeks 9–10 Capstone.

**Suggested Answers**:

| Option | Answer | Implications |
|--------|--------|--------------|
| A | 8 weeks (2 weeks per module) + capstone week | Shorter, faster pacing; fewer optional deep dives |
| B | 10 weeks (Intro+Setup, 2w M1, 2w M2, 2w M3, 2w M4, 1w capstone) | Balanced pacing with time for integration |
| C | 12 weeks (adds dedicated hardware/ops + review weeks) | More time for capstone + optional Isaac/Unity |
| Custom | Provide week count + mapping | We will align chapters/assessments accordingly |

**Selected default**: Option B (10 weeks).

## Question 2: Assessments (types + weights)

**Context**: Each chapter lists an assessment item, but grading policy is unspecified.

**Default decision**: **Quizzes 20%, Labs 40%, Capstone 40%**.

**Suggested Answers**:

| Option | Answer | Implications |
|--------|--------|--------------|
| A | Quizzes 20%, Labs 40%, Capstone 40% | Practical-heavy; encourages building |
| B | Quizzes 30%, Labs 30%, Capstone 40% | More theory-check; still capstone-driven |
| C | Labs 60%, Capstone 40% (no quizzes) | Purely hands-on; requires strong rubric clarity |
| Custom | Provide types + weights | We will adjust chapter assessment items to match |

**Selected default**: Option A (20/40/40).

## Question 3: Hardware list + budgets (concrete)

**Context**: The spec includes minimum vs recommended, but you requested hardware list + budgets (on-prem and Ether lab).

**Default decision**: Currency is **USD**; use **3 on-prem tiers** (minimum/recommended/pro) and **2 cloud tiers** (entry GPU vs mid/high GPU) with cost caps per student-month.

**Suggested Answers**:

| Option | Answer | Implications |
|--------|--------|--------------|
| A | 2 on-prem tiers ($1k / $2k) + 1 cloud tier ($200/month) | Simple purchasing and planning |
| B | 3 on-prem tiers (min / rec / pro) + 2 cloud tiers (T4 vs A10/A100) | Covers Isaac Sim variability; more complexity |
| C | Cloud-first Ether lab only (no local GPU expectation) | Lowest student hardware requirement; higher ops overhead |
| Custom | Provide list + budgets | We will encode exact requirements and lab topology |

**Selected default**: Option B (tiered on-prem + tiered cloud).
