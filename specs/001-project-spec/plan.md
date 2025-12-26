# Implementation Plan: Book Site (Docusaurus) — Build & Ship

**Branch**: `001-project-spec` | **Date**: 2025-12-25 | **Spec**: `C:\ai_cli\hackathon\hackathon1\hackathon1\specs\001-project-spec\spec.md`  
**Input**: Feature specification from `C:\ai_cli\hackathon\hackathon1\hackathon1\specs\001-project-spec\spec.md`

**Note**: This template is filled in by the `/speckit.plan` command. See `.codex/commands/speckit.plan.md` for the execution workflow.

## Summary

Ship a Docusaurus v3 “book” site that is deployable to GitHub Pages, has a clear module/chapter ToC, and contains Modules 1–4 with labs/quizzes plus capstone and hardware/lab architecture chapters, with final polish and deploy verification.

## Technical Context

<!--
  ACTION REQUIRED: Replace the content in this section with the technical details
  for the project. The structure here is presented in advisory capacity to guide
  the iteration process.
-->

**Language/Version**: Node.js (>=18; CI uses 20), TypeScript 5.6  
**Primary Dependencies**: Docusaurus 3.6 (`@docusaurus/core`, `@docusaurus/preset-classic`), React 18, Mermaid theme  
**Storage**: N/A (static site; Markdown/MDX + static assets)  
**Testing**: `npm run typecheck` + `npm run build` (build fails on broken links)  
**Target Platform**: Static web (GitHub Pages)  
**Project Type**: Docusaurus documentation site  
**Performance Goals**: Fast first-load for docs; production build completes in CI within ~10 minutes  
**Constraints**: No server runtime; GitHub Pages base path must be correct (`/<repo>/`)  
**Scale/Scope**: Dozens of Markdown pages, images, Mermaid diagrams

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Gate: Spec completeness**: `spec.md` includes prioritized user stories, acceptance scenarios,
  requirements, and success criteria.
- **Gate: Testability**: each user story has an independent test; test waivers (if any) are
  explicitly documented with alternative verification steps.
- **Gate: Scope discipline**: plan matches the spec; out-of-scope items are deferred, not silently
  added.
- **Gate: Risk visibility**: major risks/unknowns are listed and either resolved (research) or
  intentionally accepted.

**Gate Results (pre-research)**:

- Spec completeness: PASS
- Testability: PASS (manual + CI checks defined per story)
- Scope discipline: PASS (plan covers Phase 1–5 outcomes in `spec.md`)
- Risk visibility: PASS (deployment baseUrl, broken links, content consistency)

## Execution Plan (Phase 1–5)

This section matches the user-requested build-and-ship phases. Each phase lists outputs (files), acceptance checks, and “done when” criteria.

### Phase 1 — Repo + Docusaurus scaffolding + Pages deploy working

**Outputs (files)**:

- `C:\ai_cli\hackathon\hackathon1\hackathon1\package.json`
- `C:\ai_cli\hackathon\hackathon1\hackathon1\docusaurus.config.ts`
- `C:\ai_cli\hackathon\hackathon1\hackathon1\sidebars.ts`
- `C:\ai_cli\hackathon\hackathon1\hackathon1\.github\workflows\deploy.yml`
- `C:\ai_cli\hackathon\hackathon1\hackathon1\README.md`

**Acceptance checks**:

- Local: `npm ci`, `npm run typecheck`, `npm run build`
- CI: GitHub Actions workflow “Deploy to GitHub Pages” succeeds on `main`
- Runtime: GitHub Pages site loads and the intro page is reachable

**Done when**:

- A push to `main` results in an updated GitHub Pages site and local build checks pass.

### Phase 2 — Create ToC + sidebars + chapter templates

**Outputs (files)**:

- `C:\ai_cli\hackathon\hackathon1\hackathon1\sidebars.ts` (module/chapter outline reflected)
- Template pages/snippets under `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\templates\`
  - `chapter-template.md`
  - `lab-template.md`
  - `quiz-template.md`

**Acceptance checks**:

- Sidebar matches the intended module/chapter ordering
- A new chapter created from the template renders correctly locally (`npm run start`)

**Done when**:

- A contributor can create a new chapter/lab/quiz by copying a template and it appears in the sidebar without build errors.

### Phase 3 — Write Module 1–4 chapters + labs + quizzes

**Outputs (files)**:

- Module 1–4 content pages under `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\` (chapters and supporting lab/quiz content)
- Any new images/diagrams under `C:\ai_cli\hackathon\hackathon1\hackathon1\static\img\`

**Acceptance checks**:

- Each module has chapters discoverable from the sidebar
- Each module includes labs and quizzes per template conventions
- `npm run build` succeeds with broken-link checks enabled

**Done when**:

- Modules 1–4 are complete, navigable, and build cleanly (no broken links/assets).

### Phase 4 — Capstone chapter + hardware/lab architecture chapter

**Outputs (files)**:

- `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\capstone.md` (or equivalent)
- `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\hardware-lab-architecture.md` (or equivalent)
- Supporting diagrams/images under `C:\ai_cli\hackathon\hackathon1\hackathon1\static\img\`

**Acceptance checks**:

- Both chapters are linked from the ToC and render correctly
- Any Mermaid diagrams render as intended
- `npm run build` succeeds

**Done when**:

- Capstone and hardware/lab architecture guidance is publish-ready and verified in both local build and deployed site.

### Phase 5 — Polish + final deploy verification

**Outputs (files)**:

- Link/image fixes across `C:\ai_cli\hackathon\hackathon1\hackathon1\docs\`
- Diagram fixes across Markdown/MDX content
- Any CI config tweaks needed for consistent URL/baseUrl behavior

**Acceptance checks**:

- `npm run typecheck` passes
- `npm run build` passes (no broken links, no broken markdown links)
- Deployed Pages smoke test (intro + one chapter per module + capstone) passes

**Done when**:

- Local checks pass and a deployed-site smoke test confirms no 404s and correct rendering for representative content.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/speckit.plan command output)
├── research.md          # Phase 0 output (/speckit.plan command)
├── data-model.md        # Phase 1 output (/speckit.plan command)
├── quickstart.md        # Phase 1 output (/speckit.plan command)
├── contracts/           # Phase 1 output (/speckit.plan command)
└── tasks.md             # Phase 2 output (/speckit.tasks command - NOT created by /speckit.plan)
```

### Source Code (repository root)
<!--
  ACTION REQUIRED: Replace the placeholder tree below with the concrete layout
  for this feature. Delete unused options and expand the chosen structure with
  real paths (e.g., apps/admin, packages/something). The delivered plan must
  not include Option labels.
-->

```text
# [REMOVE IF UNUSED] Option 1: Single project (DEFAULT)
src/
├── models/
├── services/
├── cli/
└── lib/

tests/
├── contract/
├── integration/
└── unit/

# [REMOVE IF UNUSED] Option 2: Web application (when "frontend" + "backend" detected)
backend/
├── src/
│   ├── models/
│   ├── services/
│   └── api/
└── tests/

frontend/
├── src/
│   ├── components/
│   ├── pages/
│   └── services/
└── tests/

# [REMOVE IF UNUSED] Option 3: Mobile + API (when "iOS/Android" detected)
api/
└── [same as backend above]

ios/ or android/
└── [platform-specific structure: feature modules, UI flows, platform tests]
```

**Structure Decision**: Single Docusaurus documentation site (static) deployed via GitHub Pages; source-of-truth content lives under `docs/`, assets under `static/`, and site config under repo root (`docusaurus.config.ts`, `sidebars.ts`).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |

## Constitution Check (post-design)

- Spec completeness: PASS
- Testability: PASS
- Scope discipline: PASS
- Risk visibility: PASS
