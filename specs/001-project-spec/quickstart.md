# Quickstart (Phase 1): Local Dev, Build, Deploy

## Prerequisites

- Node.js `>= 18` (CI uses Node 20)
- npm `>= 9`

## Install

```bash
npm ci
```

## Local dev

```bash
npm run start
```

Open the printed URL and confirm navigation works.

## Typecheck

```bash
npm run typecheck
```

## Production build + serve

```bash
npm run build
npm run serve
```

## Deploy (GitHub Pages)

- Deployment is handled by `C:\ai_cli\hackathon\hackathon1\hackathon1\.github\workflows\deploy.yml`.
- The workflow sets:
  - `DOCUSAURUS_URL=https://<owner>.github.io`
  - `DOCUSAURUS_BASE_URL=/<repo>/`
  - `GITHUB_REPOSITORY_OWNER` and `GITHUB_REPOSITORY` for repo links

## Smoke-check checklist (post-deploy)

- Homepage loads (intro).
- Sidebar navigation works for at least one chapter per module.
- A page with Mermaid content renders diagrams.
- Images referenced from `/img/...` render.

