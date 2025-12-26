---
title: "Build + Deploy Verification Log"
---

# Build + Deploy Verification Log

Use this page to record “known good” build outputs and deployment smoke checks for each cohort/release.

## Local build checks

```bash
npm ci
npm run typecheck
npm run build
```

Record results:

- Date: 2025-12-25
- Node version: v22.20.0
- `npm run typecheck`: PASS
- `npm run build`: PASS (outputs to `dist/`)

## GitHub Pages deployment

Expected URL shape:

- `https://<owner>.github.io/<repo>/`

Record results:

- Date:
- Commit SHA:
- Deployed URL:

## Smoke check (post-deploy)

- [ ] Homepage loads (intro)
- [ ] Sidebar navigation works (no 404s)
- [ ] Mermaid diagram renders (pick at least one page)
- [ ] Images under `/img/...` render (pick at least one page)
