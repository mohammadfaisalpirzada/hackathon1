# Physical AI & Humanoid Robotics (Quarter 2) — Course Site

This repository contains a Docusaurus v3 documentation site for the course
**Physical AI & Humanoid Robotics (Quarter 2)**.

## Prerequisites (for site development)

- Node.js `>= 18`
- npm `>= 9`

## Local dev

```bash
npm install
npm run start
```

## Build

```bash
npm run build
npm run serve
```

## Deploy (GitHub Pages)

Deployment is handled by `.github/workflows/deploy.yml`.

Assumption: the site is served at `https://<owner>.github.io/<repo>/`.
If you deploy to a different URL/base path, set `DOCUSAURUS_URL` and
`DOCUSAURUS_BASE_URL` in the workflow (or repository variables).

