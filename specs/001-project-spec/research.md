# Research (Phase 0): Book Site (Docusaurus) — Decisions & Rationale

This phase records the key technical decisions needed to execute the Phase 1–5 shipping plan. No open “NEEDS CLARIFICATION” items remain for planning.

## Decisions

### Hosting & Deploy

- **Decision**: Deploy as a GitHub Pages project site at `https://<owner>.github.io/<repo>/` with `baseUrl: /<repo>/` in production.
- **Rationale**: Fits a static docs/book site; low operational overhead; CI deploy is repeatable and reviewable.
- **Alternatives considered**: Custom hosting (Netlify/Vercel/S3+CloudFront) rejected for added setup and secrets/infra overhead.

### Site Framework

- **Decision**: Use Docusaurus v3 with TypeScript configuration.
- **Rationale**: First-class docs navigation (sidebars), MDX support, built-in broken-link enforcement, and a clean local dev loop.
- **Alternatives considered**: MkDocs/Material and GitBook-style tools rejected to stay aligned with existing repo scaffolding.

### Mermaid & Diagrams

- **Decision**: Enable Mermaid diagrams via Docusaurus Markdown Mermaid support (`@docusaurus/theme-mermaid`).
- **Rationale**: Diagrams stay versioned with content, render consistently, and avoid external tooling.
- **Alternatives considered**: Image-only diagrams rejected due to maintenance cost and poor diffability.

### Content Organization

- **Decision**: Keep learning content under `docs/` and model “Modules” as sidebar categories that contain ordered chapter docs.
- **Rationale**: Docusaurus sidebars already represent hierarchical ToCs; categories map naturally to modules.
- **Alternatives considered**: A separate “module index” generator rejected (unnecessary complexity).

### Chapter/Lab/Quiz Templates

- **Decision**: Provide copyable Markdown templates under `docs/templates/` and enforce a minimal frontmatter convention via JSON Schemas (contracts).
- **Rationale**: Keeps authoring simple and repeatable without custom plugins; schemas provide a lightweight “contract” for consistency.
- **Alternatives considered**: Custom Docusaurus plugin for templates rejected for scope/schedule risk.

## Risks & Mitigations

- **Risk**: Wrong GitHub Pages `baseUrl` breaks assets/navigation.  
  **Mitigation**: Build sets `DOCUSAURUS_URL`/`DOCUSAURUS_BASE_URL` in CI; local dev uses `/`.
- **Risk**: Broken internal links/images fail production builds.  
  **Mitigation**: Keep `onBrokenLinks: "throw"` and run `npm run build` before merge.
- **Risk**: Content drift/inconsistent structure across chapters.  
  **Mitigation**: Use templates + frontmatter schemas; add a Phase 5 polish checklist.

