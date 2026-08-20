# ADR 0001: Frontend Architecture for the Project Website

- **Status**: adopted, phases 0-2 completed; English-only content amendment adopted 2026-08-21
- **Date**: 2026-07-30

## Context

The project needs a static website for the project report, published through GitHub Pages.

At the start of this work, `site/` contained three hand-written single-file HTML pages:

| Page | Approx. lines | Purpose |
|---|---|---|
| `site/inventory/index.html` | ~1800 | Inventory browser in Traditional Chinese |
| `site/inventory/index.en.html` | ~1770 | English inventory browser |
| `site/assembly-guide/index.en.html` | ~1490 | English assembly guide with 50 scanned manual pages |

### Problems in the Original Structure

1. **CSS duplication**: each page embedded its own ~700-line `<style>` block
2. **Data hardcoded in JS**: `MODULES_DATA` was duplicated across language versions and had already drifted out of sync
3. **No shared layout**: every new page required another full copy-and-paste

During cleanup, three additional bugs were found that had gone unnoticed precisely because the pages
were isolated single files: the Chinese page was missing lightbox markup, the English page had an
unterminated `<script>`, and the gallery was not wired to the lightbox correctly.

### Constraints

- GitHub Pages publish size limit is **1GB**
- Website content was bilingual during the initial migration; it is now English-only
- Existing Markdown files in `docs/` should remain the canonical source when possible
- The team prefers npm over yarn or pnpm

## Options Considered

### A. Keep Vanilla HTML and Refactor

Move shared CSS into a common file and move `MODULES_DATA` into JSON.

- Pros:
  - zero build tooling
  - minimal migration effort
- Cons:
  - still no true layout reuse
  - image optimization would need custom scripts
  - Markdown-based content would still need manual copying

### B. Astro

- Pros:
  - works well with Markdown and component-based pages
  - `astro:assets` automatically generates responsive images and WebP variants
  - keeps client-side JavaScript optional
  - built-in routing supported the original bilingual requirement
  - official GitHub Pages deployment support exists
- Cons:
  - adds a build step and npm dependencies
  - requires real migration work to split pages into components

### C. VitePress

- Pros:
  - very strong Markdown-first workflow
- Cons:
  - awkward fit for the custom inventory browser
  - more constrained by the default documentation theme

## Decision

Use **Astro**, introduced in phases so each stage can be reviewed independently.

| Phase | Scope | Status |
|---|---|---|
| 0 | Repair existing pages and normalize asset paths | Complete |
| 1 | Extract shared data into structured source files | Complete |
| 2 | Migrate to Astro layouts and components | Complete |
| 3 | Enable GitHub Actions deployment | Workflow prepared, still manual |

### What Phase 2 Changed

- The Astro project lives at the repository root with `srcDir: site/src` and `outDir: _site`.
- `assets/` remains in place. Images are loaded through root-relative `import.meta.glob` usage in
  `site/src/lib/images.ts`, so asset naming rules do not need to change.
- Shared design tokens were consolidated into `site/src/styles/tokens.css`.
- English routes are served directly from `/`; the former `/zh/` route tree was removed.
- Cards are statically rendered by Astro and detailed content is stored in HTML templates that are
  cloned into modals on the client side.

### Why Image Optimization Moved Earlier

Image optimization was originally planned for Phase 3, but `astro:assets` naturally fits the Astro
migration. Delaying it would have meant doing the same work twice. In practice, it reduced sample
image size from roughly 356kB to 155kB.

### Current Status of Phase 3

`.github/workflows/deploy-pages.yml` already exists and has been validated. It remains manual-only
for now because the GitHub Pages source setting has not yet been switched to "GitHub Actions" in
repository settings. Enabling automatic pushes before that switch would only produce predictable
deployment failures.

### Removed Artifacts

- `site/inventory/index.html`
- `site/inventory/index.en.html`
- `site/assembly-guide/index.en.html`
- `scripts/build_assembly_html.py`

These were replaced by Astro pages and structured data files. The old assembly script was removed
because it was a large hardcoded HTML generator rather than a reusable data pipeline.

### English-Only Amendment (2026-08-21)

The site was simplified to one English route tree. Chinese UI strings, Chinese page routes, and
Chinese inventory records were removed. The structured inventory schema retains `i18n.en` so the
content boundary remains explicit without maintaining unused locale branches.

## Consequences

- The website now depends on npm packages listed in `package.json`
- Build output goes to `_site/`, which is ignored
- Local preview uses `npm run dev` instead of `python3 -m http.server`
- Asset naming rules in `CONVENTIONS.md` remain unchanged
