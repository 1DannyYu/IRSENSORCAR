import type { ImageMetadata } from 'astro';

/**
 * Convert a repository-relative asset path such as `assets/inventory/001_x.jpg`
 * into Astro ImageMetadata for responsive image generation.
 *
 * Assets live outside srcDir, so the glob is rooted at the repository root.
 */
const files = import.meta.glob<{ default: ImageMetadata }>(
  '/assets/**/*.{jpg,jpeg,png,JPG,PNG}',
  { eager: true },
);

export function asset(path: string): ImageMetadata {
  const key = '/' + path.replace(/^\/+/, '');
  const hit = files[key];
  if (!hit) {
    throw new Error(
      `Asset ${key} was not found. Data paths must be relative to the repository root; see CONVENTIONS.md.`,
    );
  }
  return hit.default;
}

export function hasAsset(path: string): boolean {
  return ('/' + path.replace(/^\/+/, '')) in files;
}
