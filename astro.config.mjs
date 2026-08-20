// @ts-check
import { defineConfig } from 'astro/config';

/**
 * Website source lives in site/src and the build output goes to _site/.
 * Assets stay at the repository root and are resolved through astro:assets with import.meta.glob,
 * so the repository asset naming rules do not need any special-case build copies.
 */
export default defineConfig({
  site: 'https://1dannyyu.github.io',
  base: '/IRSENSORCAR',
  srcDir: './site/src',
  publicDir: './site/public',
  outDir: './_site',
  build: { format: 'directory' },
});
