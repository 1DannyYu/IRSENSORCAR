import type { Locale } from '../i18n/ui';

/** Infer an image caption from the asset filename. */
export function imageCaption(filename: string, _locale: Locale): string {
  const rules: Array<[RegExp, string]> = [
    [/Front/, '📸 Front View & Components'],
    [/Back/, '🔍 Back View & Pinout Labels'],
    [/Angle/, '📐 Perspective View'],
    [/CloseUp|Pinout/, '🔎 Pinout & Chip Detail'],
    [/Sheet|Chart|Diagram|Guide/, '📋 Reference Chart'],
  ];
  for (const [re, caption] of rules) {
    if (re.test(filename)) return caption;
  }
  return '📷 Module Photo';
}
