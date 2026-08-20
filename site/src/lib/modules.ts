import modulesData from '../data/modules.json';
import { CATEGORIES } from '../data/categories';

export type Module = (typeof modulesData)[number];
export type CategoryKey = (typeof CATEGORIES)[number]['key'];

export const modules: Module[] = modulesData;

/** Category keys that have listing pages (excludes `all`). */
export const listingCategories = CATEGORIES.filter((c) => c.key !== 'all');

export function isCategoryKey(value: string): value is Exclude<CategoryKey, 'all'> {
  return listingCategories.some((c) => c.key === value);
}

export function getModule(id: string): Module | undefined {
  return modules.find((m) => m.id === id);
}

export function modulesInCategory(category: string): Module[] {
  if (category === 'all') return modules;
  return modules.filter((m) => m.category === category);
}

export function categoryLabel(key: string): string {
  const found = CATEGORIES.find((c) => c.key === key);
  return found ? found.label : key;
}

export function inventoryPath(category?: string, id?: string): string {
  if (id && category) return `inventory/${category}/${id}`;
  if (category && category !== 'all') return `inventory/${category}`;
  return 'inventory';
}
