export const LOCALES = ['en'] as const;
export type Locale = (typeof LOCALES)[number];

export const ui = {
  en: {
    'site.title': "Danny's Hardware & Sensor Inventory",
    'site.subtitle': 'Smart Car + Robotic Concept Project Notes',
    'nav.home': 'Overview',
    'nav.inventory': 'My Inventory and Tools',
    'nav.assembly': 'Assembly Guide',
    'home.eyebrow': 'Raspberry Pi 5 · NeZha I2C · Smart Car + Robotic',
    'home.title': 'Smart Car + Robotic Concept Project',
    'home.lede':
      'A Raspberry Pi 5 drives the NeZha bus board over I2C - four motors, four servos and onboard lighting. This site collects the parts inventory, assembly steps and hardware notes.',
    'project.note':
      'Not every part in this inventory will be used in the final build. The project direction may change because of cost, hardware and software compatibility, or damaged parts, but the overall idea remains a Smart Car + Robotic concept.',
    'inventory.title': 'My Inventory and Tools',
    'inventory.search': 'Search modules or chips (e.g. MPU6050, Servo, PCA9685, Ultrasonic)…',
    'inventory.count': 'Modules',
    'inventory.unit': '',
    'inventory.empty.title': 'No modules matched',
    'inventory.empty.desc': 'Try a different keyword or switch category.',
    'inventory.photos': 'photos',
    'inventory.categories': 'Categories',
    'inventory.category.showing': 'Showing',
    'inventory.back': '← Back to category',
    'inventory.section.photos': 'Module Photos',
    'inventory.section.desc': 'Overview & Details',
    'inventory.section.specs': 'Specifications',
    'inventory.section.raspberryPi': 'Raspberry Pi 5 Wiring',
    'inventory.section.arduino': 'Arduino Wiring',
    'inventory.section.stm32': 'STM32 Wiring',
    'inventory.section.code': 'Code Example',
    'inventory.wiring.pin': 'Pin',
    'inventory.wiring.conn': 'Connects to',
    'pinout.title': 'Raspberry Pi 5 GPIO Pinout',
    'pinout.lede':
      'The 40-pin header, colour-coded by function. Numbering follows the physical board: odd pins in the left column, even pins in the right, with Pin 1 at the end nearest the USB-C power connector.',
    'pinout.legend': 'Function groups',
    'pinout.col.func': 'Function',
    'pinout.col.pin': 'Pin',
    'pinout.nezha':
      'The NeZha bus board IIC header (G / SDA / SCL / 5V) maps to the three marked pins: SDA to Pin 3, SCL to Pin 5, G to Pin 6. Leave the 5V wire off when the board already has its own supply, so two rails do not fight.',
    'pinout.warn':
      'GPIO logic is 3.3V. Never feed a 5V signal into a GPIO pin, and never feed 5V into Pin 1 or Pin 17.',
    'pinout.source': 'Sources',
    'pinout.source.doc': 'Project hardware notes',
    'pinout.source.official': 'Official Raspberry Pi GPIO documentation',
    'assembly.title': 'Dasheng Multi-Form Smart Car & Robotic Arm Assembly Guide',
    'lightbox.filename': 'Filename',
  },
} as const;

export function t(locale: Locale) {
  return (key: keyof (typeof ui)['en']): string => ui[locale][key];
}

/** Build an internal URL from the configured base and a repository-relative path. */
export function href(_locale: Locale, path = ''): string {
  const base = import.meta.env.BASE_URL.replace(/\/+$/, '');
  const clean = path.replace(/^\/|\/$/g, '');
  return `${base}${clean ? `/${clean}` : ''}/`;
}
