/**
 * Raspberry Pi 5 40-pin header data.
 *
 * The canonical source is docs/hardware/raspberry-pi-5-pinout.md. This file is
 * only its renderable representation; update both files when pin definitions change.
 */

export type PinGroup =
  | 'power3v3'
  | 'power5v'
  | 'ground'
  | 'i2c'
  | 'uart'
  | 'spi'
  | 'pwm'
  | 'pcm'
  | 'gpio';

export interface Pin {
  /** Physical pin number, 1-40. */
  pin: number;
  /** Primary label, such as `GPIO 18` or `5V`. */
  name: string;
  /** Alternate function, such as `PCM_CLK`; hidden on narrow screens. */
  alt?: string;
  group: PinGroup;
}

/**
 * Group colours intentionally do not use the tokens.css accent values: the legend
 * prioritises distinct groups, while interface accents serve a different purpose.
 */
export const PIN_GROUPS: Record<PinGroup, { label: string; color: string }> = {
  power3v3: { label: '3.3V Power', color: '#f59e0b' },
  power5v: { label: '5V Power', color: '#ef4444' },
  ground: { label: 'Ground', color: '#64748b' },
  i2c: { label: 'I2C', color: '#38bdf8' },
  uart: { label: 'UART', color: '#10b981' },
  spi: { label: 'SPI', color: '#a855f7' },
  pwm: { label: 'PWM', color: '#ec4899' },
  pcm: { label: 'PCM / I2S', color: '#3b82f6' },
  gpio: { label: 'General GPIO', color: '#94a3b8' },
};

export const PI5_PINS: Pin[] = [
  { pin: 1, name: '3.3V', group: 'power3v3' },
  { pin: 2, name: '5V', group: 'power5v' },
  { pin: 3, name: 'GPIO 2', alt: 'SDA', group: 'i2c' },
  { pin: 4, name: '5V', group: 'power5v' },
  { pin: 5, name: 'GPIO 3', alt: 'SCL', group: 'i2c' },
  { pin: 6, name: 'GND', group: 'ground' },
  { pin: 7, name: 'GPIO 4', alt: 'GPCLK0', group: 'gpio' },
  { pin: 8, name: 'GPIO 14', alt: 'TXD', group: 'uart' },
  { pin: 9, name: 'GND', group: 'ground' },
  { pin: 10, name: 'GPIO 15', alt: 'RXD', group: 'uart' },
  { pin: 11, name: 'GPIO 17', group: 'gpio' },
  { pin: 12, name: 'GPIO 18', alt: 'PCM_CLK', group: 'pcm' },
  { pin: 13, name: 'GPIO 27', group: 'gpio' },
  { pin: 14, name: 'GND', group: 'ground' },
  { pin: 15, name: 'GPIO 22', group: 'gpio' },
  { pin: 16, name: 'GPIO 23', group: 'gpio' },
  { pin: 17, name: '3.3V', group: 'power3v3' },
  { pin: 18, name: 'GPIO 24', group: 'gpio' },
  { pin: 19, name: 'GPIO 10', alt: 'MOSI', group: 'spi' },
  { pin: 20, name: 'GND', group: 'ground' },
  { pin: 21, name: 'GPIO 9', alt: 'MISO', group: 'spi' },
  { pin: 22, name: 'GPIO 25', group: 'gpio' },
  { pin: 23, name: 'GPIO 11', alt: 'SCLK', group: 'spi' },
  { pin: 24, name: 'GPIO 8', alt: 'CE0', group: 'spi' },
  { pin: 25, name: 'GND', group: 'ground' },
  { pin: 26, name: 'GPIO 7', alt: 'CE1', group: 'spi' },
  { pin: 27, name: 'GPIO 0', alt: 'ID_SD', group: 'gpio' },
  { pin: 28, name: 'GPIO 1', alt: 'ID_SC', group: 'gpio' },
  { pin: 29, name: 'GPIO 5', group: 'gpio' },
  { pin: 30, name: 'GND', group: 'ground' },
  { pin: 31, name: 'GPIO 6', group: 'gpio' },
  { pin: 32, name: 'GPIO 12', alt: 'PWM0', group: 'pwm' },
  { pin: 33, name: 'GPIO 13', alt: 'PWM1', group: 'pwm' },
  { pin: 34, name: 'GND', group: 'ground' },
  { pin: 35, name: 'GPIO 19', alt: 'PCM_FS', group: 'pcm' },
  { pin: 36, name: 'GPIO 16', group: 'gpio' },
  { pin: 37, name: 'GPIO 26', group: 'gpio' },
  { pin: 38, name: 'GPIO 20', alt: 'PCM_DIN', group: 'pcm' },
  { pin: 39, name: 'GND', group: 'ground' },
  { pin: 40, name: 'GPIO 21', alt: 'PCM_DOUT', group: 'pcm' },
];

/** The three signal pins used by the NeZha board IIC header. */
export const NEZHA_I2C_PINS: readonly number[] = [3, 5, 6];

/** Split pins into physical rows: odd on the left and even on the right. */
export function pinRows(pins: Pin[] = PI5_PINS): { left: Pin; right: Pin }[] {
  const odd = pins.filter((p) => p.pin % 2 === 1);
  const even = pins.filter((p) => p.pin % 2 === 0);
  return odd.map((left, i) => ({ left, right: even[i] }));
}
