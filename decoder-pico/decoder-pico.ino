#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "hardware/gpio.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"

// -----------------------------------------------------------------------------
// Clock & Core Overclock Configuration
// -----------------------------------------------------------------------------
// 300 MHz: Extreme Integer PLL lock (150 MHz Flash)
// QSPI Flash Clock: 133.0 MHz (QMI /2 Divider)
#define TARGET_SYS_FREQ_KHZ  300000
#define TARGET_I2C_FREQ_HZ   800000 // 800 kHz High-Speed

// -----------------------------------------------------------------------------
// Pin Configuration & Hardware Constants
// -----------------------------------------------------------------------------
const uint8_t PS2_CLK_PIN  = 2; // GP2
const uint8_t PS2_DATA_PIN = 3; // GP3
const uint8_t I2C_SDA_PIN  = 4; // GP4 (External 4.7k Pull-up to 3.3V)
const uint8_t I2C_SCL_PIN  = 5; // GP5 (External 4.7k Pull-up to 3.3V)
const uint8_t LED_KEYPRESS_PIN = 24;
const uint8_t LED_HEARTBEAT_PIN = 25;

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_BUF_SIZE  1024 // 128 * 64 / 8 bytes
#define OLED_RESET     -1
#define OLED_ADDR      0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oled_ready = false;

// -----------------------------------------------------------------------------
// Triple-Buffer Allocation (SRAM)
// -----------------------------------------------------------------------------
uint8_t staging_buf[OLED_BUF_SIZE]; // Buffer 2: Staging / Ready frame latch
uint8_t tx_buf[OLED_BUF_SIZE];      // Buffer 3: Outbound I2C transmission buffer

volatile bool staging_has_new_frame = false;
uint32_t last_display_push_millis = 0;
const uint32_t OLED_REFRESH_INTERVAL_MS = 8; // ~120 FPS Extreme refresh cap

// -----------------------------------------------------------------------------
// PS/2 Circular Ring Buffer & ISR State
// -----------------------------------------------------------------------------
#define PS2_BUF_SIZE 128
volatile uint8_t rx_buf[PS2_BUF_SIZE];
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;

volatile uint8_t  bit_idx = 0;
volatile uint8_t  raw_byte = 0;
volatile uint8_t  parity_bit = 0;
volatile uint32_t last_clk_micros = 0;

// ISR Diagnostics & Performance Counters
volatile uint32_t isr_frames_received = 0;
volatile uint32_t isr_parity_errors   = 0;
volatile uint32_t isr_framing_errors  = 0;
volatile uint32_t isr_glitch_drops    = 0;
volatile uint32_t isr_buffer_overruns = 0;
volatile uint32_t i2c_bus_errors      = 0;

// OLED Text Tape Buffer (18 visible characters)
#define TAPE_MAX_LEN 18
char text_tape[TAPE_MAX_LEN + 1] = "";


// -----------------------------------------------------------------------------
// Core 1 to Core 0 Serial Queue
// -----------------------------------------------------------------------------
#define SERIAL_QUEUE_SIZE 32
char serial_queue[SERIAL_QUEUE_SIZE][224];
volatile uint8_t sq_head = 0;
volatile uint8_t sq_tail = 0;

volatile bool setup_complete = false;

void enqueue_serial(const char* str) {
  uint8_t next_head = (sq_head + 1) % SERIAL_QUEUE_SIZE;
  if (next_head != sq_tail) {
    strncpy((char*)serial_queue[sq_head], str, 223);
    serial_queue[sq_head][223] = '\0';
    arm_dmb();
    sq_head = next_head;
  }
}

// Cortex-M33 Data Memory Barrier
static inline void arm_dmb() {
  __asm__ volatile ("dmb 0xF" ::: "memory");
}

// -----------------------------------------------------------------------------
// Scan Code Set 2 Translation (Standard C++ Function)
// -----------------------------------------------------------------------------
char get_scancode_char(uint8_t code) {
  switch (code) {
    // Alphabet
    case 0x1C: return 'a'; case 0x32: return 'b'; case 0x21: return 'c';
    case 0x23: return 'd'; case 0x24: return 'e'; case 0x2B: return 'f';
    case 0x34: return 'g'; case 0x33: return 'h'; case 0x43: return 'i';
    case 0x3B: return 'j'; case 0x42: return 'k'; case 0x4B: return 'l';
    case 0x3A: return 'm'; case 0x31: return 'n'; case 0x44: return 'o';
    case 0x4D: return 'p'; case 0x15: return 'q'; case 0x2D: return 'r';
    case 0x1B: return 's'; case 0x2C: return 't'; case 0x3C: return 'u';
    case 0x2A: return 'v'; case 0x1D: return 'w'; case 0x22: return 'x';
    case 0x35: return 'y'; case 0x1A: return 'z';
    // Top-Row Numbers
    case 0x45: return '0'; case 0x16: return '1'; case 0x1E: return '2';
    case 0x26: return '3'; case 0x25: return '4'; case 0x2E: return '5';
    case 0x36: return '6'; case 0x3D: return '7'; case 0x3E: return '8';
    case 0x46: return '9';
    // Punctuation & Controls
    case 0x0E: return '`'; case 0x4E: return '-'; case 0x55: return '=';
    case 0x54: return '['; case 0x5B: return ']'; case 0x5D: return '\\';
    case 0x4C: return ';'; case 0x52: return '\''; case 0x41: return ',';
    case 0x49: return '.'; case 0x4A: return '/'; case 0x61: return '<';
    case 0x29: return ' '; case 0x5A: return '\n'; case 0x66: return '\b';
    case 0x0D: return '\t';
    default:   return 0;
  }
}

char get_shifted_symbol(char c) {
  switch (c) {
    case '1': return '!'; case '2': return '@'; case '3': return '#';
    case '4': return '$'; case '5': return '%'; case '6': return '^';
    case '7': return '&'; case '8': return '*'; case '9': return '(';
    case '0': return ')'; case '`': return '~'; case '-': return '_';
    case '=': return '+'; case '[': return '{'; case ']': return '}';
    case '\\': return '|'; case ';': return ':'; case '\'': return '\"';
    case ',': return '<'; case '.': return '>'; case '/': return '?';
    case '<': return '>';
    default:  return c;
  }
}

// -----------------------------------------------------------------------------
// Low-Level ISR (Clock Falling Edge)
// -----------------------------------------------------------------------------
void __attribute__((section(".time_critical.ps2_isr"))) ps2_clock_isr() {
  uint8_t bit_val = gpio_get(PS2_DATA_PIN) ? 1 : 0;
  uint32_t now = micros();
  uint32_t delta = now - last_clk_micros;

  if (delta < 25) {
    isr_glitch_drops++;
    return;
  }

  if (delta > 200) {
    bit_idx = 0;
  }

  switch (bit_idx) {
    case 0: // Start bit (0)
      if (bit_val == 0) {
        raw_byte = 0;
        bit_idx = 1;
      }
      last_clk_micros = now;
      break;

    case 1: case 2: case 3: case 4:
    case 5: case 6: case 7: case 8: // Data bits (LSB first)
      raw_byte |= (bit_val << (bit_idx - 1));
      bit_idx++;
      last_clk_micros = now;
      break;

    case 9: // Parity bit
      parity_bit = bit_val;
      bit_idx = 10;
      last_clk_micros = now;
      break;

    case 10: // Stop bit (1)
      if (bit_val == 1) {
        uint8_t p = parity_bit;
        for (int i = 0; i < 8; i++) {
          p ^= ((raw_byte >> i) & 0x01);
        }

        if (p == 1) { // Odd parity valid
          uint8_t next_head = (rx_head + 1) & (PS2_BUF_SIZE - 1);
          if (next_head != rx_tail) {
            rx_buf[rx_head] = raw_byte;
            arm_dmb();
            rx_head = next_head;
            isr_frames_received++;
          } else {
            isr_buffer_overruns++;
          }
        } else {
          isr_parity_errors++;
        }
      } else {
        isr_framing_errors++;
      }
      bit_idx = 0;
      last_clk_micros = now;
      break;

    default:
      bit_idx = 0;
      last_clk_micros = now;
      break;
  }
}

bool __attribute__((section(".time_critical.ps2_avail"))) ps2_available() {
  return (rx_head != rx_tail);
}

uint8_t __attribute__((section(".time_critical.ps2_read"))) ps2_read() {
  if (rx_head == rx_tail) return 0;
  uint8_t data = rx_buf[rx_tail];
  arm_dmb();
  rx_tail = (rx_tail + 1) & (PS2_BUF_SIZE - 1);
  return data;
}

// -----------------------------------------------------------------------------
// Unified Key Resolver
// -----------------------------------------------------------------------------
void resolve_key(uint8_t code, bool is_ext, bool shift, bool ctrl, bool caps, bool num,
                 char* desc_out, size_t desc_sz, char* ascii_out) {
  *ascii_out = 0;

  if (is_ext) {
    switch (code) {
      case 0x75: snprintf(desc_out, desc_sz, "ARROW_UP");    break;
      case 0x72: snprintf(desc_out, desc_sz, "ARROW_DOWN");  break;
      case 0x6B: snprintf(desc_out, desc_sz, "ARROW_LEFT");  break;
      case 0x74: snprintf(desc_out, desc_sz, "ARROW_RIGHT"); break;
      case 0x71: snprintf(desc_out, desc_sz, "DELETE");      break;
      case 0x70: snprintf(desc_out, desc_sz, "INSERT");      break;
      case 0x6C: snprintf(desc_out, desc_sz, "HOME");        break;
      case 0x69: snprintf(desc_out, desc_sz, "END");         break;
      case 0x7D: snprintf(desc_out, desc_sz, "PAGE_UP");     break;
      case 0x7A: snprintf(desc_out, desc_sz, "PAGE_DOWN");   break;
      case 0x5A: snprintf(desc_out, desc_sz, "KP_ENTER");    *ascii_out = '\n'; break;
      case 0x4A: snprintf(desc_out, desc_sz, "KP_DIVIDE");   *ascii_out = '/';  break;
      case 0x7C: snprintf(desc_out, desc_sz, "PRT_SCREEN");  break;
      case 0x7E: snprintf(desc_out, desc_sz, "CTRL_BREAK");  break;
      case 0x1F: snprintf(desc_out, desc_sz, "LEFT_GUI");    break;
      case 0x27: snprintf(desc_out, desc_sz, "RIGHT_GUI");   break;
      case 0x2F: snprintf(desc_out, desc_sz, "APPS_MENU");   break;
      case 0x14: snprintf(desc_out, desc_sz, "R-CTRL");      break;
      case 0x11: snprintf(desc_out, desc_sz, "R-ALT");       break;
      case 0x23: snprintf(desc_out, desc_sz, "MUTE");        break;
      case 0x32: snprintf(desc_out, desc_sz, "VOL_DOWN");    break;
      case 0x21: snprintf(desc_out, desc_sz, "VOL_UP");      break;
      case 0x34: snprintf(desc_out, desc_sz, "PLAY_PAUSE");  break;
      case 0x3B: snprintf(desc_out, desc_sz, "STOP");        break;
      case 0x4D: snprintf(desc_out, desc_sz, "NEXT_TRACK");  break;
      case 0x15: snprintf(desc_out, desc_sz, "PREV_TRACK");  break;
      case 0x2B: snprintf(desc_out, desc_sz, "CALCULATOR");  break;
      case 0x5E: snprintf(desc_out, desc_sz, "POWER");       break;
      case 0x5F: snprintf(desc_out, desc_sz, "SLEEP");       break;
      case 0x63: snprintf(desc_out, desc_sz, "WAKE");        break;
      default:   snprintf(desc_out, desc_sz, "EXT_0x%02X", code); break;
    }
    return;
  }

  // Modifiers & Locks
  if (code == 0x12)      { snprintf(desc_out, desc_sz, "L-SHIFT"); return; }
  if (code == 0x59)      { snprintf(desc_out, desc_sz, "R-SHIFT"); return; }
  if (code == 0x14)      { snprintf(desc_out, desc_sz, "L-CTRL");  return; }
  if (code == 0x11)      { snprintf(desc_out, desc_sz, "L-ALT");   return; }
  if (code == 0x58)      { snprintf(desc_out, desc_sz, "CAPS_LOCK"); return; }
  if (code == 0x77)      { snprintf(desc_out, desc_sz, "NUM_LOCK");  return; }
  if (code == 0x7E)      { snprintf(desc_out, desc_sz, "SCROLL_LOCK"); return; }
  if (code == 0x84)      { snprintf(desc_out, desc_sz, "SYSRQ"); return; }

  // Function Keys
  if (code == 0x76) { snprintf(desc_out, desc_sz, "ESC"); *ascii_out = 0x1B; return; }
  if (code == 0x05) { snprintf(desc_out, desc_sz, "F1");  return; }
  if (code == 0x06) { snprintf(desc_out, desc_sz, "F2");  return; }
  if (code == 0x04) { snprintf(desc_out, desc_sz, "F3");  return; }
  if (code == 0x0C) { snprintf(desc_out, desc_sz, "F4");  return; }
  if (code == 0x03) { snprintf(desc_out, desc_sz, "F5");  return; }
  if (code == 0x0B) { snprintf(desc_out, desc_sz, "F6");  return; }
  if (code == 0x83) { snprintf(desc_out, desc_sz, "F7");  return; }
  if (code == 0x0A) { snprintf(desc_out, desc_sz, "F8");  return; }
  if (code == 0x01) { snprintf(desc_out, desc_sz, "F9");  return; }
  if (code == 0x09) { snprintf(desc_out, desc_sz, "F10"); return; }
  if (code == 0x78) { snprintf(desc_out, desc_sz, "F11"); return; }
  if (code == 0x07) { snprintf(desc_out, desc_sz, "F12"); return; }

  // Keypad Math Operators
  if (code == 0x79) { snprintf(desc_out, desc_sz, "KP_PLUS");  *ascii_out = '+'; return; }
  if (code == 0x7B) { snprintf(desc_out, desc_sz, "KP_MINUS"); *ascii_out = '-'; return; }
  if (code == 0x7C) { snprintf(desc_out, desc_sz, "KP_MULT");  *ascii_out = '*'; return; }

  // Keypad Numeric vs Navigation
  if (code == 0x70 || code == 0x69 || code == 0x72 || code == 0x7A ||
      code == 0x6B || code == 0x73 || code == 0x74 || code == 0x6C ||
      code == 0x75 || code == 0x7D || code == 0x71) {
    bool effective_num = (num ^ shift);
    if (effective_num) {
      switch (code) {
        case 0x70: snprintf(desc_out, desc_sz, "KP_0"); *ascii_out = '0'; break;
        case 0x69: snprintf(desc_out, desc_sz, "KP_1"); *ascii_out = '1'; break;
        case 0x72: snprintf(desc_out, desc_sz, "KP_2"); *ascii_out = '2'; break;
        case 0x7A: snprintf(desc_out, desc_sz, "KP_3"); *ascii_out = '3'; break;
        case 0x6B: snprintf(desc_out, desc_sz, "KP_4"); *ascii_out = '4'; break;
        case 0x73: snprintf(desc_out, desc_sz, "KP_5"); *ascii_out = '5'; break;
        case 0x74: snprintf(desc_out, desc_sz, "KP_6"); *ascii_out = '6'; break;
        case 0x6C: snprintf(desc_out, desc_sz, "KP_7"); *ascii_out = '7'; break;
        case 0x75: snprintf(desc_out, desc_sz, "KP_8"); *ascii_out = '8'; break;
        case 0x7D: snprintf(desc_out, desc_sz, "KP_9"); *ascii_out = '9'; break;
        case 0x71: snprintf(desc_out, desc_sz, "KP_DECIMAL"); *ascii_out = '.'; break;
      }
    } else {
      switch (code) {
        case 0x75: snprintf(desc_out, desc_sz, "KP_UP");     break;
        case 0x72: snprintf(desc_out, desc_sz, "KP_DOWN");   break;
        case 0x6B: snprintf(desc_out, desc_sz, "KP_LEFT");   break;
        case 0x74: snprintf(desc_out, desc_sz, "KP_RIGHT");  break;
        case 0x71: snprintf(desc_out, desc_sz, "KP_DEL");    break;
        case 0x70: snprintf(desc_out, desc_sz, "KP_INS");    break;
        case 0x6C: snprintf(desc_out, desc_sz, "KP_HOME");   break;
        case 0x69: snprintf(desc_out, desc_sz, "KP_END");    break;
        case 0x7D: snprintf(desc_out, desc_sz, "KP_PGUP");   break;
        case 0x7A: snprintf(desc_out, desc_sz, "KP_PGDN");   break;
        case 0x73: snprintf(desc_out, desc_sz, "KP_CENTER"); break;
      }
    }
    return;
  }

  // Standard Alpha & Control Characters
  char ch = get_scancode_char(code);
  if (ch != 0) {
    if (ctrl) {
      if (ch >= 'a' && ch <= 'z') {
        *ascii_out = (char)(ch - 'a' + 1);
        snprintf(desc_out, desc_sz, "CTRL_%c", ch - 32);
      } else if (ch == '[')  { *ascii_out = 0x1B; snprintf(desc_out, desc_sz, "CTRL_["); }
      else if (ch == '\\') { *ascii_out = 0x1C; snprintf(desc_out, desc_sz, "CTRL_\\"); }
      else if (ch == ']')  { *ascii_out = 0x1D; snprintf(desc_out, desc_sz, "CTRL_]"); }
      else if (ch == ' ')  { *ascii_out = 0x00; snprintf(desc_out, desc_sz, "CTRL_SPACE"); }
      else if (ch == '\b') { *ascii_out = 0x7F; snprintf(desc_out, desc_sz, "CTRL_BS"); }
      else if (ch == '\t') { *ascii_out = '\t'; snprintf(desc_out, desc_sz, "CTRL_TAB"); }
      else if (ch == '\n') { *ascii_out = '\n'; snprintf(desc_out, desc_sz, "CTRL_ENTER"); }
      else {
        *ascii_out = ch;
        snprintf(desc_out, desc_sz, "CTRL_%c", ch);
      }
    } else {
      if (ch >= 'a' && ch <= 'z') {
        if (shift ^ caps) ch -= 32;
        *ascii_out = ch;
      } else if (shift) {
        *ascii_out = get_shifted_symbol(ch);
      } else {
        *ascii_out = ch;
      }

      if (ch == '\n')      snprintf(desc_out, desc_sz, "ENTER");
      else if (ch == '\b') snprintf(desc_out, desc_sz, "BACKSPACE");
      else if (ch == '\t') snprintf(desc_out, desc_sz, "TAB");
      else if (ch == ' ')  snprintf(desc_out, desc_sz, "SPACE");
      else                 snprintf(desc_out, desc_sz, "KEY_%c", (ch >= 'a' && ch <= 'z') ? (ch - 32) : ch);
    }
    return;
  }

  snprintf(desc_out, desc_sz, "RAW_0x%02X", code);
}

// -----------------------------------------------------------------------------
// Triple-Buffer Stage 1: Render into Draw Buffer & Commit to Staging
// -----------------------------------------------------------------------------
void render_frame_to_staging(uint8_t last_code, const char* event_type, const char* key_name,
                             char ascii_char, bool shift, bool ctrl, bool alt,
                             bool caps, bool num, bool scrl) {
  if (!oled_ready) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Convert raw code to binary string
  char bin_buf[9];
  for(int i = 7; i >= 0; i--) {
    bin_buf[7 - i] = (last_code & (1 << i)) ? '1' : '0';
  }
  bin_buf[8] = ' ';

  // === YELLOW SECTION (Rows 0 - 15) ===
  display.setTextSize(2);
  display.setCursor(16, 0); // Center the 8 characters (12px * 8 = 96px, 128-96=32, 32/2=16)
  display.print(bin_buf);

  // === BLUE SECTION (Rows 16 - 63) ===
  display.setTextSize(1);
  display.setCursor(0, 24);
  display.print("Action: ");
  display.print(event_type); // "MAKE" or "BREAK"

  display.setTextSize(2);
  display.setCursor(0, 40);
  display.print(key_name); // Human-readable key name

  // Commit Draw Buffer -> Staging Buffer (~1.1 us)
  memcpy(staging_buf, display.getBuffer(), OLED_BUF_SIZE);
  arm_dmb();
  staging_has_new_frame = true;
}

// -----------------------------------------------------------------------------
// Triple-Buffer Stage 2: Push Outbound Frame to SSD1306 Over 600 kHz I2C
// -----------------------------------------------------------------------------
void flush_staging_to_display() {
  if (!oled_ready || !staging_has_new_frame) return;

  // Latch Staging Buffer -> Outbound TX Buffer
  memcpy(tx_buf, staging_buf, OLED_BUF_SIZE);
  arm_dmb();
  staging_has_new_frame = false;

  // Assert Horizontal Addressing Mode (0x20 0x00) + Column/Page Ranges
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(0x00); // Command stream
  Wire.write(0x20); // Set Memory Addressing Mode
  Wire.write(0x00); // Horizontal Addressing Mode
  Wire.write(0x21); // Set column address
  Wire.write(0);    // Column start (0)
  Wire.write(127);  // Column end (127)
  Wire.write(0x22); // Set page address
  Wire.write(0);    // Page start (0)
  Wire.write(7);    // Page end (7)
  if (Wire.endTransmission() != 0) {
    i2c_bus_errors++;
    return;
  }

  // Burst TX buffer in 16-byte payloads (17 bytes total per transaction)
  for (int i = 0; i < OLED_BUF_SIZE; i += 16) {
    Wire.beginTransmission(OLED_ADDR);
    Wire.write(0x40); // Data prefix
    Wire.write(&tx_buf[i], 16);
    if (Wire.endTransmission() != 0) {
      i2c_bus_errors++;
      break;
    }
  }
}

// -----------------------------------------------------------------------------
// Telemetry Formatter
// -----------------------------------------------------------------------------
void print_telemetry(uint8_t code, const char* event_type, const char* prefix_type, 
                     const char* desc, char ascii_out, bool lshift, bool rshift, 
                     bool lctrl, bool rctrl, bool lalt, bool ralt, 
                     bool caps, bool num, bool scrl) {
  char line[192];
  char ascii_repr[20];

  if (ascii_out >= 32 && ascii_out <= 126) {
    snprintf(ascii_repr, sizeof(ascii_repr), "'%c' (0x%02X)", ascii_out, (uint8_t)ascii_out);
  } else if (ascii_out == '\n') {
    snprintf(ascii_repr, sizeof(ascii_repr), "\\n (0x0A)");
  } else if (ascii_out == '\t') {
    snprintf(ascii_repr, sizeof(ascii_repr), "\\t (0x09)");
  } else if (ascii_out == '\b') {
    snprintf(ascii_repr, sizeof(ascii_repr), "\\b (0x08)");
  } else if (ascii_out == 0x1B) {
    snprintf(ascii_repr, sizeof(ascii_repr), "ESC (0x1B)");
  } else if (ascii_out != 0) {
    snprintf(ascii_repr, sizeof(ascii_repr), "CTRL:0x%02X", (uint8_t)ascii_out);
  } else {
    snprintf(ascii_repr, sizeof(ascii_repr), "NONE");
  }

  char mods[32] = "";
  if (lshift || rshift) strcat(mods, "SHIFT ");
  if (lctrl || rctrl)   strcat(mods, "CTRL ");
  if (lalt || ralt)     strcat(mods, "ALT ");
  if (strlen(mods) == 0) strcpy(mods, "NONE ");

  char locks[32] = "";
  if (caps) strcat(locks, "CAPS ");
  if (num)  strcat(locks, "NUM ");
  if (scrl) strcat(locks, "SCRL ");
  if (strlen(locks) == 0) strcpy(locks, "NONE ");

  snprintf(line, sizeof(line), 
    "[0x%02X] %-5s | %-6s | Key: %-12s | Char: %-14s | Mods: %-8s | Locks: %s",
    code, event_type, prefix_type, desc, ascii_repr, mods, locks);

  enqueue_serial(line);
}

// -----------------------------------------------------------------------------
// Setup & Main Loop
// -----------------------------------------------------------------------------
void setup() {
  // 1. Boost core switching regulator to 1.20V
  vreg_set_voltage(VREG_VOLTAGE_1_20);
  delay(10);

  // 2. Lock System PLL to 266 MHz
  bool clock_set_ok = set_sys_clock_khz(TARGET_SYS_FREQ_KHZ, true);

  // 3. Initialize UART & I2C at 266 MHz
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  Wire.setClock(TARGET_I2C_FREQ_HZ); // 600 kHz Overclocked I2C

  // 4. Configure Pad Drivers AFTER Wire.begin() to prevent SDK register overwrite
  gpio_set_slew_rate(I2C_SDA_PIN, GPIO_SLEW_RATE_FAST);
  gpio_set_slew_rate(I2C_SCL_PIN, GPIO_SLEW_RATE_FAST);
  gpio_set_drive_strength(I2C_SDA_PIN, GPIO_DRIVE_STRENGTH_4MA);
  gpio_set_drive_strength(I2C_SCL_PIN, GPIO_DRIVE_STRENGTH_4MA);
  gpio_set_input_hysteresis_enabled(I2C_SDA_PIN, true);
  gpio_set_input_hysteresis_enabled(I2C_SCL_PIN, true);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[ERROR] SSD1306 Allocation Failed!");
    oled_ready = false;
  } else {
    oled_ready = true;

    // Overclock SSD1306 internal display refresh oscillator (~450 kHz)
    Wire.beginTransmission(OLED_ADDR);
    Wire.write(0x00);
    Wire.write(0xD5); // Set Display Clock Divide Ratio / Oscillator Frequency
    Wire.write(0xF0); // Max oscillator frequency, divide ratio = 1
    Wire.endTransmission();

    render_frame_to_staging(0x00, "INIT", "SYSTEM_READY", 0, false, false, false, false, true, false);
    flush_staging_to_display();
  }

  pinMode(PS2_CLK_PIN, INPUT_PULLUP);
  pinMode(PS2_DATA_PIN, INPUT_PULLUP);

  pinMode(LED_KEYPRESS_PIN, OUTPUT);
  pinMode(LED_HEARTBEAT_PIN, OUTPUT);
  digitalWrite(LED_KEYPRESS_PIN, LOW);
  digitalWrite(LED_HEARTBEAT_PIN, LOW);

  gpio_set_input_hysteresis_enabled(PS2_CLK_PIN, true);
  gpio_set_input_hysteresis_enabled(PS2_DATA_PIN, true);

  attachInterrupt(digitalPinToInterrupt(PS2_CLK_PIN), ps2_clock_isr, FALLING);

  Serial.println("==================================================================================");
  Serial.printf("   RP2350 (ARM Cortex-M33) System Clock: %lu MHz | PLL Lock: %s\n", 
                clock_get_hz(clk_sys) / 1000000, clock_set_ok ? "LOCKED" : "FAILED");
  Serial.printf("   I2C Bus: 600 kHz (4.7k Pull-ups) | Triple-Buffered SRAM Engine: 3072 Bytes Active\n");
  Serial.printf("   QSPI Flash Divider: /2 (%lu MHz Flash Clock)\n", 
                (clock_get_hz(clk_sys) / 1000000) / 2);
  Serial.println("   Format: [HEX] EVENT | PREFIX | Key: IDENTIFIER | Char: ASCII | MODS | LOCKS    ");
  Serial.println("==================================================================================");
  setup_complete = true;
}

void loop() {
  static bool is_break = false;
  static bool is_extended = false;
  static uint32_t last_byte_millis = 0;
  static uint32_t last_heartbeat_millis = 0;

  static bool lshift = false, rshift = false;
  static bool lctrl  = false, rctrl  = false;
  static bool lalt   = false, ralt   = false;

  static bool caps_lock = false, caps_lock_down = false;
  static bool num_lock  = true,  num_lock_down  = false;
  static bool scroll_lock = false, scroll_lock_down = false;

  static uint8_t pause_skip_count = 0;
  static uint32_t last_led_blink = 0;

  // LED Heartbeat (500ms toggle) on GPIO 25
  if (millis() - last_led_blink >= 500) {
    last_led_blink = millis();
    digitalWrite(LED_HEARTBEAT_PIN, !digitalRead(LED_HEARTBEAT_PIN));
  }



  // Heartbeat logging (every 10 seconds)
  if (millis() - last_heartbeat_millis > 10000) {
    last_heartbeat_millis = millis();
    char diag[224];
    snprintf(diag, sizeof(diag), "[HEARTBEAT] Freq: %lu MHz | I2C: 800 kHz | Frames: %lu | Overruns: %lu | Parity: %lu | Bus Errs: %lu",
             clock_get_hz(clk_sys) / 1000000, isr_frames_received, isr_buffer_overruns, isr_parity_errors, i2c_bus_errors);
    enqueue_serial(diag);
  }

  // Flush dangling prefixes on timeout (> 50 ms)
  if ((is_break || is_extended || pause_skip_count > 0) && (millis() - last_byte_millis > 50)) {
    is_break = false;
    is_extended = false;
    pause_skip_count = 0;
    enqueue_serial("[WARN] Frame timeout (>50ms). Prefixes reset.");
  }

  while (ps2_available()) {
    uint8_t code = ps2_read();
    last_byte_millis = millis();

    // BAT (Self-Test Passed)
    if (code == 0xAA) {
      lshift = rshift = lctrl = rctrl = lalt = ralt = false;
      is_break = is_extended = false;
      pause_skip_count = 0;
      enqueue_serial("[STATUS] 0xAA received -> Keyboard Self-Test (BAT) Passed. State reset.");
      render_frame_to_staging(code, "BAT", "SELF_TEST_OK", 0, false, false, false, caps_lock, num_lock, scroll_lock);
      continue;
    }

    if (code == 0xFA) { enqueue_serial("[STATUS] 0xFA -> Command Acknowledged (ACK)"); continue; }
    if (code == 0xFE) { enqueue_serial("[STATUS] 0xFE -> Resend Request from Keyboard"); continue; }
    if (code == 0xFC) { enqueue_serial("[ERROR]  0xFC -> Keyboard Self-Test Failed"); continue; }
    if (code == 0xFD) { enqueue_serial("[ERROR]  0xFD -> Keyboard Key Diagnostic Failure"); continue; }

    // 8-byte Pause/Break burst (E1 14 77 E1 F0 14 F0 77)
    if (pause_skip_count > 0) {
      pause_skip_count--;
      continue;
    }
    if (code == 0xE1) {
      pause_skip_count = 7;
      print_telemetry(0xE1, "MAKE", "SPECIAL", "PAUSE/BREAK", 0, 
                      lshift, rshift, lctrl, rctrl, lalt, ralt, caps_lock, num_lock, scroll_lock);
      render_frame_to_staging(0xE1, "MAKE", "PAUSE/BREAK", 0, (lshift || rshift), (lctrl || rctrl), (lalt || ralt), caps_lock, num_lock, scroll_lock);
      continue;
    }

    // Prefixes
    if (code == 0xF0) {
      is_break = true;
      continue;
    } else if (code == 0xE0) {
      is_extended = true;
      continue;
    }

    // Filter Fake Shift bytes (E0 12 / E0 59)
    if (is_extended && (code == 0x12 || code == 0x59)) {
      is_break = false;
      is_extended = false;
      continue;
    }

    const char* prefix_str = is_extended ? "EXT-E0" : "STD";
    bool shift_active = (lshift || rshift);
    bool ctrl_active  = (lctrl  || rctrl);
    bool alt_active   = (lalt   || ralt);

    char key_name[24];
    char ascii_char = 0;

    resolve_key(code, is_extended, shift_active, ctrl_active, caps_lock, num_lock,
                key_name, sizeof(key_name), &ascii_char);

    // ==========================================
    // KEY RELEASE (BREAK)
    // ==========================================
    if (is_break) {
      digitalWrite(LED_KEYPRESS_PIN, LOW); // Turn off LED on key release
      if (!is_extended) {
        if (code == 0x12) lshift = false;
        if (code == 0x59) rshift = false;
        if (code == 0x14) lctrl  = false;
        if (code == 0x11) lalt   = false;
        if (code == 0x58) caps_lock_down = false;
        if (code == 0x77) num_lock_down  = false;
        if (code == 0x7E) scroll_lock_down = false;
      } else {
        if (code == 0x14) rctrl  = false;
        if (code == 0x11) ralt   = false;
      }

      print_telemetry(code, "BREAK", prefix_str, key_name, 0,
                      lshift, rshift, lctrl, rctrl, lalt, ralt, caps_lock, num_lock, scroll_lock);

      render_frame_to_staging(code, "BREAK", key_name, 0, (lshift || rshift), (lctrl || rctrl), (lalt || ralt),
                              caps_lock, num_lock, scroll_lock);

      is_break = false;
      is_extended = false;
      continue;
    }

    // ==========================================
    // KEY PRESS (MAKE)
    // ==========================================
    if (!is_extended) {
      if (code == 0x12) lshift = true;
      else if (code == 0x59) rshift = true;
      else if (code == 0x14) lctrl  = true;
      else if (code == 0x11) lalt   = true;
      else if (code == 0x58) {
        if (!caps_lock_down) { caps_lock = !caps_lock; caps_lock_down = true; }
      } else if (code == 0x77) {
        if (!num_lock_down) { num_lock = !num_lock; num_lock_down = true; }
      } else if (code == 0x7E) {
        if (!scroll_lock_down) { scroll_lock = !scroll_lock; scroll_lock_down = true; }
      }
    } else {
      if (code == 0x14) rctrl = true;
      else if (code == 0x11) ralt = true;
    }

    // Update live typing tape on OLED
    if (ascii_char >= 32 && ascii_char <= 126) {
      size_t len = strlen(text_tape);
      if (len < TAPE_MAX_LEN) {
        text_tape[len] = ascii_char;
        text_tape[len + 1] = '\0';
      } else {
        memmove(text_tape, text_tape + 1, TAPE_MAX_LEN - 1);
        text_tape[TAPE_MAX_LEN - 1] = ascii_char;
        text_tape[TAPE_MAX_LEN] = '\0';
      }
    } else if (code == 0x66 && !is_extended) { // Backspace
      size_t len = strlen(text_tape);
      if (len > 0) text_tape[len - 1] = '\0';
    } else if (ascii_char == '\n') { // Enter clears live typing line
      text_tape[0] = '\0';
    }

    print_telemetry(code, "MAKE", prefix_str, key_name, ascii_char,
                    lshift, rshift, lctrl, rctrl, lalt, ralt, caps_lock, num_lock, scroll_lock);

    render_frame_to_staging(code, "MAKE", key_name, ascii_char, (lshift || rshift), (lctrl || rctrl), (lalt || ralt),
                            caps_lock, num_lock, scroll_lock);

    // Turn on Keypress LED while key is held
    digitalWrite(LED_KEYPRESS_PIN, HIGH);

    is_extended = false;
  }


}
// -----------------------------------------------------------------------------
// CORE 1: Serial Monitor & OLED I2C Controller
// -----------------------------------------------------------------------------
void setup1() {
  // Wait for Core 0 to finish hardware init
  while (!setup_complete) {
    delay(1);
  }
}

void loop1() {
  static uint32_t last_oled_push_millis = 0;

  // 1. Drain the Serial Queue
  while (sq_head != sq_tail) {
    Serial.println((char*)serial_queue[sq_tail]);
    arm_dmb();
    sq_tail = (sq_tail + 1) % SERIAL_QUEUE_SIZE;
  }

  // 2. Flush Staging Buffer to OLED over I2C
  if (staging_has_new_frame && (millis() - last_oled_push_millis >= OLED_REFRESH_INTERVAL_MS)) {
    last_oled_push_millis = millis();
    flush_staging_to_display();
  }
}
