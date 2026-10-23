// MCP2515 CAN controller - Wokwi custom chip (SPI mode 0)
// Emulates the register map and SPI commands used by the mcp_can library.
// Modes: CONFIG, NORMAL, LOOPBACK, LISTENONLY, SLEEP.
// LOOPBACK: sent frames are received back. NORMAL: sent frames are logged
// to the Chips Console (there is no other node on the simulated bus).
#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define REG_CANSTAT 0x0E
#define REG_CANCTRL 0x0F
#define REG_CANINTE 0x2B
#define REG_CANINTF 0x2C
#define REG_EFLG    0x2D

#define MODE_NORMAL   0x00
#define MODE_SLEEP    0x20
#define MODE_LOOPBACK 0x40
#define MODE_LISTEN   0x60
#define MODE_CONFIG   0x80

enum { ST_CMD, ST_READ_ADDR, ST_READ, ST_WRITE_ADDR, ST_WRITE,
       ST_BM_ADDR, ST_BM_MASK, ST_BM_DATA, ST_STATUS, ST_RXSTATUS, ST_IGNORE };

typedef struct {
  pin_t cs, intr;
  spi_dev_t spi;
  uint8_t buf[1];
  bool selected;
  int state;
  uint8_t addr, bm_mask;
  uint8_t clear_rx_flag;   // set by READ RX BUFFER, applied when CS rises
  uint8_t reg[128];
} chip_t;

static const uint8_t TXCTRL[3] = {0x30, 0x40, 0x50};
static const uint8_t RXCTRL[2] = {0x60, 0x70};

static uint8_t mode(chip_t *c) { return c->reg[REG_CANSTAT] & 0xE0; }

static void update_int(chip_t *c) {
  pin_write(c->intr, (c->reg[REG_CANINTF] & c->reg[REG_CANINTE]) ? LOW : HIGH);
}

static void chip_reset(chip_t *c) {
  memset(c->reg, 0, sizeof(c->reg));
  c->reg[REG_CANCTRL] = 0x87;
  c->reg[REG_CANSTAT] = MODE_CONFIG;
  update_int(c);
}

static void receive_frame(chip_t *c, const uint8_t *f /* SIDH..D7, 13 bytes */) {
  int n;
  if (!(c->reg[REG_CANINTF] & 0x01)) n = 0;
  else if (!(c->reg[REG_CANINTF] & 0x02)) n = 1;
  else { c->reg[REG_EFLG] |= 0x40; printf("[MCP2515] RX overflow, frame dropped\n"); return; }
  uint8_t base = RXCTRL[n];
  memcpy(&c->reg[base + 1], f, 13);
  c->reg[base + 5] &= 0x4F;                         // keep RTR + DLC
  bool ext = f[1] & 0x08, rtr = f[4] & 0x40;
  c->reg[base] = (c->reg[base] & 0xF7) | ((rtr && !ext) ? 0x08 : 0);
  c->reg[REG_CANINTF] |= (n == 0) ? 0x01 : 0x02;
}

static void transmit(chip_t *c, int n) {
  uint8_t base = TXCTRL[n];
  const uint8_t *f = &c->reg[base + 1];
  bool ext = f[1] & 0x08;
  uint32_t id = ((uint32_t)f[0] << 3) | (f[1] >> 5);
  if (ext) id = (id << 18) | ((uint32_t)(f[1] & 0x03) << 16) | ((uint32_t)f[2] << 8) | f[3];
  uint8_t dlc = f[4] & 0x0F; if (dlc > 8) dlc = 8;
  printf("[MCP2515] TX%d id=0x%lX%s dlc=%d data:", n, (unsigned long)id, ext ? " (ext)" : "", dlc);
  for (int i = 0; i < dlc; i++) printf(" %02X", f[5 + i]);
  printf("%s\n", mode(c) == MODE_LOOPBACK ? "  -> looped back" : "");
  if (mode(c) == MODE_LOOPBACK) receive_frame(c, f);
  c->reg[base] &= ~0x08;                            // TXREQ done
  c->reg[REG_CANINTF] |= (uint8_t)(0x04 << n);      // TXnIF
}

static void process_tx(chip_t *c) {
  uint8_t m = mode(c);
  if (m != MODE_NORMAL && m != MODE_LOOPBACK) return;
  for (int n = 0; n < 3; n++)
    if (c->reg[TXCTRL[n]] & 0x08) transmit(c, n);
}

static void write_reg(chip_t *c, uint8_t a, uint8_t v) {
  a &= 0x7F;
  uint8_t lo = a & 0x0F;
  if (lo == 0x0E) return;                           // CANSTAT is read-only
  if (lo == 0x0F) {                                 // CANCTRL (mirrored)
    c->reg[REG_CANCTRL] = v;
    if (v & 0x10) for (int n = 0; n < 3; n++) c->reg[TXCTRL[n]] &= ~0x08;  // ABAT
    c->reg[REG_CANSTAT] = (c->reg[REG_CANSTAT] & 0x1F) | (v & 0xE0);
  } else {
    c->reg[a] = v;
  }
  process_tx(c);
  update_int(c);
}

static uint8_t read_reg(chip_t *c, uint8_t a) {
  a &= 0x7F;
  if ((a & 0x0F) == 0x0E) return c->reg[REG_CANSTAT];
  if ((a & 0x0F) == 0x0F) return c->reg[REG_CANCTRL];
  return c->reg[a];
}

static uint8_t read_status(chip_t *c) {
  uint8_t f = c->reg[REG_CANINTF];
  return (f & 0x03) | ((c->reg[0x30] & 0x08) >> 1) | ((f & 0x04) << 1) |
         ((c->reg[0x40] & 0x08) << 1) | ((f & 0x08) << 2) |
         ((c->reg[0x50] & 0x08) << 3) | ((f & 0x10) << 3);
}

static uint8_t rx_status(chip_t *c) {
  uint8_t f = c->reg[REG_CANINTF] & 0x03, s = (uint8_t)(f << 6);
  uint8_t base = (f & 0x01) ? 0x60 : (f & 0x02) ? 0x70 : 0;
  if (base) {
    bool ext = c->reg[base + 2] & 0x08, rtr = (c->reg[base + 5] & 0x40) != 0;
    s |= (ext ? 0x10 : 0) | (rtr ? 0x08 : 0);
  }
  return s;
}

static void spi_byte(chip_t *c, uint8_t b) {
  uint8_t out = 0xFF;
  switch (c->state) {
    case ST_CMD:
      if (b == 0xC0) { chip_reset(c); c->state = ST_IGNORE; }
      else if (b == 0x03) c->state = ST_READ_ADDR;
      else if (b == 0x02) c->state = ST_WRITE_ADDR;
      else if (b == 0x05) c->state = ST_BM_ADDR;
      else if (b == 0xA0) { c->state = ST_STATUS; out = read_status(c); }
      else if (b == 0xB0) { c->state = ST_RXSTATUS; out = rx_status(c); }
      else if ((b & 0xF8) == 0x40 && (b & 0x07) <= 5) {          // LOAD TX BUFFER
        uint8_t k = b & 0x07;
        c->addr = TXCTRL[k >> 1] + ((k & 1) ? 6 : 1);
        c->state = ST_WRITE;
      } else if ((b & 0xF8) == 0x80) {                            // RTS
        for (int n = 0; n < 3; n++) if (b & (1 << n)) c->reg[TXCTRL[n]] |= 0x08;
        process_tx(c); update_int(c);
        c->state = ST_IGNORE;
      } else if ((b & 0xF9) == 0x90) {                            // READ RX BUFFER
        uint8_t n = (b >> 2) & 1;
        c->addr = RXCTRL[n] + ((b & 0x02) ? 6 : 1);
        c->clear_rx_flag = (n == 0) ? 0x01 : 0x02;
        out = read_reg(c, c->addr); c->addr = (c->addr + 1) & 0x7F;
        c->state = ST_READ;
      } else c->state = ST_IGNORE;
      break;
    case ST_READ_ADDR:
      c->addr = b & 0x7F;
      out = read_reg(c, c->addr); c->addr = (c->addr + 1) & 0x7F;
      c->state = ST_READ;
      break;
    case ST_READ:
      out = read_reg(c, c->addr); c->addr = (c->addr + 1) & 0x7F;
      break;
    case ST_WRITE_ADDR: c->addr = b & 0x7F; c->state = ST_WRITE; break;
    case ST_WRITE:  write_reg(c, c->addr, b); c->addr = (c->addr + 1) & 0x7F; break;
    case ST_BM_ADDR: c->addr = b & 0x7F; c->state = ST_BM_MASK; break;
    case ST_BM_MASK: c->bm_mask = b; c->state = ST_BM_DATA; break;
    case ST_BM_DATA:
      write_reg(c, c->addr, (read_reg(c, c->addr) & ~c->bm_mask) | (b & c->bm_mask));
      c->state = ST_IGNORE;
      break;
    case ST_STATUS: out = read_status(c); break;
    case ST_RXSTATUS: out = rx_status(c); break;
    default: break;
  }
  c->buf[0] = out;
}

static void on_spi_done(void *user_data, uint8_t *buffer, uint32_t count) {
  chip_t *c = (chip_t *)user_data;
  if (!count || !c->selected) return;
  spi_byte(c, buffer[0]);
  if (pin_read(c->cs) == LOW) spi_start(c->spi, c->buf, 1);
}

static void on_cs_change(void *user_data, pin_t pin, uint32_t value) {
  (void)pin;
  chip_t *c = (chip_t *)user_data;
  if (value == LOW) {
    c->selected = true;
    c->state = ST_CMD;
    c->clear_rx_flag = 0;
    c->buf[0] = 0xFF;
    spi_start(c->spi, c->buf, 1);
  } else {
    c->selected = false;
    spi_stop(c->spi);
    if (c->clear_rx_flag) {
      c->reg[REG_CANINTF] &= ~c->clear_rx_flag;
      c->clear_rx_flag = 0;
      update_int(c);
    }
  }
}

void chip_init(void) {
  chip_t *c = calloc(1, sizeof(chip_t));
  c->cs = pin_init("CS", INPUT_PULLUP);
  c->intr = pin_init("INT", OUTPUT_HIGH);
  const spi_config_t spi_cfg = {
    .sck = pin_init("SCK", INPUT),
    .mosi = pin_init("SI", INPUT),
    .miso = pin_init("SO", INPUT),
    .mode = 0,
    .done = on_spi_done,
    .user_data = c,
  };
  c->spi = spi_init(&spi_cfg);
  const pin_watch_config_t cs_watch = { .edge = BOTH, .pin_change = on_cs_change, .user_data = c };
  pin_watch(c->cs, &cs_watch);
  chip_reset(c);
  printf("[MCP2515] ready (16 MHz crystal assumed)\n");
}
