// MCP2515 CAN controller - Wokwi custom chip (SPI mode 0)
// Emulates the register map and SPI commands used by the mcp_can library.
// Modes: CONFIG, NORMAL, LOOPBACK, LISTENONLY, SLEEP.
// LOOPBACK: sent frames are received back. NORMAL: sent frames are logged
// to the Chips Console (there is no other node on the simulated bus).
#include "wokwi-api.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Global constants
// ---------------------------------------------------------------------------

// Register addresses
static const uint8_t REG_CANSTAT = 0x0E;
static const uint8_t REG_CANCTRL = 0x0F;
static const uint8_t REG_CANINTE = 0x2B;
static const uint8_t REG_CANINTF = 0x2C;
static const uint8_t REG_EFLG    = 0x2D;

// Operating modes (OPMOD bits in CANSTAT / REQOP bits in CANCTRL)
static const uint8_t MODE_NORMAL   = 0x00;
static const uint8_t MODE_SLEEP    = 0x20;
static const uint8_t MODE_LOOPBACK = 0x40;
static const uint8_t MODE_LISTEN   = 0x60;
static const uint8_t MODE_CONFIG   = 0x80;

// SPI instruction opcodes
static const uint8_t CMD_RESET       = 0xC0;
static const uint8_t CMD_READ        = 0x03;
static const uint8_t CMD_WRITE       = 0x02;
static const uint8_t CMD_BIT_MODIFY  = 0x05;
static const uint8_t CMD_READ_STATUS = 0xA0;
static const uint8_t CMD_RX_STATUS   = 0xB0;
static const uint8_t CMD_LOAD_TX     = 0x40; // base; mask with CMD_LOAD_TX_MASK
static const uint8_t CMD_RTS         = 0x80; // base; mask with CMD_RTS_MASK
static const uint8_t CMD_READ_RX     = 0x90; // base; mask with CMD_READ_RX_MASK

static const uint8_t CMD_LOAD_TX_MASK = 0xF8;
static const uint8_t CMD_RTS_MASK     = 0xF8;
static const uint8_t CMD_READ_RX_MASK = 0xF9;

// Bit masks and flags
static const uint8_t MODE_MASK       = 0xE0;
static const uint8_t ADDR_MASK       = 0x7F;
static const uint8_t NIBBLE_MASK     = 0x0F;
static const uint8_t TXREQ_BIT       = 0x08; // TXBnCTRL.TXREQ
static const uint8_t RXRTR_BIT       = 0x08; // RXBnCTRL.RXRTR
static const uint8_t ABAT_BIT        = 0x10; // CANCTRL.ABAT
static const uint8_t EXIDE_BIT       = 0x08; // TXBnSIDL / RXBnSIDL
static const uint8_t RTR_BIT         = 0x40; // TXBnDLC / RXBnDLC
static const uint8_t RX0IF_BIT       = 0x01;
static const uint8_t RX1IF_BIT       = 0x02;
static const uint8_t TX0IF_BIT       = 0x04;
static const uint8_t TX1IF_BIT       = 0x08;
static const uint8_t TX2IF_BIT       = 0x10;
static const uint8_t RX_OVERFLOW_BIT = 0x40; // EFLG.RX1OVR
static const uint8_t RX_STATUS_EXT   = 0x10; // RX STATUS response bit
static const uint8_t RX_STATUS_RTR   = 0x08; // RX STATUS response bit
static const uint8_t SPI_IDLE_BYTE   = 0xFF;

// Reset / frame layout
static const uint8_t CANCTRL_RESET_VALUE = 0x87;
static const uint8_t FRAME_LEN           = 13; // SIDH..D7
static const uint8_t DLC_MAX             = 8;

// TXBnCTRL / RXBnCTRL base addresses (n = buffer index)
static const uint8_t TXCTRL[3] = { 0x30, 0x40, 0x50 };
static const uint8_t RXCTRL[2] = { 0x60, 0x70 };

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

enum {
    ST_CMD,
    ST_READ_ADDR,
    ST_READ,
    ST_WRITE_ADDR,
    ST_WRITE,
    ST_BM_ADDR,
    ST_BM_MASK,
    ST_BM_DATA,
    ST_STATUS,
    ST_RXSTATUS,
    ST_IGNORE
};

typedef struct {
    pin_t cs, intr;
    spi_dev_t spi;
    uint8_t buf[1];
    bool selected;
    int state;
    uint8_t addr, bm_mask;
    uint8_t clear_rx_flag; // set by READ RX BUFFER, applied when CS rises
    uint8_t reg[128];
} chip_t;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static uint8_t mode(chip_t *c) {
    return c->reg[REG_CANSTAT] & MODE_MASK;
}

static void update_int(chip_t *c) {
    pin_write(c->intr, (c->reg[REG_CANINTF] & c->reg[REG_CANINTE]) ? LOW : HIGH);
}

static void chip_reset(chip_t *c) {
    memset(c->reg, 0, sizeof(c->reg));
    c->reg[REG_CANCTRL] = CANCTRL_RESET_VALUE;
    c->reg[REG_CANSTAT] = MODE_CONFIG;
    update_int(c);
}

static void receive_frame(chip_t *c, const uint8_t *f /* SIDH..D7, 13 bytes */) {
    int n;
    if (!(c->reg[REG_CANINTF] & RX0IF_BIT))
        n = 0;
    else if (!(c->reg[REG_CANINTF] & RX1IF_BIT))
        n = 1;
    else {
        c->reg[REG_EFLG] |= RX_OVERFLOW_BIT;
        printf("[MCP2515] RX overflow, frame dropped\n");
        return;
    }
    uint8_t base = RXCTRL[n];
    memcpy(&c->reg[base + 1], f, FRAME_LEN);
    c->reg[base + 5] &= 0x4F; // keep RTR + DLC
    bool ext = f[1] & EXIDE_BIT, rtr = f[4] & RTR_BIT;
    c->reg[base] = (c->reg[base] & 0xF7) | ((rtr && !ext) ? RXRTR_BIT : 0);
    c->reg[REG_CANINTF] |= (n == 0) ? RX0IF_BIT : RX1IF_BIT;
}

static void transmit(chip_t *c, int n) {
    uint8_t base = TXCTRL[n];
    const uint8_t *f = &c->reg[base + 1];
    bool ext = f[1] & EXIDE_BIT;
    uint32_t id = ((uint32_t)f[0] << 3) | (f[1] >> 5);
    if (ext)
        id = (id << 18) | ((uint32_t)(f[1] & 0x03) << 16) | ((uint32_t)f[2] << 8) | f[3];
    uint8_t dlc = f[4] & NIBBLE_MASK;
    if (dlc > DLC_MAX)
        dlc = DLC_MAX;
    printf("[MCP2515] TX%d id=0x%lX%s dlc=%d data:", n, (unsigned long)id,
        ext ? " (ext)" : "", dlc);
    for (int i = 0; i < dlc; i++)
        printf(" %02X", f[5 + i]);
    printf("%s\n", mode(c) == MODE_LOOPBACK ? "  -> looped back" : "");
    if (mode(c) == MODE_LOOPBACK)
        receive_frame(c, f);
    c->reg[base] &= ~TXREQ_BIT;                    // TXREQ done
    c->reg[REG_CANINTF] |= (uint8_t)(TX0IF_BIT << n); // TXnIF
}

static void process_tx(chip_t *c) {
    uint8_t m = mode(c);
    if (m != MODE_NORMAL && m != MODE_LOOPBACK)
        return;
    for (int n = 0; n < 3; n++)
        if (c->reg[TXCTRL[n]] & TXREQ_BIT)
            transmit(c, n);
}

static void write_reg(chip_t *c, uint8_t a, uint8_t v) {
    a &= ADDR_MASK;
    uint8_t lo = a & NIBBLE_MASK;
    if (lo == REG_CANSTAT)
        return;         // CANSTAT is read-only
    if (lo == REG_CANCTRL) {   // CANCTRL (mirrored)
        c->reg[REG_CANCTRL] = v;
        if (v & ABAT_BIT)
            for (int n = 0; n < 3; n++)
                c->reg[TXCTRL[n]] &= ~TXREQ_BIT; // ABAT
        c->reg[REG_CANSTAT] = (c->reg[REG_CANSTAT] & ~MODE_MASK) | (v & MODE_MASK);
    } else {
        c->reg[a] = v;
    }
    process_tx(c);
    update_int(c);
}

static uint8_t read_reg(chip_t *c, uint8_t a) {
    a &= ADDR_MASK;
    if ((a & NIBBLE_MASK) == REG_CANSTAT)
        return c->reg[REG_CANSTAT];
    if ((a & NIBBLE_MASK) == REG_CANCTRL)
        return c->reg[REG_CANCTRL];
    return c->reg[a];
}

static uint8_t read_status(chip_t *c) {
    uint8_t f = c->reg[REG_CANINTF];
    return (f & (RX0IF_BIT | RX1IF_BIT))
        | ((c->reg[TXCTRL[0]] & TXREQ_BIT) >> 1)
        | ((f & TX0IF_BIT) << 1)
        | ((c->reg[TXCTRL[1]] & TXREQ_BIT) << 1)
        | ((f & TX1IF_BIT) << 2)
        | ((c->reg[TXCTRL[2]] & TXREQ_BIT) << 3)
        | ((f & TX2IF_BIT) << 3);
}

static uint8_t rx_status(chip_t *c) {
    uint8_t f = c->reg[REG_CANINTF] & (RX0IF_BIT | RX1IF_BIT);
    uint8_t s = (uint8_t)(f << 6);
    uint8_t base = (f & RX0IF_BIT) ? RXCTRL[0] : (f & RX1IF_BIT) ? RXCTRL[1] : 0;
    if (base) {
        bool ext = c->reg[base + 2] & EXIDE_BIT;
        bool rtr = (c->reg[base + 5] & RTR_BIT) != 0;
        s |= (ext ? RX_STATUS_EXT : 0) | (rtr ? RX_STATUS_RTR : 0);
    }
    return s;
}

static void spi_byte(chip_t *c, uint8_t b) {
    uint8_t out = SPI_IDLE_BYTE;
    switch (c->state) {
        case ST_CMD:
            if (b == CMD_RESET) {
                chip_reset(c);
                c->state = ST_IGNORE;
            } else if (b == CMD_READ) {
                c->state = ST_READ_ADDR;
            } else if (b == CMD_WRITE) {
                c->state = ST_WRITE_ADDR;
            } else if (b == CMD_BIT_MODIFY) {
                c->state = ST_BM_ADDR;
            } else if (b == CMD_READ_STATUS) {
                c->state = ST_STATUS;
                out = read_status(c);
            } else if (b == CMD_RX_STATUS) {
                c->state = ST_RXSTATUS;
                out = rx_status(c);
            } else if ((b & CMD_LOAD_TX_MASK) == CMD_LOAD_TX && (b & 0x07) <= 5) { // LOAD TX BUFFER
                uint8_t k = b & 0x07;
                c->addr = TXCTRL[k >> 1] + ((k & 1) ? 6 : 1);
                c->state = ST_WRITE;
            } else if ((b & CMD_RTS_MASK) == CMD_RTS) { // RTS
                for (int n = 0; n < 3; n++)
                    if (b & (1 << n))
                        c->reg[TXCTRL[n]] |= TXREQ_BIT;
                process_tx(c);
                update_int(c);
                c->state = ST_IGNORE;
            } else if ((b & CMD_READ_RX_MASK) == CMD_READ_RX) { // READ RX BUFFER
                uint8_t n = (b >> 2) & 1;
                c->addr = RXCTRL[n] + ((b & 0x02) ? 6 : 1);
                c->clear_rx_flag = (n == 0) ? RX0IF_BIT : RX1IF_BIT;
                out = read_reg(c, c->addr);
                c->addr = (c->addr + 1) & ADDR_MASK;
                c->state = ST_READ;
            } else {
                c->state = ST_IGNORE;
            }
            break;
        case ST_READ_ADDR:
            c->addr = b & ADDR_MASK;
            out = read_reg(c, c->addr);
            c->addr = (c->addr + 1) & ADDR_MASK;
            c->state = ST_READ;
            break;
        case ST_READ:
            out = read_reg(c, c->addr);
            c->addr = (c->addr + 1) & ADDR_MASK;
            break;
        case ST_WRITE_ADDR:
            c->addr = b & ADDR_MASK;
            c->state = ST_WRITE;
            break;
        case ST_WRITE:
            write_reg(c, c->addr, b);
            c->addr = (c->addr + 1) & ADDR_MASK;
            break;
        case ST_BM_ADDR:
            c->addr = b & ADDR_MASK;
            c->state = ST_BM_MASK;
            break;
        case ST_BM_MASK:
            c->bm_mask = b;
            c->state = ST_BM_DATA;
            break;
        case ST_BM_DATA:
            write_reg(c, c->addr,
                (read_reg(c, c->addr) & ~c->bm_mask) | (b & c->bm_mask));
            c->state = ST_IGNORE;
            break;
        case ST_STATUS:
            out = read_status(c);
            break;
        case ST_RXSTATUS:
            out = rx_status(c);
            break;
        default:
            break;
    }
    c->buf[0] = out;
}

static void on_spi_done(void *user_data, uint8_t *buffer, uint32_t count) {
    chip_t *c = (chip_t *)user_data;
    if (!count || !c->selected)
        return;
    spi_byte(c, buffer[0]);
    if (pin_read(c->cs) == LOW)
        spi_start(c->spi, c->buf, 1);
}

static void on_cs_change(void *user_data, pin_t pin, uint32_t value) {
    (void)pin;
    chip_t *c = (chip_t *)user_data;
    if (value == LOW) {
        c->selected = true;
        c->state = ST_CMD;
        c->clear_rx_flag = 0;
        c->buf[0] = SPI_IDLE_BYTE;
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
    const pin_watch_config_t cs_watch = {
        .edge = BOTH,
        .pin_change = on_cs_change,
        .user_data = c
    };
    pin_watch(c->cs, &cs_watch);
    chip_reset(c);
    printf("[MCP2515] ready (16 MHz crystal assumed)\n");
}
