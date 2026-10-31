/*
 * Gauntlet packet v1 - header-only reference codec for the ESP32s (sender and round-screen receiver).
 * See protocol/PACKET.md. Little-endian, packed, 40 bytes. C99, no dependencies beyond <stdint.h>.
 *
 *   gauntlet_packet_t pkt = {0};
 *   pkt.mode = 0; pkt.seq = seq++; pkt.millis = millis();
 *   pkt.q[0] = (int16_t)(qw * 32767.0f); ...
 *   pkt.flex[0] = adc_thumb; ...
 *   memset(pkt.servo, 255, 8); pkt.servo[0] = 20; ...
 *   pkt.flags = GAUNTLET_FLAG_HAND_IMU | GAUNTLET_FLAG_BODY_IMU | GAUNTLET_FLAG_FLEX;
 *   gauntlet_packet_finalize(&pkt);             // writes magic, version, reserved and crc
 *   uart_write(&pkt, sizeof pkt);
 *
 * Receiver: feed bytes to gauntlet_parser_push() and act on every returned 1.
 */
#ifndef GAUNTLET_PACKET_H
#define GAUNTLET_PACKET_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GAUNTLET_PACKET_SIZE 40
#define GAUNTLET_MAGIC0 0x47 /* 'G' */
#define GAUNTLET_MAGIC1 0x50 /* 'P' */
#define GAUNTLET_VERSION 1
#define GAUNTLET_SERVO_ABSENT 255

#define GAUNTLET_FLAG_HAND_IMU (1u << 0)
#define GAUNTLET_FLAG_BODY_IMU (1u << 1)
#define GAUNTLET_FLAG_FLEX (1u << 2)
#define GAUNTLET_FLAG_EVENT (1u << 3)

enum gauntlet_mode {
  GAUNTLET_MODE_IDLE = 0,
  GAUNTLET_MODE_HAND_OPEN = 1,
  GAUNTLET_MODE_FOREARM_OPEN = 2,
  GAUNTLET_MODE_ALL_OPEN = 3,
  GAUNTLET_MODE_REPULSOR = 4,
  GAUNTLET_MODE_CALIBRATION = 5
};

typedef struct __attribute__((packed)) {
  uint8_t magic[2];  /* 'G' 'P' */
  uint8_t version;   /* 1 */
  uint8_t mode;      /* firmware mode id */
  uint16_t seq;      /* increments every packet, wraps */
  uint32_t millis;   /* sender uptime, ms */
  int16_t q[4];      /* hand orientation relative to the body sensor, w x y z, Q15 */
  uint16_t flex[5];  /* raw ADC, thumb index middle ring pinky */
  uint8_t servo[8];  /* commanded degrees 0..180, 255 = channel not present */
  uint8_t flags;     /* GAUNTLET_FLAG_* */
  uint8_t reserved;  /* 0 */
  uint16_t crc16;    /* CRC-16/CCITT-FALSE over bytes 0..37 */
} gauntlet_packet_t;

#ifdef __cplusplus
static_assert(sizeof(gauntlet_packet_t) == GAUNTLET_PACKET_SIZE, "gauntlet_packet_t must be 40 bytes");
#else
_Static_assert(sizeof(gauntlet_packet_t) == GAUNTLET_PACKET_SIZE, "gauntlet_packet_t must be 40 bytes");
#endif

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout. check("123456789") = 0x29B1 */
static inline uint16_t gauntlet_crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  size_t i;
  for (i = 0; i < len; i++) {
    int b;
    crc ^= (uint16_t)data[i] << 8;
    for (b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

/* Fill in magic, version, reserved and crc16. Call after setting the payload fields. */
static inline void gauntlet_packet_finalize(gauntlet_packet_t *pkt) {
  uint16_t crc;
  pkt->magic[0] = GAUNTLET_MAGIC0;
  pkt->magic[1] = GAUNTLET_MAGIC1;
  pkt->version = GAUNTLET_VERSION;
  pkt->reserved = 0;
  crc = gauntlet_crc16((const uint8_t *)pkt, 38);
  ((uint8_t *)pkt)[38] = (uint8_t)(crc & 0xFF); /* explicit little-endian store */
  ((uint8_t *)pkt)[39] = (uint8_t)(crc >> 8);
}

/* 1 when the 40 bytes at `bytes` have the right magic, version and CRC. */
static inline int gauntlet_packet_valid(const uint8_t *bytes) {
  uint16_t crc;
  if (bytes[0] != GAUNTLET_MAGIC0 || bytes[1] != GAUNTLET_MAGIC1 || bytes[2] != GAUNTLET_VERSION) return 0;
  crc = (uint16_t)(bytes[38] | ((uint16_t)bytes[39] << 8));
  return crc == gauntlet_crc16(bytes, 38);
}

/* Copy a validated frame into a struct (memcpy, so the caller's struct need not be aligned). */
static inline int gauntlet_packet_decode(const uint8_t *bytes, gauntlet_packet_t *out) {
  if (!gauntlet_packet_valid(bytes)) return 0;
  memcpy(out, bytes, GAUNTLET_PACKET_SIZE);
  return 1;
}

/* Q15 helpers. */
static inline float gauntlet_q15_to_float(int16_t v) { return (float)v / 32767.0f; }
static inline int16_t gauntlet_float_to_q15(float v) {
  float r = v * 32767.0f + (v >= 0 ? 0.5f : -0.5f);
  if (r > 32767.0f) r = 32767.0f;
  if (r < -32768.0f) r = -32768.0f;
  return (int16_t)r;
}

/* ---- Streaming parser for the receiver (round screen): scan for the header, verify CRC, else slide. ---- */
typedef struct {
  uint8_t buf[GAUNTLET_PACKET_SIZE];
  uint8_t n;              /* bytes currently in buf */
  uint32_t packets;       /* valid frames */
  uint32_t crc_errors;    /* good header, bad CRC */
  uint32_t dropped_bytes; /* bytes discarded while resyncing */
} gauntlet_parser_t;

static inline void gauntlet_parser_init(gauntlet_parser_t *p) { memset(p, 0, sizeof *p); }

/* Feed one byte. Returns 1 when `out` now holds a valid packet. */
static inline int gauntlet_parser_push(gauntlet_parser_t *p, uint8_t byte, gauntlet_packet_t *out) {
  /* reject bytes that cannot start/continue a header early, so garbage never fills the buffer */
  if (p->n == 0 && byte != GAUNTLET_MAGIC0) { p->dropped_bytes++; return 0; }
  if (p->n == 1 && byte != GAUNTLET_MAGIC1) { p->dropped_bytes += 1; p->n = (byte == GAUNTLET_MAGIC0) ? 1 : 0; return 0; }
  if (p->n == 2 && byte != GAUNTLET_VERSION) { p->dropped_bytes += 2; p->n = (byte == GAUNTLET_MAGIC0) ? 1 : 0; return 0; }
  p->buf[p->n++] = byte;
  if (p->n < GAUNTLET_PACKET_SIZE) return 0;
  if (gauntlet_packet_valid(p->buf)) {
    memcpy(out, p->buf, GAUNTLET_PACKET_SIZE);
    p->packets++;
    p->n = 0;
    return 1;
  }
  /* bad CRC: drop the first byte and slide the rest by one, then keep scanning inside the buffer */
  p->crc_errors++;
  p->dropped_bytes++;
  memmove(p->buf, p->buf + 1, GAUNTLET_PACKET_SIZE - 1);
  p->n = GAUNTLET_PACKET_SIZE - 1;
  {
    /* re-establish the invariant that buf[0..n) starts with a plausible header */
    uint8_t i = 0;
    while (i < p->n && !(p->buf[i] == GAUNTLET_MAGIC0 && (i + 1 >= p->n || p->buf[i + 1] == GAUNTLET_MAGIC1) &&
                         (i + 2 >= p->n || p->buf[i + 2] == GAUNTLET_VERSION)))
      i++;
    if (i > 0) {
      p->dropped_bytes += i;
      memmove(p->buf, p->buf + i, (size_t)(p->n - i));
      p->n = (uint8_t)(p->n - i);
    }
  }
  return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* GAUNTLET_PACKET_H */
