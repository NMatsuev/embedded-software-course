#include "link_proto.h"
#include <stdio.h>
#include <string.h>

static uint8_t xor_sum(const char *s, size_t n)
{
    uint8_t x = 0;
    while (n--) x ^= (uint8_t)*s++;
    return x;
}

static uint8_t crc8(const char *s, size_t n)
{
    uint8_t crc = 0x00;          
    while (n--) {
        crc ^= (uint8_t)*s++;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x07);
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

int link_build_data(char *buf, size_t size, const link_data_t *f)
{
    char body[LINK_LINE_MAX];
    int n = snprintf(body, sizeof(body), "DHT,%u,%u,%u,%d,%u",
                     (unsigned)f->dst, (unsigned)f->src,
                     (unsigned)f->seq, (int)f->temp10, (unsigned)f->hum10);
    if (n <= 0 || n >= (int)sizeof(body)) return 0;

    int total = snprintf(buf, size, "$%s*%02X\r\n", body,
                         (unsigned)xor_sum(body, (size_t)n));
    if (total <= 0 || (size_t)total >= size) return 0;
    return total;
}

void link_rx_init(link_rx_t *rx, uint8_t my_addr)
{
    memset(rx, 0, sizeof(*rx));
    rx->my_addr = my_addr;
}

static const char *parse_u(const char *p, uint32_t *v)
{
    uint32_t r = 0; int digits = 0;
    while (*p >= '0' && *p <= '9') {
        r = r * 10 + (uint32_t)(*p - '0');
        if (++digits > 5) return NULL;
        p++;
    }
    if (!digits) return NULL;
    *v = r;
    return p;
}

static const char *parse_i(const char *p, int32_t *v)
{
    int neg = 0; uint32_t u;
    if (*p == '-') { neg = 1; p++; }
    p = parse_u(p, &u);
    if (!p) return NULL;
    *v = neg ? -(int32_t)u : (int32_t)u;
    return p;
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static link_result_t parse_line(link_rx_t *rx, link_data_t *out)
{
    /* line = "$DHT,d,s,q,t,h*XX"*/
    const char *star = memchr(rx->line, '*', rx->len);
    if (!star || (rx->line + rx->len) - star != 3) return LINK_BAD;

    int h1 = hexval(star[1]), h2 = hexval(star[2]);
    if (h1 < 0 || h2 < 0) return LINK_BAD;
    uint8_t got = (uint8_t)((h1 << 4) | h2);
    if (got != xor_sum(rx->line + 1, (size_t)(star - rx->line - 1))) return LINK_BAD;

    if (strncmp(rx->line, "$DHT,", 5) != 0) return LINK_BAD;

    const char *p = rx->line + 5;
    uint32_t d, s, q, h; int32_t t;
    if (!(p = parse_u(p, &d)) || *p++ != ',') return LINK_BAD;
    if (!(p = parse_u(p, &s)) || *p++ != ',') return LINK_BAD;
    if (!(p = parse_u(p, &q)) || *p++ != ',') return LINK_BAD;
    if (!(p = parse_i(p, &t)) || *p++ != ',') return LINK_BAD;
    if (!(p = parse_u(p, &h)) || p != star)   return LINK_BAD;
    if (d > 255 || s > 255 || q > 65535 || h > 1000 || t < -500 || t > 1000)
        return LINK_BAD;

    out->dst = (uint8_t)d; out->src = (uint8_t)s;
    out->seq = (uint16_t)q; out->temp10 = (int16_t)t; out->hum10 = (uint16_t)h;

    if (out->dst == rx->my_addr || out->dst == LINK_ADDR_BROADCAST)
        return LINK_FRAME;
    return LINK_FOREIGN;
}

link_result_t link_rx_feed(link_rx_t *rx, uint8_t b, link_data_t *out)
{
    if (b == '$') {                   
        rx->active = 1;
        rx->len = 0;
        rx->line[rx->len++] = '$';
        return LINK_NONE;
    }
    if (!rx->active) return LINK_NONE; 

    if (b == '\r') return LINK_NONE;
    if (b == '\n') {
        rx->active = 0;
        return parse_line(rx, out);
    }
    if (b >= 0x80 || rx->len >= LINK_LINE_MAX) {
        rx->active = 0;
        return LINK_BAD;
    }
    rx->line[rx->len++] = (char)b;
    return LINK_NONE;
}