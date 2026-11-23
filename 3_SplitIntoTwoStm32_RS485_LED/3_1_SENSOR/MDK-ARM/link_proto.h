#ifndef LINK_PROTO_H
#define LINK_PROTO_H

#include <stdint.h>
#include <stddef.h>

#define LINK_ADDR_BROADCAST  0
#define LINK_ADDR_SENSOR     1   
#define LINK_ADDR_LOGGER     2   

#define LINK_LINE_MAX        40 

/*$DHT,<dst>,<src>,<seq>,<temp10>,<hum10>*<XX>\r\*/
typedef struct {
    uint8_t  dst;
    uint8_t  src;
    uint16_t seq;
    int16_t  temp10;
    uint16_t hum10;
} link_data_t;

typedef enum {
    LINK_NONE = 0,   
    LINK_FRAME,     
    LINK_FOREIGN,    
    LINK_BAD       
} link_result_t;

typedef struct {
    char    line[LINK_LINE_MAX + 1];
    uint8_t len;
    uint8_t active;
    uint8_t my_addr;
} link_rx_t;

int  link_build_data(char *buf, size_t size, const link_data_t *f);

void          link_rx_init(link_rx_t *rx, uint8_t my_addr);
link_result_t link_rx_feed(link_rx_t *rx, uint8_t byte, link_data_t *out);

#endif