#ifndef _NEC_PROT_H
#define _NEC_PROT_H

/*
 * nec_protocol.c function prototypes
 */
int remote_init();
uint32_t remote_decode();
void remote_encode(uint8_t value, uint8_t addr);


#endif  /* _NEC_PROT_H */
