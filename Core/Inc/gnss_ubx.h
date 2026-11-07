#ifndef GNSS_UBX_H
#define GNSS_UBX_H

#include <stdbool.h>
#include <stdint.h>

bool gnss_ubx_configure_receiver(void);

uint32_t gnss_ubx_get_baud_rate_found(void);

#endif