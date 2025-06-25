#ifndef PICO_WIFI_TRANSPORT_H
#define PICO_WIFI_TRANSPORT_H

#include <uxr/client/profile/transport/custom/custom_transport.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

bool pico_wifi_transport_open(struct uxrCustomTransport * transport);
bool pico_wifi_transport_close(struct uxrCustomTransport * transport);
size_t pico_wifi_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode);
size_t pico_wifi_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode);

#ifdef __cplusplus
}
#endif

#endif // PICO_WIFI_TRANSPORT_H
