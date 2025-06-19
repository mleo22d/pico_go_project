#ifndef PICO_WIFI_TRANSPORT_H
#define PICO_WIFI_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <lwip/sockets.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

typedef struct pico_wifi_transport
{
    int sock;
    struct sockaddr_in agent_addr;
} pico_wifi_transport_t;

bool pico_wifi_transport_open(struct uxrCustomTransport * transport);
bool pico_wifi_transport_close(struct uxrCustomTransport * transport);
size_t pico_wifi_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t pico_wifi_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

#ifdef __cplusplus
}
#endif

#endif // PICO_WIFI_TRANSPORT_H
