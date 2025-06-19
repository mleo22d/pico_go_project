#include "pico_wifi_transport.h"
#include <lwip/inet.h>

bool pico_wifi_transport_open(struct uxrCustomTransport * transport)
{
    pico_wifi_transport_t * ctx = (pico_wifi_transport_t *) transport->args;
    ctx->sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (ctx->sock < 0) {
        return false;
    }
    if (lwip_connect(ctx->sock, (struct sockaddr *)&ctx->agent_addr, sizeof(ctx->agent_addr)) < 0) {
        lwip_close(ctx->sock);
        return false;
    }
    return true;
}

bool pico_wifi_transport_close(struct uxrCustomTransport * transport)
{
    pico_wifi_transport_t * ctx = (pico_wifi_transport_t *) transport->args;
    lwip_close(ctx->sock);
    return true;
}

size_t pico_wifi_transport_write(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err)
{
    pico_wifi_transport_t * ctx = (pico_wifi_transport_t *) transport->args;
    int sent = lwip_send(ctx->sock, buf, len, 0);
    if (sent < 0) {
        *err = 1;
        return 0;
    }
    return sent;
}

size_t pico_wifi_transport_read(struct uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * err)
{
    pico_wifi_transport_t * ctx = (pico_wifi_transport_t *) transport->args;
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = timeout * 1000;
    lwip_setsockopt(ctx->sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    int recvd = lwip_recv(ctx->sock, buf, len, 0);
    if (recvd <= 0) {
        *err = 1;
        return 0;
    }
    return recvd;
}
