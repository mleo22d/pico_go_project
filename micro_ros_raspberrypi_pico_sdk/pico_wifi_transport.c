#include <stdio.h>
#include <string.h>
#include "pico/cyw43_arch.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>
#include "wifi_config.h"

static int udp_socket;
static struct sockaddr_in agent_addr;

bool pico_wifi_transport_open(struct uxrCustomTransport * transport)
{
    if (cyw43_arch_init()) {
        return false;
    }

    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        return false;
    }

    udp_socket = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_socket < 0) {
        return false;
    }

    agent_addr.sin_family = AF_INET;
    agent_addr.sin_port = htons(AGENT_PORT);
    inet_aton(AGENT_IP, &agent_addr.sin_addr);

    return true;
}

bool pico_wifi_transport_close(struct uxrCustomTransport * transport)
{
    lwip_close(udp_socket);
    return true;
}

size_t pico_wifi_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
    int sent = lwip_sendto(udp_socket, buf, len, 0, (struct sockaddr *)&agent_addr, sizeof(agent_addr));
    return (sent < 0) ? 0 : (size_t)sent;
}

size_t pico_wifi_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
    struct sockaddr_in source_addr;
    socklen_t addr_len = sizeof(source_addr);

    lwip_fcntl(udp_socket, F_SETFL, O_NONBLOCK);

    uint64_t start = time_us_64();
    while ((time_us_64() - start) < timeout * 1000) {
        int received = lwip_recvfrom(udp_socket, buf, len, 0, (struct sockaddr *)&source_addr, &addr_len);
        if (received > 0) {
            return (size_t)received;
        }
        sleep_ms(1); // evitar bloqueo
    }

    *errcode = 1;
    return 0;
}
