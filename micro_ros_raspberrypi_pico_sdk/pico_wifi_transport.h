#ifndef MICRO_ROS_WIFI_TRANSPORT_H_
#define MICRO_ROS_WIFI_TRANSPORT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

bool pico_wifi_transport_open(struct uxrCustomTransport* transport);
bool pico_wifi_transport_close(struct uxrCustomTransport* transport);
size_t pico_wifi_transport_write(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* errcode);
size_t pico_wifi_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* errcode);

#ifdef __cplusplus
}
#endif

#endif  // MICRO_ROS_WIFI_TRANSPORT_H_
