// lwip_config/lwipopts.h
#ifndef LWIPOPTS_H
#define LWIPOPTS_H

#define NO_SYS                          1
#define LWIP_SOCKET                     1
#define LWIP_TIMEVAL_PRIVATE            0
#define LWIP_SO_RCVTIMEO                1
#define LWIP_UDP                        1
#define LWIP_NETCONN                    0
#define LWIP_NETIF_API                  0
#define MEMP_NUM_SYS_TIMEOUT            5

#endif /* LWIPOPTS_H */
