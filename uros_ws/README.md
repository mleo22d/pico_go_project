
### 1. Start Micro-ROS Agent
Micro-ROS follows the client-server architecture, so you need to start the Micro-ROS Agent.
You can do so using the [micro-ros-agent Docker](https://hub.docker.com/r/microros/micro-ros-agent):
```bash
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200
```

## What files are relevant?
- `pico_uart_transport.c`: Contains the board specific implementation of the serial transport (no change needed).
- `CMakeLists.txt`: CMake file.
- `main.c`: The actual ROS 2 publisher.

