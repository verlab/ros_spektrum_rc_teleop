# ros_spektrum_rc_teleop
This project develops a teleop node for RC controllers running on an ESP32 microcontroller.

## Usage

A simple guide is presented below:

### Arduino IDE Configuration

You must install the **Rosserial Arduino Library** through the Arduino IDE's library manager and modify the original `Rosserial_Arduino_Library/src/ros.h` file as follows:

```c++
#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"

#if defined(ESP8266) or defined(ESP32) or defined(ROSSERIAL_ARDUINO_TCP)
  #include "ArduinoTcpHardware.h" // Remove this if condition so it always includes ArduinoHardware.h
#else
  #include "ArduinoHardware.h" 
#endif

namespace ros
{
#if defined(__AVR_ATmega8__) or defined(__AVR_ATmega168__)
  /* Downsize our buffers */
  typedef NodeHandle_<ArduinoHardware, 6, 6, 150, 150> NodeHandle;

#elif defined(__AVR_ATmega328P__)

  typedef NodeHandle_<ArduinoHardware, 25, 25, 280, 280> NodeHandle;

#elif defined(SPARK)

  typedef NodeHandle_<ArduinoHardware, 10, 10, 2048, 2048> NodeHandle;

#else

  typedef NodeHandle_<ArduinoHardware> NodeHandle; // Default 25, 25, 512, 512

#endif
}


#endif
```
After modification, the result should be:


```c++
#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"


/* Modified line to solve protocol erros*/

#include "ArduinoHardware.h"

namespace ros
{
#if defined(__AVR_ATmega8__) or defined(__AVR_ATmega168__)
  /* downsize our buffers */
  typedef NodeHandle_<ArduinoHardware, 6, 6, 150, 150> NodeHandle;

#elif defined(__AVR_ATmega328P__)

  typedef NodeHandle_<ArduinoHardware, 25, 25, 280, 280> NodeHandle;

#elif defined(SPARK)

  typedef NodeHandle_<ArduinoHardware, 10, 10, 2048, 2048> NodeHandle;

#else

  typedef NodeHandle_<ArduinoHardware> NodeHandle; // default 25, 25, 512, 512

#endif
}

#endif
```

This modification is necessary to resolve some protocol communication errors that may occur while running the node.

### ROS workstation configuration

To configure your ROS workstation, you only need to install two ROS packages:

```bash
sudo apt-get install ros-${ROS_DISTRO}-rosserial-arduino
sudo apt-get install ros-${ROS_DISTRO}-rosserial
```

That's it!

### Running the node

Running the node is straightforward. Follow these steps:

1. Upload the repository code to your ESP32 microcontroller.
2. Run `roscore` on your ROS workstation.
3. Start the `rosserial` client in your workspace with the following command:
```bash
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```


