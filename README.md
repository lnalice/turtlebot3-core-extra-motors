# turtlebot3_core_extra_motors
This can control extra two motors in a turtlebot3 waffle.

# Prerequisites
**1. Computer**
- Packages for turtlebot3 waffle should be installed.
- OpenCR 1.0, examples for turtlebot3 should be install in arduino ide.
- **Dynamixel2Arduino** library should be installed in arduino ide.

**2. Dynamixel X series**
- OpenCR must be connected to the computer using arduino ide when uploading this ino file.
- Dynamixel motors must be connected to OpenCR 1.0
- **ID**: wheel motors(1, 2), one extra motor (4), two extra motors (3, 4)
- **baudrate**: 1000000

**3. LED**
- LED must be connected to OpenCR 1.0
- **communication**: UART (no use SoftwareSerial)
- The number of LED is **4** (front, back, left, right of a robot)

# File Structure
```
├── LICENSE
├── README.md
├── lib
│   └── motor_driver
│       └── oneExtraMotor
│           ├── turtlebot3_motor_driver.cpp
│           └── turtlebot3_motor_driver.h
├── oneExtraMotor
│   ├── positionMode
│   │   ├── turtlebot3_core.ino
│   │   ├── turtlebot3_core_config.h
│   │   └── turtlebot3_waffle.h
│   └── velocityMode
│       ├── turtlebot3_core.ino
│       ├── turtlebot3_core_config.h
│       └── turtlebot3_waffle.h
└── twoExtraMotors
    └── positionMode
        ├── customVelocity
        │   ├── turtlebot3_core.ino
        │   ├── turtlebot3_core_config.h
        │   └── turtlebot3_waffle.h
        └── syncVelocity
            ├── bluetoothLED
            │   ├── README.md
            │   ├── turtlebot3_core.ino
            │   ├── turtlebot3_core_config.h
            │   └── turtlebot3_waffle.h
            ├── subscriberLED
            │   ├── README.md
            │   ├── turtlebot3_core.ino
            │   ├── turtlebot3_core_config.h
            │   └── turtlebot3_waffle.h
            ├── turtlebot3_core.ino
            ├── turtlebot3_core_config.h
            └── turtlebot3_waffle.h
```

# 1. Motor Driver
this files are under the file lib/motor_driver. This is only for onExtraMotor/velocityMotor. Without this case, you can use default motor_driver.
* turtlebot3_motor_driver.cpp
* turtlebot3_motor_driver.h

# 2. Turelebot3 Core: Dynamixel
I modified turtlebot3_waffle's core to control more Dynamixel motors.
If you upload this modified turtlebot core to OpenCR 1.0 using Arduino, you can control more extra motors and LED. 
* turtlebot3_core.ino
* turtlebot3_core_config.h
* turtlebot3_waffle.h

There're two big directories.
* **one extra motor**: when you want to control one more motor (position mode / velocity mode)
* **two extra motors**: when you want to control two more motors (position mode)

### 2-1. One Extra Motor

* **velocity mode**: the motor should be velocity mode
* **position mode**: the motor should be position mode

### 2-2. Two Extra Motors

Two motors's mode should be position mode

* **customVelocity**: two motors start moving **AT THE SAME SPEED** simultaneously. This is why if their degree to move is different, their operations will not end moving at the same time. (same velocity)
* **syncVelocity** : two motors start moving **AT THE COORDINATED SPEED** simultaneously. two motors start and finish moving simultaneously, even though their degree to move is different. (different velocity)

#### > Compare customVelocity and syncVelocity

|/|customVelocity|syncVelocity|
|--|--|--|
| mode | position | position |
|input | degree | degree |
| start time | same | same |
| end time | **SAME**  | **DIFFERENT**| 
| velocity | **DIFFERENT** | **SAME** |

# 3. Turelebot3 Core: Dynamixel + LED

This code is from **'twoExtraMotors/syncVelocity'**. OpenCR 1.0 would control two extra motors and 4 LED.

* turtlebot3_core.ino
* turtlebot3_core_config.h
* turtlebot3_waffle.h

There're two big directories.

* **bluetoothLED**: when you want to control two more motors (position mode) and LED using **BLUETOOTH**
* **subscriberLED**: when you want to control two more motors (position mode) and LED using **SUBSCRIBER** of ros topic

### 3-1. Control 4 LED using 'bluetooth'

* LED would be controlled by bluetooth. Connect to the terminal via Bluetooth.
    * If you send **'oo'**, you can **turn on** the LED
    * If you sent **'xx'**, you can **turn off** the LED
* Two motors's mode should be position mode
* LED and bluetooth module should be connected to OpenCR 1.0

### 3-2. Control 4 LED using 'subscriber' of ros topic

* LED would be controlled when topic '/blink_led'(msg: std_msgs/Byte).
    * When the robot moves forward, FRONT LED lights up (0b0001)
    * When the robot turns to the right, RIGHT LED lights up (0b0010)
    * When the robot moves backward, BACK LED lights up (0b0100)
    * When the robot turns to the left, LEFT LED lights up (0b1000)
    * When the robot control the module, ALL LED lights up (0b1111)
* Two motors's mode should be position mode
* LED should be connected to OpenCR 1.0

#### > Compare bluetoothLED and subscriberLED

|Terminal|Robot|bluetoothLED|subscriberLED|
|--|--|--|--|
| aa |_| TURN ON ||
| xx |_| TURN OFF ||
| rr |_| RIGHT LED ||
| ll |_| LEFT LED ||
| ff |_| FRONT LED ||
| bb |_| BACK LED ||
|| move forward |_| FRONT LED |
|| move backward |_| BACK LED |
|| turn right |_| RIGHT LED |
|| turn left |_| LEFT LED | 
|| control module |_| ALL |