# turtlebot3_core_extra_motors
This can control extra two motors in a turtlebot3 waffle.

# Prerequisites
**1. Computer**
- Packages for turtlebot3 waffle should be installed.
- OpenCR, examples for turtlebot3 should be install in arduino ide.
- **Dynamixel2Arduino** library should be installed in arduino ide.

**2. Dynamixel X series**
- OpenCR shoule be connected the computer using arduino ide.
- **ID**: wheel motors(1, 2), one extra motor (4), two extra motors (3, 4)
- **baudrate**: 1000000

# File Structure
```
├── LICENSE
├── README.md
├── lib
│   └── motor_driver
│       ├── default
│       │   ├── turtlebot3_motor_driver.cpp
│       │   └── turtlebot3_motor_driver.h
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
            ├── turtlebot3_core.ino
            ├── turtlebot3_core_config.h
            └── turtlebot3_waffle.h
```

# 1. Motor Driver
this files are under the file lib/motor_driver. This is only for onExtraMotor/velocityMotor. Without this case, you can use default motor_driver.
* turtlebot3_motor_driver.cpp
* turtlebot3_motor_driver.h

# 2. Turelebot3 Core
I modified turtlebot3_waffle's core to control more Dynamixel motors.
If you upload turtlebot core using three files, you can control more extra motors. 
* turtlebot3_core.ino
* turtlebot3_core_config.h
* turtlebot3_waffle.h

There're two big directories.
* **one extra motor**: when you want to control one more motor (position mode / velocity mode)
* **two extra motors**: when you want to control two more motors (position mode)

### 2-1. One Extra Motor

* **velocity mode**: the motor should be velocity mode.
* **position mode**: the motor should be position mode.

### 2-2. Two Extra Motors

Two motors's mode should be position mode

* **customVelocity**: two motors start moving in the same time. If their degree to move is different, their operations will not end up simulately. (same velocity)
* **syncVelocity** : two motors start moving and finish moving in the same time. (different velocity)

|/|customVelocity|syncVelocity|
|--|--|--|
|input | degree | degree |
| start time | same | same |
| end time | same | different| 
| velocity | different | same |