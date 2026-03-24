# Hexapod Robot Platform

Hexapod robot designed for testing hexa-model control, mechanical design learning and prove what we had learnt at university are "useful".
It's just a platform moving robot so typicaly you can put anything on the top: arm, camera, lidar, machine gun,...

*NOTE 27/01/26: DON'T BUY MG996 Servo with Plastic gearbox, they will strip after 4-5 run times. 
---

##  Overview

This project focuses on building a simple hexapod with:

* Embedded firmware (low-level control)
* Gait generation & kinematics
* Use available module and also DIY PCB 

---

##  System Architecture

### Hardware

* 6 legs (3 DOF each)
* Modular joint/leg interface
* Central control board (MCU-based)
* Power distribution via contact plates + alignment pins
* Magnetic alignment system (non-current carrying)

### Software Stack

```
[ High-Level Control ]
        ↓
[ Gait Planner ]
        ↓
[ Inverse Kinematics ]
        ↓
[ Servo Control Layer ]
        ↓
[ Hardware Drivers (PWM / UART / CAN) ]
```

---

##  Features

* Modular mechanical + electrical design
* Plug-and-play leg modules
* Real-time gait control
* Scalable control architecture
* Designed for embedded + robotics R&D

---

##  Repository Structure

```
/firmware        → MCU code (STM32 / ESP32 / etc.)
/hardware        → PCB, schematics, mechanical CAD
/simulation      → Kinematics / gait simulation
/docs            → Design documents
```

---

##  Technologies

* Embedded C 
* ESP32
* I2C Driver
* Solidworks
* Matlab Modeling

---

##  Kinematics (Concept)

Each leg:

* 3 DOF → Coxa, Femur, Tibia
* Inverse kinematics used for foot positioning

Future work:

* Dynamic gait adaptation
* Terrain-aware walking
* Feedback control (IMU + sensors)

---

##  Roadmap

* Improve modular connector reliability
* Add sensor feedback (IMU, force sensing)
* Implement advanced gaits
* Develop ROS interface (optional)
* Optimize power distribution system

---

##  Contribution

This is an R&D-focused project. Contributions, ideas, and experiments are welcome.
Thanks
