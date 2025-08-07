# 🤖 Three-Wheel Omnidirectional Robot

A high-mobility omnidirectional robotic platform designed using a 3-wheel omni-wheel chassis. The system is powered by a Teensy 4.1 microcontroller and uses real-time input from a PS4 controller for intuitive movement and field-oriented control. It integrates PID-based heading stabilization and sensor feedback to achieve smooth and accurate motion.

---

## 🚀 Features

- 🔄 **Omnidirectional Motion** using 3 omni-wheels and vector-based calculations
- 🧠 **Heading Stabilization** via BNO055 IMU and PID control loop
- 🎮 **PS4 Controller Support** using ESP32 over I2C for real-time control
- ⚙️ **Field-Oriented Control** for smoother navigation aligned with robot heading
- 📊 **Analog Sensor Feedback** for position referencing
- 💻 **Optimized Embedded C++ Code** for low-latency motor control and communication

---

## 🛠️ Technologies Used

- **Microcontroller:** Teensy 4.1  
- **Motion Control:** Cytron MD30C Motor Drivers  
- **Orientation Sensing:** BNO055 IMU  
- **Communication:** ESP32 (I2C Master-Slave with Teensy)  
- **Controller Interface:** PS4 DualShock Controller  
- **Programming:** C++, Arduino IDE, Teensyduino  

---

## 🧩 Architecture Overview

```plaintext
        [PS4 Controller]
               │
            [ESP32]
               │  (I2C)
          [Teensy 4.1]
         ┌──────────────┐
         │              │
  [BNO055]         [ADS1115]
         │              │
    [PID Control]   [Analog Position Sensing]
         │
    [Field-Oriented Control]
         │
     [Motor Output]
 ┌──────┬──────┬──────┐
 │M1    │M2    │M3    │
└──────┴──────┴──────┘
