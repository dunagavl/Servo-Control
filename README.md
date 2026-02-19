
#Servo Control
Simulink generates a control input (servo angle), which is transmitted over serial to the dsPIC. The firmware parses the incoming data and updates the PWM duty cycle accordingly, producing the appropriate servo control pulse.

---

## Hardware Platform

- **Microcontroller:** dsPIC33FJ32GS608  
- **Architecture:** 16-bit Digital Signal Controller  
- **PWM Frequency:** 50 Hz (20 ms period)  
- **Servo Type:** Standard hobby servo  
- **Communication Interface:** UART Serial  

The dsPIC33FJ32GS608 was selected for its:

- High-resolution PWM modules  
- Deterministic timing behavior  
- Motor control-oriented peripheral set  
- Efficient 16-bit processing core  

---

## PWM Implementation

PWM configuration was implemented in `PWM.c` using the dsPIC hardware PWM / Output Compare module.

### Servo Timing Requirements

A standard servo expects:

- **Period:** 20 ms (50 Hz)  
- **Pulse Width Range:**  
  - 1.0 ms → 0°  
  - 1.5 ms → 90°  
  - 2.0 ms → 180°  

### Timer Period Calculation

The PWM period register was configured using:

PR = (Fcy / (Prescaler × Fpwm)) − 1

Where:

- Fcy = Instruction cycle frequency  
- Fpwm = 50 Hz  

Duty cycle values are updated dynamically based on incoming serial data.

### Why Hardware PWM?

Hardware PWM was used instead of software-based toggling to ensure:

- Precise pulse timing  
- Minimal jitter  
- Reduced CPU overhead  
- Deterministic signal generation  

This is critical for reliable servo positioning.

---

## Serial Communication (Simulink → dsPIC)

Simulink transmits control values over UART to the microcontroller.

### Control Flow

1. Simulink generates an angle command.  
2. The value is transmitted over serial.  
3. The dsPIC UART module receives the data.  
4. Firmware converts the value into a PWM duty cycle.  
5. The PWM register is updated in real time.  

This architecture bridges model-based design and embedded firmware deployment.

---

## Angle to Pulse Width Mapping

If Simulink sends an angle:

θ ∈ [0°, 180°]

It is mapped to pulse width:

PW = 1ms + (θ / 180) × 1ms

The pulse width is then converted into timer counts and written to the PWM duty register.

---

## Software Structure
/src
├── Main.c → System initialization and control logic
├── PWM.c → PWM configuration and duty updates


### Main Responsibilities

- Oscillator and peripheral initialization  
- UART configuration  
- Serial data parsing  
- Duty cycle updates  
- Real-time actuator control  

---

## Development Environment

- MPLAB X IDE  
- XC16 Compiler  
- PICkit programmer  
- Simulink model for serial control  

---

## Learning Outcomes

This project strengthened understanding of:

- Embedded C firmware development  
- Hardware peripheral configuration  
- Timer-based PWM generation  
- Real-time embedded control  
- Serial protocol implementation  
- Integration between control modeling tools and embedded hardware  

---

## Applications

This architecture is applicable to:

- Robotics actuation systems  
- Mechatronics platforms  
- Motor control systems  
- Industrial automation  
- Surgical robotic subsystems  
- Embedded control deployments  

---

## Future Improvements

- Closed-loop servo feedback  
- On-device PID control  
- Multi-channel PWM expansion  
- Command filtering and smoothing  
- Fault detection and safety limits  

---

## Technical Summary

This project demonstrates the complete pipeline from high-level control modeling in Simulink to low-level hardware actuation using deterministic PWM generation on a 16-bit microcontroller. It reflects practical skills in embedded systems, control integration, and real-time firmware design.

