# Single-Phase Unipolar SPWM Inverter ⚡

![Project Status](https://img.shields.io/badge/Status-Completed-success)
![Hardware](https://img.shields.io/badge/Hardware-Arduino%20%7C%20IR2112%20%7C%20IRF540N-blue)
![Simulation](https://img.shields.io/badge/Simulation-Proteus%20%7C%20MATLAB-orange)
![PCB Design](https://img.shields.io/badge/PCB-Altium%20%7C%20EasyEDA-green)

A pure sine wave DC-to-AC inverter built from scratch using **Unipolar Sinusoidal Pulse Width Modulation (SPWM)**. This project leverages the **Arduino Nano** to generate high-frequency control signals (31.25 kHz), driving a full H-Bridge to convert 12V DC into 50Hz AC with low harmonic distortion.

---

## 📖 Table of Contents
- [Overview](#overview)
- [Key Features](#key-features)
- [System Architecture](#system-architecture)
- [Hardware Specifications](#hardware-specifications)
- [Software Implementation](#software-implementation)
- [Simulation & Results](#simulation--results)
- [Getting Started](#getting-started)
- [Future Improvements](#future-improvements)

---

## 🧐 Overview
Traditional square wave inverters are inefficient and harmful to sensitive electronics. This project implements a **software-defined** inverter that shifts complexity from hardware to code. By using **Unipolar SPWM**, we effectively double the switching frequency seen by the output filter, allowing for a smaller, cost-effective LC filter (2mH inductor) while maintaining a clean 50Hz sine wave output.

**Core Technologies:**
* **Controller:** ATmega328P (Arduino Nano) with direct register manipulation.
* **Driver Stage:** IR2112 High/Low Side Drivers with Bootstrap topology.
* **Power Stage:** IRF540N MOSFET Full Bridge.
* **Filter:** Custom passive LC filter (E-core Inductor).

---

## 🚀 Key Features
* **Pure Sine Wave Output:** <5% THD suitable for inductive loads.
* **Unipolar Modulation:** Reduces switching losses and doubles effective ripple frequency.
* **High-Frequency Switching:**
    * **Leg A:** 31.25 kHz (Timer1)
    * **Leg B:** 62.5 kHz (Timer0)
* **Software Deadtime:** 3µs protection gap to prevent H-Bridge shoot-through.
* **Optimized Code:** Look-Up Table (LUT) based ISR to bypass floating-point calculation lag.

---

## 🛠 System Architecture

### Block Diagram
*(Add your Block Diagram image here: `![Block Diagram](images/block_diagram.png)`)*

### Circuit Logic
The system uses a **Unipolar** switching strategy:
1.  **Positive Half-Cycle:** Leg A switches PWM (31kHz) while Leg B is clamped LOW.
2.  **Negative Half-Cycle:** Leg B switches PWM (62kHz) while Leg A is clamped LOW.

---

## 🔌 Hardware Specifications

| Component | Specification | Function |
| :--- | :--- | :--- |
| **Microcontroller** | Arduino Nano (ATmega328P) | Generates SPWM signals via Timers 0 & 1 |
| **MOSFETs** | IRF540N (x4) | 100V, 33A N-Channel Switches |
| **Gate Driver** | IR2112 (x2) | High/Low side drive with Bootstrapping |
| **Inductor** | 2mH (E-Core) | High-current smoothing filter |
| **Capacitor** | 0.15 µF | Polyester Film (High Voltage) |
| **Transformer** | 12V-0-12V / 220V | Step-up Toroidal/EI Core |

*PCB designed using **Altium Designer** and **EasyEDA Pro**.*

---

## 💻 Software Implementation
The Arduino logic is written in **C++** but bypasses standard `analogWrite` functions for speed.

### Key Logic:
1.  **Timer1 (16-bit):** Runs at **31.25 kHz** (No Prescaler, Phase Correct PWM). Drives the high-resolution sine wave.
2.  **Timer0 (8-bit):** Runs at **62.5 kHz** (No Prescaler). Handles the high-speed switching for the negative half-cycle.
3.  **Interrupt Service Routine (ISR):** Updates the duty cycle every $32\mu s$ from a pre-calculated Sine Look-Up Table (200 samples).

> **Warning:** The code modifies Timer0, which breaks standard Arduino functions like `millis()` and `delay()`. A custom `loopTick` counter is implemented for timekeeping.

---

## 📊 Simulation & Results

### Verified with Oscilloscope
* **Carrier Frequency:** Verified at **31.25 kHz**.
* **Output Frequency:** Stable at **50.1 Hz**.
* **Waveform:** Clean Sine Wave after LC filtering.

| PWM Signal (MATLAB/Simulink) | Final Output (Hardware) |
| :---: | :---: |
| images/signal%20spwm%201%20of%201.png | *(Add Scope Screenshot)*
---

## ⚙️ Getting Started

### Prerequisites
* Arduino IDE
* **Libraries:** None! (Direct Register Access used)

### Installation
1.  Clone the repo:
    ```bash
    git clone [https://github.com/yourusername/spwm-inverter.git](https://github.com/yourusername/spwm-inverter.git)
    ```
2.  Open `final_inverter_code.ino` in Arduino IDE.
3.  Connect your Arduino Nano.
4.  **Upload** the code.

### Wiring Guide
* **Pin 9 (D9):** High Side Leg A (PWM)
* **Pin 8 (D8):** Low Side Leg A (Digital)
* **Pin 6 (D6):** High Side Leg B (PWM)
* **Pin 5 (D5):** Low Side Leg B (Digital)

---

## 🔮 Future Improvements
* [ ] **Feedback Loop:** Implement PID control using ZMPT101B voltage sensor to stabilize output under load.
* [ ] **Isolation:** Add 6N137 Optocouplers to isolate the Arduino Logic (5V) from the Power Stage (12V).
* [ ] **Soft Start:** Implement ramp-up logic to prevent inrush current.

---

## 👥 Contributors
* **[Your Name]** - *Firmware & PCB Design*
* **[Team Member Name]** - *Simulation & Testing*

## 📄 License
This project is open-source and available under the [MIT License](LICENSE).
