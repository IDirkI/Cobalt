# Cobalt

Cobalt is a C++17 library for **math** and **control systems** intended to run on embedded hardware for robotics purposes first and foremost.

---

## ✨ Features v2.0.0
- **Math**
    - Fixed size Vector & Matricies (`cobalt::math::linear_algebra`)
    - Transforms & Quaternions (`cobalt::math::geometry`) 
    - Complex numbers (`cobalt::math::algebra`)
- **Control**
    - Discrete-time PID controller (`cobalt::control`):
        - Bilinear rtasform (Tustin) discretization
        - Derivative path LPF filtering
        - Integral path anti-windup via clamping
        - Output clamping
        - System state/memory read & write utility
- **Hardware Abstraction Layer (HAL)**
    - GPIO abstraction layer for
        - Arduino
        - ESP-IDF
    - I2C abstraction layer for
        - Arduino
        - ESP-IDF
- **Tests**
    - Catch2 Unit test implementation
    - CSV data logging + python script for visualization in PID unit tests

---

## 📦 Installation
### Prerequisites
 - CMAKE ≥ 3.15
 - C++17 compiler (tested with GCC)

### Build
``` bash
git clone https://github.com/IDirkI/Cobalt.git
cd cobalt
mkdir build && cd build
cmake ..
make -j
```

> #### 🐜 Note:  Platoform.io
> Use given flags & unflags in the `platform.ini` file:\
> `build-flags` = `-I lib/Cobalt/include -std=gnu++17`\
> `build-unflags` = `-std=gnu++11`


## 🎯 Roadmap for v2.0.0 life-span
- [x] HAL support starting with ESP32/Arduino systems
- [ ] Inverse-kinematics(IK) calculator
- [ ] ROS-like light weight middle-ware for simple communication
- [ ] Save/load options on system EEPROM for controller configs
- [ ] Expanded math module with:
    - [x] `Transform`, Homogeneous transforms & rotations
    - [ ]  `Tf` & `ZTf`, Continious-time and discrete-time transfer functions  