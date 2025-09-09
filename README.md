# Cobalt

Cobalt is a C++17 library for **math** and **control systems** intended to run on embedded hardware for robotics purposes first and foremost.

---

## ✨ Features v1.0.0
- **Math**
    - Fixed size vector & matricies (`cobalt::math::linear_algebra`)
    - Complex numbers (`cobalt::math::algebra`)
    - Quaternions (`cobalt::math::geometry`)
- **Control**
    - Discrete-time PID controller (`cobalt::control`):
        - Bilinear rtasform (Tustin) discretization
        - Derivative path LPF filtering
        - Integral path anti-windup via clamping
        - Output clamping
        - System state/memory read & write utility
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

## 🎯 Roadmap for v2.0.0
- [ ] Full HAL support starting with ESP32/Arduino systems
- [ ] ROS-like light weight middle-ware for simple communication
- [ ] Save/load options on system EEPROM for controller configs
- [ ] Expanded math module with:
    - [ ] `Dual`, dual vectors for completeness
    - [ ] `Transform`, Homogeneous transforms & rotations
    - [ ]  `Tf` & `ZTf`, Continious-time and discrete-time transfer functions  