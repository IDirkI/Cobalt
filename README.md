# Cobalt

Cobalt is a C/C++ library for **math** and **control systems** intended to run on embedded hardware for robotics purposes first.

---

## Features v2.1.0
- 🟩 **Math**
    - Fixed size Vector & Matrices (`cobalt::math::linear_algebra`)
    - Transforms & Quaternions (`cobalt::math::geometry`) 
    - Complex numbers (`cobalt::math::algebra`)
- 🟦 **Control**
    - Discrete-time PID controller (`cobalt::control`):
        - Bilinear rtasform (Tustin) discretization
        - Derivative path LPF filtering
        - Integral path anti-windup via clamping
        - Output clamping
        - System state/memory read & write utility
- 🟧 **Hardware Abstraction Layer (HAL)**
    - GPIO abstraction layer (`cobalt::hal::GPIO`) for
        - Arduino
        - ESP-IDF
    - I2C abstraction layer (`cobalt::hal::I2C`) for
        - Arduino
        - ESP-IDF
- 🟥 **Robotics & Kinematics**
    - Robot Chains (`cobalt::kinematics::robot_chain`) to represent planar robots
    - Custom `.rob` file parser to generate Robot Chains automatically
        - Joint limits
        - Joint home & initial values
        - Joint axis
        - Link length & mass
    - IK & FK calculation algorithms
- 🧪 **Tests**
    - Catch2 Unit test implementation
    - CSV data logging + python script for visualization in PID unit tests

---

## Installation
### Prerequisites
 - CMAKE  ≥ 3.15
 - Python ≥ 3.8 
 - C++17 (tested with GCC)


### Installation & Usage
You can either (a) build locally and include in a project or (b) use direcly in a Platform.io project

#### <u>a) Building locally with CMake:</u>
1)  Install locally
    ``` bash
    git clone https://github.com/IDirkI/Cobalt.git
    cd Cobalt
    cmake -B build -S .
    cmake --build build
    cmake --install build
    ```
2)  Include in your project's `CMakeLists.txt`
    ```cmake
    find_package(Cobalt REQUIRED)
    target_link_libraries(<your executable> Cobalt::cobalt)
    ```
3)  _Optionally include_
    ```cmake
    include("${Cobalt_DIR}/CobaltFunctions.cmake")
    cobalt_generate_robot_headers()
    ```
    _to allow parsing of `.rob` files_

Library will be installed at `C:/dev/Cobalt`



#### <u>b) Using with Platform.io:</u>

1) Drop the provided _Cobalt_ files in the .zip under `lib/`
2) In `platform.ini`, use the flags
    ``` ini
    build_flags = -I lib/Cobalt/include
    ```
3)  _Optionally include_
    ``` ini
    extra_scripts = pre:lib/Cobalt/tools/kinematics/pio_run_rob_parser.py
    ```
    _to allow parsing of `.rob` files_


### ROB  files
#### <u>a) Building locally with CMake</u>
- Place `.rob` files under `<project root>/robots` 
- Robot headers will generate under `<project root>/build/robots`

#### <u>b) Using with Platform.io:</u>
- Place `.rob` files under `<project root>/lib/Cobalt/tools/kinematics/robots` 
- Robot headers will generate under `<project root>/lib/Cobalt/include/kinematics/robots`

---

## Roadmap for v2.0.0 life-span
- [x] HAL support starting with ESP32/Arduino systems
- [x] Inverse-kinematics(IK) calculator
- [ ] HAL timer
- [ ] ROS-like light weight middle-ware for simple communication
- [ ] Header parser and EEPROM Save/load for controller configs
- [ ] Expanded math module with:
    - [x] `Transform`, Homogeneous transforms & rotations
    - [ ]  `Tf` & `ZTf`, Continious-time and discrete-time transfer functions 

--- 

## Known Issues
- `Matrix` & `Vector` functions can have unexpected results under certain circumstances
- RobotChains can get stuck on initialization if joint & link numbers don't match up