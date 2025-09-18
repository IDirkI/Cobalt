# Changelog

## ⚙️ Cobalt v2.0.0
---
### ⛓️‍💥 Breaking Changes
- Removed `toString` methods from math types

### 🚀 Added 
- Hardware abstracted GPIO and I2C interfaces for Arduino and ESP-IDF frameworks.
- New `Transform` type to rotate & translate 3-vectors or represent poses.

### 🛠️ Fixed
- `operator-()` implementation in `Vector`, `Matrix` and `Complex`

### 📍 Planned
- Controller save/load functionality
- ROS-like middlware
- More board/hardware additions to the HAL