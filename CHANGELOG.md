# Changelog

## [Cobalt v2.2.0] - 6.1.2026

---
### Breaking Changes
- Major refactoring of the math API breaks many previous calls to it as functions have new parametrs, different labels or don't exist anymore
- Kinematics module is not updated to the new math module so kinematics is unusuable currently
- Transform orientation is now represented as `Quaternion` instead of `Matrix<3,3>`
- `Complex` is deprecated and will be removed in the future

### Added 
#### Math
- Centralized math config via `config.hpp`
- New unified indexing type `index_t` and new templated threshold `epsilon<T>`
- Stricter template restrictions between numeric types and floating point types

#### Vector
- Expanded vector utility for data manipulation and statistics
- New triple product operation

#### Matrix
- New matrix functions and utility 
- Exact left/right pseudo-inverse implementations
- Improved efficiency in different decomposition operations

#### Quaternion
- New factories
- Expanded interpolation, conversion, and angular analysis utilities
- Fixed quaternion arithmetic

#### Transform
- Quaternion-based transform operations
- Interpolation and kinematic utility
- New conversion and validation utilities

### Fixed
- Multiple correctness issues in vector, matrix and quaternion types
- Algebraic errors
- Out-of-bounds access bugs in matrix functions
- Minor spelling mistakes

### Tests
- Comprehensive rewrite of math module tests
- Expanded quaternion utility test coverage