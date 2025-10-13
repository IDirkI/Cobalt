# Changelog

## Cobalt v2.1.0

---

### Added 
- Joint, Link and RobotChain types for robot descriptions
- FK and IK calculations on RobotChains 
- On compile `.rob` file parser, used to turn robot description to usable header files under `build/robots` or `include/cobalt/kinematics/robots` depending on if the library is built or being used in the project
- Machine wide build option & alongside `cobalt_generate_robot_headers()` CMAKE function defintion for use in downstream projects

### Fixed
- `operator*()` and `operator*=()` implementations in `Matrix`
- `I2C` defined properly when on ESPIDF framework
- Minor comments and spellings