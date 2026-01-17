# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

### Planned
- Hardware assembly and wiring
- Open-loop step response testing
- System identification (K, τ extraction)
- Bang-bang controller implementation
- PID controller implementation
- Performance testing and optimization
- Video demonstration
- Final project presentation

---

## [0.3.0] - 2024-12-XX

### Added
- Complete PID tuning methods documentation
- Ziegler-Nichols open-loop method
- Cohen-Coon tuning formulas
- AMIGO method for first-order systems
- Manual tuning guidelines and decision tree
- Anti-windup implementation strategies
- Bill of materials with UK supplier links
- Enhanced main README with project overview
- Development roadmap and milestones
- Expected performance metrics section

### Changed
- Expanded README with detailed project goals
- Added technical specifications table
- Improved documentation structure

---

## [0.2.0] - 2024-12-XX

### Added
- MATLAB system identification script (`system_model.m`)
- MATLAB PID tuning calculator (`tune_pid.m`)
- MATLAB plotting utilities (`plot_results.m`)
- System identification methodology documentation
- First-order transfer function fitting
- R² model validation
- Comprehensive theory documentation covering:
  - Continuous and discrete-time PID equations
  - P, I, D term breakdown and characteristics
  - Anti-windup strategies
  - Derivative filtering techniques
  - Expected thermal system behavior

### Documentation
- Added detailed mathematical derivations
- Included Arduino code examples for discrete PID
- Created performance comparison tables
- Added troubleshooting guides

---

## [0.1.0] - 2024-12-XX

### Added
- Initial project structure
- Repository organization (docs/, arduino/, matlab/, results/, hardware/)
- Arduino Milestone 1: Temperature sensor test code
- DS18B20 sensor integration with OneWire protocol
- CSV data output for Serial Plotter
- Error checking and LED status indicators
- Wiring diagrams for Milestone 1
- Hardware documentation structure
- MIT License
- README.md with project description
- .gitignore for Arduino, MATLAB, and OS files

### Hardware
- Selected core components:
  - Arduino Uno R3
  - DS18B20 waterproof temperature sensor
  - SSR-25DA solid state relay
  - 12V 5W silicone heating pad
  - 12V 2A power supply

---

## Project Milestones

### Milestone 1: Temperature Sensing ✓
**Status:** Code complete, awaiting hardware  
**Deliverables:**
- [x] Arduino sensor test code
- [x] Wiring diagram
- [x] CSV data logging format
- [ ] Hardware assembly
- [ ] Sensor validation test

### Milestone 2: Bang-Bang Control
**Status:** Planned  
**Deliverables:**
- [ ] On/off control algorithm
- [ ] Hysteresis implementation
- [ ] Performance characterization

### Milestone 3: PID Implementation
**Status:** Planned  
**Deliverables:**
- [ ] Discrete PID controller
- [ ] Anti-windup mechanism
- [ ] Tunable gains via Serial
- [ ] Real-time data logging

### Milestone 4: Optimized PID
**Status:** Planned  
**Deliverables:**
- [ ] System-identified gains
- [ ] Performance comparison plots
- [ ] Final tuning and validation
- [ ] Documentation of results

---

## Notes

### Development Environment
- **Arduino IDE:** 1.8.19
- **MATLAB Version:** R2023a
- **Git Version:** 2.39.0
- **OS:** macOS

### Dependencies
- OneWire Library v2.3.7
- DallasTemperature Library v3.9.0

### Testing Hardware
- Arduino Uno R3 (Elegoo)
- Elegoo Super Starter Kit components
- Digital multimeter for validation

---

## Future Enhancements

**Potential additions beyond core project:**
- [ ] Web interface for remote monitoring
- [ ] SD card data logging
- [ ] Multiple temperature zones
- [ ] Adaptive/gain-scheduled control
- [ ] Model predictive control (MPC)
- [ ] System identification via recursive least squares
- [ ] Auto-tuning via relay feedback

---

## Links

- **Repository:** https://github.com/YOUR-USERNAME/PID-Temperature-Controller
- **Issues:** https://github.com/YOUR-USERNAME/PID-Temperature-Controller/issues
- **Project Board:** https://github.com/YOUR-USERNAME/PID-Temperature-Controller/projects

---

**Last Updated:** December 2025