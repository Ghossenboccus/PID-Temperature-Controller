# PID-Temperature-Controller
Arduino-based PID temperature control system with MATLAB modeling and system identification
# PID Temperature Controller

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Status: In Progress](https://img.shields.io/badge/Status-In%20Progress-orange.svg)]()
[![Hardware: Arduino](https://img.shields.io/badge/Hardware-Arduino-00979D.svg)]()
[![Language: C++](https://img.shields.io/badge/Language-C%2B%2B-blue.svg)]()

A complete implementation of a digital PID temperature controller using Arduino, demonstrating control theory principles, system identification, and real-time embedded control. This project bridges academic control theory with practical hardware implementation.

![Project Banner](results/project_banner.png)
*Closed-loop temperature control system with real-time feedback*

---

## 🎯 Project Overview

This project implements a **closed-loop PID (Proportional-Integral-Derivative) temperature control system** for a small water bath heating application. The system demonstrates:

- **Classical control theory** applied to real hardware
- **System identification** from experimental step response data
- **PID controller design** using multiple tuning methodologies
- **Embedded systems programming** with real-time constraints
- **Data acquisition and analysis** using MATLAB

### Why This Project?

Temperature control is fundamental in industrial automation, from chemical reactors to HVAC systems. This project demonstrates:

1. **Control Theory Application**: Moving from textbook equations to working hardware
2. **System Modeling**: Identifying transfer functions from real-world data
3. **Engineering Workflow**: Requirements → Design → Implementation → Testing → Documentation
4. **Problem-Solving**: Handling practical issues (noise, saturation, sensor limitations)

---

## 🛠️ Hardware Components

| Component | Purpose | Specifications |
|-----------|---------|----------------|
| Arduino Uno R3 | Microcontroller | ATmega328P, 16MHz |
| DS18B20 | Temperature sensor | ±0.5°C accuracy, 1-Wire digital |
| SSR-25DA | Solid state relay | 25A, 3-32VDC control |
| Heating Element | Heat source | 12V, 5W silicone pad |
| Power Supply | Heater power | 12V, 2A DC adapter |

**Total Cost:** ~£30 (excluding Arduino)

📄 **[Complete Bill of Materials →](hardware/bill_of_materials.md)**  
📐 **[Wiring Diagrams →](hardware/wiring_diagram.md)**

---

## 📊 System Specifications

### Target Performance

| Metric | Target Value | Notes |
|--------|--------------|-------|
| Operating Range | 30-80°C | Safe for water bath testing |
| Steady-State Error | ±1°C | Acceptable for demonstration |
| Overshoot | <5% | Minimizes temperature spike |
| Settling Time | <120s | Reasonable for thermal system |
| Sampling Rate | 1 Hz | Adequate for slow thermal process |

### Control Strategy

The system uses a **PI controller** (Proportional-Integral, no Derivative):
- **Proportional term**: Provides immediate response to current error
- **Integral term**: Eliminates steady-state offset
- **No derivative**: Avoided due to noise sensitivity in temperature measurements
```
u(t) = Kp·e(t) + Ki·∫e(t)dt
```

Where:
- `u(t)` = Control signal (PWM duty cycle, 0-255)
- `e(t)` = Error (setpoint - measured temperature)
- `Kp, Ki` = Tuned gains from system identification

---

## 🧮 Control Theory Background

### First-Order System Model

Thermal systems typically exhibit first-order lag behavior:
```
G(s) = K / (τs + 1)
```

**System Parameters** (to be identified experimentally):
- **K** = DC gain (temperature rise per unit power input)
- **τ** = Time constant (thermal inertia of the system)

### PID Control Equation (Discrete-Time)

Implemented in the Arduino:
```cpp
// Calculate error
error = setpoint - measuredTemp;

// Proportional term
P = Kp * error;

// Integral term (with anti-windup)
integral += error * dt;
I = Ki * integral;

// Control output (0-255 PWM)
output = constrain(P + I, 0, 255);
```

📚 **[Detailed Theory Documentation →](docs/theory.md)**  
🔬 **[System Identification Guide →](docs/system_identification.md)**  
⚙️ **[Tuning Methods →](docs/tuning_methods.md)**

---

## 📁 Project Structure
```
PID-Temperature-Controller/
├── README.md                    # This file
├── LICENSE                      # MIT License
├── CHANGELOG.md                 # Development history
│
├── docs/                        # Documentation
│   ├── theory.md               # PID control theory and equations
│   ├── system_identification.md # Step response analysis methodology
│   └── tuning_methods.md       # Ziegler-Nichols, Cohen-Coon, AMIGO
│
├── arduino/                     # Arduino firmware
│   ├── README.md               # Arduino setup instructions
│   ├── temperature_test/       # Milestone 1: Sensor verification
│   ├── bang_bang_control/      # Milestone 2: On/off control
│   ├── pid_controller/         # Milestone 3: PID implementation
│   └── pid_optimized/          # Milestone 4: Tuned controller
│
├── matlab/                      # Analysis and simulation
│   ├── system_model.m          # Load data, fit transfer function
│   ├── tune_pid.m              # Calculate PID gains
│   └── plot_results.m          # Generate publication plots
│
├── results/                     # Experimental data and plots
│   ├── step_response.csv       # Open-loop test data
│   ├── pid_test.csv            # Closed-loop performance data
│   ├── system_parameters.txt   # Identified K and τ
│   └── *.png                   # Performance plots
│
└── hardware/                    # Hardware documentation
    ├── bill_of_materials.md    # Component list with suppliers
    └── wiring_diagram.md       # Circuit diagrams and connections
```

---

## 🚀 Development Roadmap

### ✅ Phase 1: Planning & Theory (Completed)
- [x] Project requirements definition
- [x] Component selection and procurement
- [x] Theoretical foundation documentation
- [x] MATLAB analysis scripts preparation

### 🔄 Phase 2: Hardware Setup (In Progress)
- [x] Repository structure and documentation
- [ ] Hardware assembly and wiring
- [ ] Sensor validation (Milestone 1)
- [ ] Open-loop step response testing

### ⏳ Phase 3: System Identification (Upcoming)
- [ ] Collect step response data
- [ ] Extract system parameters (K, τ)
- [ ] Validate first-order model fit
- [ ] Document transfer function

### ⏳ Phase 4: Controller Implementation (Upcoming)
- [ ] Bang-bang control (Milestone 2)
- [ ] Basic PID implementation (Milestone 3)
- [ ] Apply tuning methods (Z-N, AMIGO)
- [ ] Performance testing and optimization (Milestone 4)

### ⏳ Phase 5: Documentation & Demonstration (Upcoming)
- [ ] Record video demonstration
- [ ] Generate performance comparison plots
- [ ] Write comprehensive build guide
- [ ] Publish final results

📋 **[Detailed Progress Tracking →](CHANGELOG.md)**

---

## 📈 Expected Results

### Open-Loop Response (No Control)
```
Temperature
  │
80°C├─────────────────  ← Steady state
  │              ╱
  │            ╱
  │          ╱
  │        ╱
  │      ╱
20°C├────╯
  └─────────────────────→ Time
    0s        300s      600s
    
    Slow rise, no regulation
```

### Closed-Loop Response (With PID)
```
Temperature
  │
50°C├──────────────────  ← Setpoint tracking
  │    ╱‾‾‾‾‾‾‾‾‾‾
  │  ╱
  │╱
20°C├
  └─────────────────────→ Time
    0s    60s   120s
    
    Fast rise, minimal overshoot
```

---

## 🎓 Learning Outcomes

This project demonstrates:

### Technical Skills
- **Control Systems**: PID theory, tuning, stability analysis
- **Embedded Programming**: Real-time C++ on microcontrollers
- **System Modeling**: Transfer functions, time constants, step response
- **Data Analysis**: MATLAB scripting, curve fitting, validation
- **Hardware Integration**: Sensors, actuators, power electronics

### Engineering Practices
- **Requirements Engineering**: Defining specifications
- **Systematic Design**: Theory → simulation → implementation
- **Testing & Validation**: Comparing predicted vs. actual performance
- **Documentation**: Clear technical writing for reproducibility

### Problem-Solving
- **Anti-windup**: Handling actuator saturation
- **Noise filtering**: Dealing with sensor noise
- **Thermal modeling**: Understanding system dynamics
- **Practical constraints**: Working within hardware limitations

---

## 🔧 Getting Started

### Prerequisites

**Hardware:**
- Arduino Uno R3 (or compatible)
- Components listed in [Bill of Materials](hardware/bill_of_materials.md)

**Software:**
- Arduino IDE (1.8.19 or later)
- MATLAB (R2020a or later) or Octave
- Git for version control

### Installation

1. **Clone the repository:**
```bash
   git clone https://github.com/YOUR-USERNAME/PID-Temperature-Controller.git
   cd PID-Temperature-Controller
```

2. **Install Arduino libraries:**
   - Open Arduino IDE
   - Go to Tools → Manage Libraries
   - Install: `OneWire` and `DallasTemperature`

3. **Upload test code:**
   - Open `arduino/temperature_test/temperature_test.ino`
   - Select board: Arduino Uno
   - Upload and open Serial Monitor (115200 baud)

4. **Run MATLAB analysis:**
   - Navigate to `matlab/` folder
   - Run `system_model.m` after collecting data

📖 **[Detailed Build Guide →](docs/build_guide.md)** *(Coming Soon)*

---

## 📊 Performance Metrics

*Results will be added here after testing*

### System Identification
- Transfer Function: `G(s) = K / (τs + 1)`
- DC Gain (K): TBD °C
- Time Constant (τ): TBD seconds
- Model Fit (R²): TBD

### PID Controller Performance
- Overshoot: TBD %
- Rise Time: TBD seconds
- Settling Time: TBD seconds
- Steady-State Error: TBD °C

---

## 📸 Gallery

*Photos and videos will be added as the project progresses*

---

## 🤝 Contributing

This is a personal portfolio project, but suggestions and feedback are welcome! Feel free to:
- Open an issue for questions or suggestions
- Fork the repository for your own experiments
- Share improvements or alternative approaches

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 👤 Author

**Gabriel Hossenboccus**  
BEng Electrical and Electronic Engineering  
Newcastle University (Class of 2025)

📧 Email: Gabriel@orchardestatesltd.co.uk  
💼 LinkedIn: [Your LinkedIn]  
🔗 Portfolio: [Your GitHub Profile]

---

## 🙏 Acknowledgments

- **Control Theory References:**
  - Åström, K. J., & Hägglund, T. (2006). *Advanced PID Control*
  - Franklin, G. F., et al. (2019). *Feedback Control of Dynamic Systems*
  
- **Arduino Community:**
  - OneWire Library by Paul Stoffregen
  - DallasTemperature Library by Miles Burton

- **Inspiration:**
  - Industrial temperature control applications
  - Academic control systems coursework at Newcastle University

---

## 📚 Related Projects

Looking for more control systems projects? Check out:
- [Multi-Axis Motion Control System](https://github.com/YOUR-USERNAME/Motion-Control) *(Coming Soon)*
- [Modbus RTU PLC Simulator](https://github.com/YOUR-USERNAME/Modbus-Simulator) *(Coming Soon)*

---

<div align="center">

**⭐ If you find this project helpful, please consider giving it a star! ⭐**

*Last Updated: December 2024*

</div>