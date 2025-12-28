# Arduino Code

## Milestones

### Milestone 1: Temperature Sensing ✓ (In Progress)
**File:** `temperature_test/temperature_test.ino`

Tests basic temperature reading from DS18B20 sensor.

**Features:**
- Reads temperature every 1 second
- Outputs CSV format for easy plotting
- Built-in error checking and LED status indicator
- Serial plotter compatible

**How to Use:**
1. Wire DS18B20 as shown in code comments
2. Install required libraries (OneWire, DallasTemperature)
3. Upload code
4. Open Serial Monitor (115200 baud)
5. Or use Serial Plotter for real-time graph

---

### Milestone 2: Bang-Bang Control (Coming Soon)
Simple on/off control to maintain target temperature.

---

### Milestone 3: PID Implementation (Coming Soon)
Full PID controller with tunable gains.

---

### Milestone 4: Optimized PID (Coming Soon)
Tuned PID with performance testing.