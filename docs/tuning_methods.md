# PID Tuning Methods

## Table of Contents
1. [Introduction](#introduction)
2. [Ziegler-Nichols Methods](#ziegler-nichols-methods)
3. [Cohen-Coon Method](#cohen-coon-method)
4. [AMIGO Method](#amigo-method)
5. [Manual Tuning Guidelines](#manual-tuning-guidelines)
6. [Comparison and Recommendations](#comparison-and-recommendations)
7. [Practical Implementation Notes](#practical-implementation-notes)

---

## Introduction

PID tuning is the process of determining the optimal controller gains (Kp, Ki, Kd) for a given system. The goal is to achieve:
- **Fast response** (short rise time)
- **Minimal overshoot** (avoid temperature spikes)
- **Zero steady-state error** (reach and maintain setpoint)
- **Good disturbance rejection** (handle external changes)

This document covers several proven tuning methods and provides practical guidance for implementation.

---

## Ziegler-Nichols Methods

### Background

Developed by John G. Ziegler and Nathaniel B. Nichols in 1942, these remain the most widely known PID tuning methods. Two approaches exist:

1. **Open-Loop Method** (Step Response) - Used when you can run open-loop tests
2. **Closed-Loop Method** (Ultimate Gain) - Used when open-loop testing isn't practical

For our temperature system, we use the **Open-Loop Method**.

### Open-Loop (Step Response) Method

**Requirements:**
- First-order plus dead time (FOPDT) model: G(s) = K·e^(-Ls) / (τs + 1)
- Experimental step response data

**Parameter Extraction:**

From step response, identify:
- **K** = Steady-state gain (°C per unit input)
- **L** = Dead time / transport delay (seconds)
- **τ** = Time constant (seconds)

**Note:** For pure first-order systems (no dead time), approximate L ≈ 0.1τ

### Ziegler-Nichols Tuning Table

| Controller Type | Kp | Ti (integral time) | Td (derivative time) |
|-----------------|----|--------------------|---------------------|
| P               | τ/(K·L) | - | - |
| PI              | 0.9τ/(K·L) | L/0.3 = 3.3L | - |
| PID             | 1.2τ/(K·L) | 2L | 0.5L |

**Convert to standard form:**
- Ki = Kp / Ti
- Kd = Kp · Td

### Example Calculation

**Given system:**
- K = 30°C
- τ = 90 seconds
- L = 9 seconds (approximated as 0.1τ)

**PI Controller:**
```
Kp = 0.9 × 90 / (30 × 9) = 0.9 × 90 / 270 = 0.3

Ti = 3.3 × 9 = 29.7 seconds

Ki = Kp / Ti = 0.3 / 29.7 = 0.0101
```

**Result:** Kp = 0.3, Ki = 0.0101, Kd = 0

### Characteristics

**Advantages:**
✅ Simple and widely applicable  
✅ Well-documented and tested  
✅ Good starting point for further tuning

**Disadvantages:**
❌ Often produces ~25% overshoot  
❌ Aggressive tuning may cause oscillations  
❌ Assumes dead time (may not suit all systems)

**Recommendation:** Use as initial values, then fine-tune manually

---

## Cohen-Coon Method

### Background

Developed in 1953 by G.H. Cohen and G.A. Coon, this method provides more aggressive tuning than Ziegler-Nichols, particularly for systems with larger dead time.

### Formulas

**PI Controller:**
```
Kp = (τ / K·L) × (0.9 + L / 12τ)

Ti = L × (30 + 3L/τ) / (9 + 20L/τ)

Ki = Kp / Ti
```

**PID Controller:**
```
Kp = (τ / K·L) × (4/3 + L / 4τ)

Ti = L × (32 + 6L/τ) / (13 + 8L/τ)

Td = L × 4 / (11 + 2L/τ)

Ki = Kp / Ti
Kd = Kp × Td
```

### Example Calculation

**Same system as before:**
- K = 30°C, τ = 90s, L = 9s

**PI Controller:**
```
Kp = (90 / 30×9) × (0.9 + 9/1080) = 0.333 × 0.908 = 0.302

Ti = 9 × (30 + 3×9/90) / (9 + 20×9/90) = 9 × 30.3 / 11 = 24.8s

Ki = 0.302 / 24.8 = 0.0122
```

**Result:** Kp = 0.302, Ki = 0.0122, Kd = 0

### Characteristics

**Advantages:**
✅ Better for systems with significant dead time  
✅ Faster response than Ziegler-Nichols  
✅ Good disturbance rejection

**Disadvantages:**
❌ More aggressive (can cause overshoot)  
❌ More complex calculation  
❌ Sensitive to model accuracy

**Recommendation:** Good for systems where speed is prioritized over overshoot

---

## AMIGO Method

### Background

**AMIGO** (Approximate M-constrained Integral Gain Optimization) was developed by Åström and Hägglund in 2004. It's specifically designed for **first-order systems** and aims to balance performance with robustness.

### Philosophy

AMIGO prioritizes:
1. **Robustness** - Less sensitive to model errors
2. **Moderate speed** - Not overly aggressive
3. **Low overshoot** - Smooth response

### Formulas

**PI Controller** (recommended for thermal systems):
```
Kp = (0.2 + 0.45τ/L) / K

Ti = (0.4L + 0.8τ) / (L + 0.1Lτ/L)

Ki = Kp / Ti
```

**PID Controller:**
```
Kp = (0.2 + 0.45τ/L) / K

Ti = (0.4L + 0.8τ) / (L + 0.1Lτ/L)

Td = 0.5LTd / (0.3L + τ)

Ki = Kp / Ti
Kd = Kp × Td
```

### Example Calculation

**System:** K = 30°C, τ = 90s, L = 9s

**PI Controller:**
```
Kp = (0.2 + 0.45×90/9) / 30 = (0.2 + 4.5) / 30 = 0.157

Ti = (0.4×9 + 0.8×90) / (9 + 0.1×9×90/9) = 75.6 / 18 = 4.2s

Ki = 0.157 / 4.2 = 0.0374
```

**Result:** Kp = 0.157, Ki = 0.0374, Kd = 0

### Characteristics

**Advantages:**
✅ Excellent for first-order systems  
✅ Very robust to modeling errors  
✅ Low overshoot (<5% typical)  
✅ Modern, research-backed approach

**Disadvantages:**
❌ Slower than Cohen-Coon  
❌ Requires accurate τ identification  
❌ Less aggressive (may be too conservative for some)

**Recommendation:** **Best choice for thermal systems** - prioritizes safety and stability

---

## Manual Tuning Guidelines

### Starting Point Method

When you have no model or want to tune empirically:

**Step 1: Start Conservative**
```
Kp = 1.0 / K  (inverse of steady-state gain)
Ki = Kp / (5τ)  (very slow integral)
Kd = 0  (disable derivative)
```

**Step 2: Tune Proportional Gain (Kp)**

1. Set Ki = 0, Kd = 0 (P-only control)
2. Apply setpoint step
3. Increase Kp until you see slight oscillations
4. Reduce Kp by 50%

**Characteristics at different Kp:**
- **Too low:** Slow response, large offset
- **Good:** Fast response, small offset, no oscillation
- **Too high:** Oscillations, instability

**Step 3: Add Integral Action (Ki)**

1. With Kp set, gradually increase Ki
2. Monitor steady-state error → should approach zero
3. If oscillations appear, reduce Ki

**Characteristics at different Ki:**
- **Too low:** Persistent offset
- **Good:** Zero offset, no oscillations
- **Too high:** Overshoot, slow oscillations

**Step 4: Add Derivative (If Needed)**

⚠️ **For thermal systems, usually skip this step** (noise sensitivity)

If overshoot is excessive:
1. Add small Kd (start with Kd = Kp × 0.1)
2. Increase until overshoot is acceptable
3. Watch for noise amplification

### Relay Tuning (Åström-Hägglund Method)

**Advanced technique** for finding ultimate gain without instability:

1. Apply relay (bang-bang) control
2. Measure oscillation period (Pu) and amplitude (a)
3. Calculate: Ku = 4d / (πa), where d = relay amplitude
4. Use Z-N closed-loop formulas

---

## Comparison and Recommendations

### Performance Comparison

| Method | Overshoot | Rise Time | Robustness | Complexity |
|--------|-----------|-----------|------------|------------|
| Ziegler-Nichols | ~25% | Fast | Medium | Low |
| Cohen-Coon | ~30% | Very Fast | Low | Medium |
| AMIGO | <5% | Medium | High | Medium |
| Manual | Variable | Variable | High | Low (time-intensive) |

### For Our Temperature System

**Recommended Tuning Strategy:**

1. **Start:** AMIGO method (best for first-order thermal systems)
2. **Alternative:** Manual starting point for hands-on learning
3. **Compare:** Z-N for reference and comparison
4. **Avoid:** Full PID with derivative (noise issues)

### Practical Decision Tree
```
Do you have a good model (K, τ identified)?
│
├─ YES → Use AMIGO for balanced performance
│        or Cohen-Coon if speed is critical
│
└─ NO → Use manual tuning:
        1. Start conservative
        2. Increase Kp first
        3. Add Ki to eliminate offset
        4. Skip Kd for thermal systems
```

---

## Practical Implementation Notes

### Anti-Windup

**Problem:** Integral term accumulates when actuator saturates

**Solutions:**

**1. Conditional Integration (Simple)**
```cpp
if (output >= 0 && output <= 255) {
  integral += error * dt;
}
```

**2. Back-Calculation (Better)**
```cpp
float output_raw = P + I + D;
float output_sat = constrain(output_raw, 0, 255);

if (output_raw != output_sat) {
  float excess = output_raw - output_sat;
  integral -= excess / Ki;  // Reduce integral
}
```

### Derivative Filtering

**Problem:** Derivative amplifies sensor noise

**Solution:** Low-pass filter
```cpp
// First-order filter
float alpha = 0.2;  // Filter coefficient (0.1-0.3)
derivative_filtered = alpha * derivative + (1 - alpha) * derivative_filtered_prev;
```

### Derivative on Measurement

**Problem:** Setpoint changes cause derivative spikes ("derivative kick")

**Solution:** Calculate derivative of measurement, not error
```cpp
// Instead of: d_error = (error - error_prev) / dt;
float d_measurement = (temp - temp_prev) / dt;
D = -Kd * d_measurement;  // Note negative sign
```

### Bumpless Transfer

**Problem:** Switching controllers causes output jump

**Solution:** Initialize integral to match current output
```cpp
void switchToAutoMode() {
  integral = (current_output - Kp * error) / Ki;
}
```

### Gain Scheduling

**Advanced:** Adapt gains based on operating point
```cpp
if (abs(error) > 10) {
  // Far from setpoint - aggressive
  Kp_active = Kp * 1.5;
} else {
  // Near setpoint - gentle
  Kp_active = Kp;
}
```

---

## Tuning Checklist

Before implementing on hardware:

- [ ] System parameters (K, τ) identified from step response
- [ ] Tuning method selected (AMIGO recommended)
- [ ] Gains calculated using chosen method
- [ ] Anti-windup strategy decided
- [ ] Sampling time appropriate (Δt << τ)
- [ ] Output limits defined (0-255 for PWM)
- [ ] Safety limits configured (max temperature)
- [ ] Data logging ready for performance analysis

During testing:

- [ ] Start with conservative gains
- [ ] Test with small setpoint changes first
- [ ] Monitor for oscillations
- [ ] Record overshoot, rise time, settling time
- [ ] Check steady-state error
- [ ] Test disturbance rejection (add ice, etc.)
- [ ] Document final tuned gains

---

## Example Arduino Implementation
```cpp
// PID Parameters (from AMIGO tuning)
float Kp = 0.157;
float Ki = 0.0374;
float Kd = 0.0;

// PID variables
float integral = 0;
float error_prev = 0;
float derivative_filtered = 0;

// Timing
unsigned long lastTime = 0;
float dt;

void loop() {
  // Timing
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;
  
  // Read temperature
  float temp = readTemperature();
  
  // Calculate error
  float error = setpoint - temp;
  
  // Proportional term
  float P = Kp * error;
  
  // Integral term (with anti-windup)
  integral += error * dt;
  integral = constrain(integral, -50, 50);  // Clamp
  float I = Ki * integral;
  
  // Derivative term (if used)
  float derivative = (error - error_prev) / dt;
  float alpha = 0.2;
  derivative_filtered = alpha * derivative + (1 - alpha) * derivative_filtered;
  float D = Kd * derivative_filtered;
  
  // Control output
  float output = P + I + D;
  output = constrain(output, 0, 255);
  
  // Apply to actuator
  analogWrite(HEATER_PIN, output);
  
  // Store for next iteration
  error_prev = error;
}
```

---

## Further Reading

### Academic References

1. **Åström, K. J., & Hägglund, T. (2006).** *Advanced PID Control.* ISA-The Instrumentation, Systems, and Automation Society.

2. **Ziegler, J. G., & Nichols, N. B. (1942).** "Optimum Settings for Automatic Controllers." *Transactions of the ASME*, 64(11).

3. **Cohen, G. H., & Coon, G. A. (1953).** "Theoretical Consideration of Retarded Control." *Transactions of the ASME*, 75.

4. **Åström, K. J., & Hägglund, T. (2004).** "Revisiting the Ziegler-Nichols step response method for PID control." *Journal of Process Control*, 14(6), 635-650.

### Online Resources

- [MATLAB PID Tuning Documentation](https://www.mathworks.com/help/control/pid-controller-tuning.html)
- [Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library)
- [Control Tutorials (University of Michigan)](http://ctms.engin.umich.edu/)

---

**See also:**
- [PID Control Theory](theory.md)
- [System Identification](system_identification.md)
- [MATLAB Tuning Scripts](../matlab/tune_pid.m)