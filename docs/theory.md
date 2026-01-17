# PID Control Theory

## Table of Contents
1. [Introduction](#introduction)
2. [PID Controller Fundamentals](#pid-controller-fundamentals)
3. [Continuous-Time PID](#continuous-time-pid)
4. [Discrete-Time Implementation](#discrete-time-implementation)
5. [Control Action Breakdown](#control-action-breakdown)
6. [Anti-Windup and Practical Considerations](#anti-windup-and-practical-considerations)

---

## Introduction

PID (Proportional-Integral-Derivative) control is the most widely used feedback control algorithm in industrial applications. Despite its age, it remains dominant due to its simplicity, effectiveness, and intuitive tuning.

This document covers the mathematical foundations and practical implementation of PID control for our temperature control system.

---

## PID Controller Fundamentals

### What is Feedback Control?

A feedback control system continuously measures the output and adjusts the input to minimize error.
```
         ┌──────────────┐
r(t) ───>│  Controller  │───> u(t) ───>┌─────────┐
 ^       └──────────────┘              │  Plant  │
 │            PID                      │ (Heater)│
 │                                     └─────────┘
 │                                          │
 │                      ┌───────────────────┘
 │                      │ y(t)
 │     e(t) = r(t) - y(t)
 └──────────────────────┘
         Error
```

**Terminology:**
- **r(t)** = Reference/Setpoint (desired temperature)
- **y(t)** = Process Variable/Output (actual temperature)
- **e(t)** = Error (difference between setpoint and actual)
- **u(t)** = Control Signal/Manipulated Variable (heater power)

---

## Continuous-Time PID

### The Classic PID Equation

In continuous time, the PID controller is expressed as:

$$
u(t) = K_p \cdot e(t) + K_i \int_0^t e(\tau) \, d\tau + K_d \frac{de(t)}{dt}
$$

Where:
- **Kₚ** = Proportional gain
- **Kᵢ** = Integral gain  
- **Kₑ** = Derivative gain

### Alternative Form (ISA Standard)

$$
u(t) = K_p \left[ e(t) + \frac{1}{T_i} \int_0^t e(\tau) \, d\tau + T_d \frac{de(t)}{dt} \right]
$$

Where:
- **Tᵢ** = Integral time constant (seconds)
- **Tₑ** = Derivative time constant (seconds)

**Conversion:**
- Kᵢ = Kₚ / Tᵢ
- Kₑ = Kₚ · Tₑ

---

## Discrete-Time Implementation

Microcontrollers operate in discrete time, so we must approximate the continuous PID equation.

### Sampling

Let **Δt** = sampling period (e.g., 0.1s = 100ms)

We calculate control action at discrete time steps: t₀, t₁, t₂, ... where tₙ = n·Δt

### Discrete PID Equation

$$
u[n] = K_p \cdot e[n] + K_i \sum_{k=0}^n e[k] \cdot \Delta t + K_d \frac{e[n] - e[n-1]}{\Delta t}
$$

Or broken down:

$$
\begin{align}
P[n] &= K_p \cdot e[n] \\
I[n] &= I[n-1] + K_i \cdot e[n] \cdot \Delta t \\
D[n] &= K_d \cdot \frac{e[n] - e[n-1]}{\Delta t} \\
u[n] &= P[n] + I[n] + D[n]
\end{align}
$$

### Arduino Implementation Variables
```cpp
// Timing
unsigned long currentTime = millis();
unsigned long lastTime = 0;
float dt = (currentTime - lastTime) / 1000.0;  // Convert to seconds

// Error calculation
float error = setpoint - measuredTemp;

// Proportional term
float P = Kp * error;

// Integral term (accumulated over time)
integral += error * dt;
float I = Ki * integral;

// Derivative term (rate of change)
float derivative = (error - lastError) / dt;
float D = Kd * derivative;

// Control output
float output = P + I + D;

// Store for next iteration
lastError = error;
lastTime = currentTime;
```

---

## Control Action Breakdown

### Proportional (P) Control

$$
u_P(t) = K_p \cdot e(t)
$$

**What it does:**
- Responds proportionally to current error
- Larger error → larger control action

**Characteristics:**
- ✅ Simple and fast
- ✅ Reduces rise time
- ❌ Steady-state error (offset)
- ❌ Can cause oscillations if Kₚ too high

**Example:** 
If error = 5°C and Kₚ = 2:
- u = 2 × 5 = 10 (maximum heating power)

If error = 0.5°C and Kₚ = 2:
- u = 2 × 0.5 = 1 (low heating power)

---

### Integral (I) Control

$$
u_I(t) = K_i \int_0^t e(\tau) \, d\tau
$$

**What it does:**
- Accumulates error over time
- Eliminates steady-state error

**Characteristics:**
- ✅ Removes offset
- ✅ Ensures zero steady-state error (in theory)
- ❌ Can cause overshoot
- ❌ Integral windup (explained below)

**Example:**
Small constant error of 0.5°C:
- After 10 seconds: I = Kᵢ × (0.5 × 10) = 5·Kᵢ
- After 20 seconds: I = Kᵢ × (0.5 × 20) = 10·Kᵢ
- Keeps increasing until error becomes zero

---

### Derivative (D) Control

$$
u_D(t) = K_d \frac{de(t)}{dt}
$$

**What it does:**
- Predicts future error based on rate of change
- Provides damping

**Characteristics:**
- ✅ Reduces overshoot
- ✅ Improves stability
- ❌ Amplifies measurement noise
- ❌ Often not needed for slow processes (like temperature)

**Example:**
Temperature rising quickly (error decreasing fast):
- de/dt = -2°C/s (error decreasing)
- If Kₑ = 1: u_D = 1 × (-2) = -2 (reduce heating)

---

### Combined PID Action
```
Time →
Temperature
  ↑
  │     Setpoint ────────────────────────
  │              /
  │            /    ← P responds to current error
  │          /      ← I eliminates offset
  │        /        ← D dampens overshoot
  │      /
  │    /
  │  /
  └──────────────────────────────────────→
     Initial    Rise    Settling
     response   time    time
```

---

## Anti-Windup and Practical Considerations

### Integral Windup Problem

**What is it?**
When the actuator saturates (e.g., heater already at 100%), the integral keeps accumulating error even though more control action is impossible.

**Result:**
- Massive overshoot when error finally decreases
- Long settling time

### Solution 1: Clamping
```cpp
// Only accumulate integral if output is not saturated
if (output < 255 && output > 0) {
  integral += error * dt;
}
```

### Solution 2: Back-Calculation
```cpp
float output = P + I + D;
float saturatedOutput = constrain(output, 0, 255);

// If saturated, reduce integral
if (output != saturatedOutput) {
  float excessOutput = output - saturatedOutput;
  integral -= excessOutput / Ki;  // Back-calculate
}
```

### Derivative Filtering

Raw derivative amplifies noise. Filter it:

$$
D_{filtered}[n] = \alpha \cdot D[n] + (1-\alpha) \cdot D_{filtered}[n-1]
$$

Where α ≈ 0.1 to 0.3 (low-pass filter)
```cpp
float derivativeFiltered = 0.8 * derivativeFiltered + 0.2 * derivative;
```

### Derivative on Measurement (Not Error)

Better practice: calculate derivative of measurement, not error.

**Why?** Setpoint changes cause derivative spikes.
```cpp
// Instead of: derivative = (error - lastError) / dt;
float derivative = -(measuredTemp - lastMeasuredTemp) / dt;  // Note negative sign
```

---

## Expected System Behavior

### Our Temperature System Model

Thermal systems are typically **first-order** processes:

$$
G(s) = \frac{K}{\tau s + 1}
$$

Where:
- **K** = DC gain (°C per Watt)
- **τ** = Time constant (seconds to 63.2% of final value)

**We will identify K and τ experimentally in the next phase.**

### Predicted Response

For a step input (heater on → full power):

$$
y(t) = K \cdot (1 - e^{-t/\tau})
$$

**Typical values we expect:**
- K ≈ 20-40°C (depends on water volume and heater power)
- τ ≈ 60-120 seconds (thermal mass of water)

---

## Summary

| Control Mode | Reduces Rise Time | Eliminates Offset | Reduces Overshoot | Increases Stability |
|--------------|-------------------|-------------------|-------------------|---------------------|
| P            | ✅ Yes            | ❌ No             | ⚠️ Small          | ⚠️ Can destabilize  |
| I            | ⚠️ Small          | ✅ Yes            | ❌ Increases      | ❌ Reduces          |
| D            | ⚠️ Small          | ❌ No             | ✅ Yes            | ✅ Improves         |

**For our temperature system:**
- Start with **PI control** (Kₑ = 0) - derivative not needed for slow thermal processes
- Tune Kₚ first for acceptable rise time
- Add Kᵢ to eliminate steady-state error
- Add Kₑ only if significant overshoot remains

---

## Next Steps

1. **System Identification** → [system_identification.md](system_identification.md)
2. **Tuning Methods** → [tuning_methods.md](tuning_methods.md)
3. **Implementation** → [../arduino/](../arduino/)

---

**References:**
- Åström, K. J., & Hägglund, T. (2006). *Advanced PID Control*. ISA.
- Franklin, G. F., Powell, J. D., & Emami-Naeini, A. (2019). *Feedback Control of Dynamic Systems*. Pearson.