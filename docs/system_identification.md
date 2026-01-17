# System Identification

## Table of Contents
1. [Overview](#overview)
2. [First-Order System Model](#first-order-system-model)
3. [Step Response Method](#step-response-method)
4. [Graphical Parameter Extraction](#graphical-parameter-extraction)
5. [Validation](#validation)

---

## Overview

**System identification** is the process of building a mathematical model of a dynamic system based on measured input-output data.

For our temperature control system, we need to:
1. Apply a step input (turn heater fully on)
2. Record the temperature response over time
3. Fit the data to a mathematical model
4. Extract system parameters (gain K, time constant τ)
5. Use these parameters for PID tuning

---

## First-Order System Model

### Why First-Order?

Thermal systems typically exhibit **first-order lag** behavior:
- Single energy storage element (thermal mass)
- Exponential response to step inputs
- No oscillations or overshoot (without control)

### Transfer Function

$$
G(s) = \frac{Y(s)}{U(s)} = \frac{K}{\tau s + 1}
$$

Where:
- **s** = Laplace variable
- **Y(s)** = Output (temperature)
- **U(s)** = Input (heater power)
- **K** = Steady-state gain (°C per unit input)
- **τ** = Time constant (seconds)

### Time-Domain Response

For a unit step input u(t) = 1:

$$
y(t) = K \cdot (1 - e^{-t/\tau})
$$

**Key characteristics:**
- At t = τ: y(τ) = 0.632·K (63.2% of final value)
- At t = 3τ: y(3τ) = 0.95·K (95% of final value)
- At t = 5τ: y(5τ) = 0.993·K (settled)

---

## Step Response Method

### Experimental Procedure

1. **Initial Conditions**
   - Start with system at ambient temperature (T₀ ≈ 20-25°C)
   - Heater off, system stable

2. **Apply Step Input**
   - Turn heater to **full power** (100% PWM)
   - Record timestamp

3. **Data Collection**
   - Sample temperature every 1 second
   - Continue until temperature stabilizes (no change for 60s)
   - Typical duration: 5-10 minutes

4. **Save Data**
   - Export as CSV: `time (s), temperature (°C)`
   - Import into MATLAB for analysis

### Expected Data
```
Time(s)  Temperature(°C)
0        23.5           ← Ambient
10       25.2
20       28.1
30       32.4
...
180      52.8
190      53.1           ← Approaching steady-state
200      53.2
```

---

## Graphical Parameter Extraction

### Method 1: Time Constant from 63.2% Rise

**Steps:**

1. **Calculate steady-state value (K):**
   $$
   K = T_{\infty} - T_0
   $$
   Where:
   - T∞ = Final steady-state temperature
   - T₀ = Initial temperature

2. **Find 63.2% point:**
   $$
   T_{63.2} = T_0 + 0.632 \cdot K
   $$

3. **Read time constant (τ):**
   - Find time when temperature reaches T₆₃.₂
   - This time is τ

**Example:**
```
T₀ = 23°C (initial)
T∞ = 53°C (final)
K = 53 - 23 = 30°C

T₆₃.₂ = 23 + 0.632 × 30 = 41.96°C

From graph: T reaches 42°C at t = 85 seconds
Therefore: τ = 85 seconds
```

### Method 2: Two-Point Method

Use two points at different % of rise:

- **28% point:** t₁ where y(t₁) = 0.283·K
- **63.2% point:** t₂ where y(t₂) = 0.632·K

Then:
$$
\tau = t_2 - t_1
$$

More robust to noise than single-point method.

### Method 3: Logarithmic Plot

Transform the equation:

$$
\ln(1 - \frac{y(t)}{K}) = -\frac{t}{\tau}
$$

Plot ln(1 - y(t)/K) vs. t:
- Should be a straight line with slope -1/τ
- Intercept at t=0 should be 0

---

## MATLAB Implementation

### Load and Plot Data
```matlab
% Load experimental data
data = readmatrix('step_response.csv');
time = data(:, 1);
temp = data(:, 2);

% Plot raw data
figure;
plot(time, temp, 'b.-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Temperature (°C)');
title('Step Response - Experimental Data');
```

### Extract Parameters
```matlab
% Calculate steady-state gain
T0 = temp(1);              % Initial temperature
Tinf = mean(temp(end-10:end));  % Average of last 10 points
K = Tinf - T0;             % Steady-state gain

fprintf('Initial Temperature: %.2f °C\n', T0);
fprintf('Final Temperature: %.2f °C\n', Tinf);
fprintf('Steady-State Gain K: %.2f °C\n', K);

% Find time constant (63.2% method)
target_temp = T0 + 0.632 * K;
idx = find(temp >= target_temp, 1);
tau = time(idx);

fprintf('Time Constant τ: %.2f seconds\n', tau);
```

### Generate Model Response
```matlab
% Create theoretical first-order response
t_model = 0:0.1:max(time);
y_model = T0 + K * (1 - exp(-t_model / tau));

% Plot comparison
hold on;
plot(t_model, y_model, 'r--', 'LineWidth', 2);
legend('Experimental Data', 'First-Order Model', 'Location', 'southeast');
```

### Calculate Fit Quality (R²)
```matlab
% Interpolate model to match experimental time points
y_model_interp = T0 + K * (1 - exp(-time / tau));

% Calculate R² (coefficient of determination)
SS_res = sum((temp - y_model_interp).^2);  % Residual sum of squares
SS_tot = sum((temp - mean(temp)).^2);      % Total sum of squares
R_squared = 1 - (SS_res / SS_tot);

fprintf('Model Fit (R²): %.4f\n', R_squared);

% Good fit: R² > 0.95
% Acceptable: R² > 0.90
% Poor fit: R² < 0.85 (consider second-order model)
```

---

## Validation

### Visual Inspection

**Good model fit:**
```
Temperature
  │
  │         Experimental ●●●●●●
  │                    ●
  │                  ●
  │                ●   Model ----
  │              ●
  │            ●
  │          ●
  │        ●
  │      ●
  │    ●
  │  ●
  └──────────────────────────→ Time
  
  R² > 0.95, curves overlap closely
```

**Poor fit (needs second-order or dead time):**
```
Temperature
  │
  │         ●●●●●● Experimental
  │       ●
  │     ●      ---- Model (too fast)
  │   ●
  │  ●
  │ ●
  │●
  └──────────────────────────→ Time
  
  R² < 0.85, significant mismatch
```

### Residual Analysis
```matlab
% Calculate residuals (errors)
residuals = temp - y_model_interp;

% Plot residuals
figure;
subplot(2,1,1);
plot(time, residuals, 'b.-');
grid on;
xlabel('Time (s)');
ylabel('Residual (°C)');
title('Model Residuals');
yline(0, 'r--');

subplot(2,1,2);
histogram(residuals, 20);
xlabel('Residual (°C)');
ylabel('Frequency');
title('Residual Distribution');

% Good fit: residuals randomly scattered around zero
% Bad fit: systematic pattern in residuals
```

---

## Expected Results for Our System

Based on typical small-scale thermal systems:

### Predicted Parameters

| Parameter | Expected Range | Units | Notes |
|-----------|---------------|-------|-------|
| K (gain) | 20-40 | °C | Depends on heater power and water volume |
| τ (time constant) | 60-120 | seconds | Larger volume → larger τ |
| T₀ (initial) | 20-25 | °C | Room temperature |
| T∞ (final) | 50-80 | °C | Limited by heater power and heat loss |

### Example Values

For a **300ml water bath** with **5W heater**:
- K ≈ 30°C
- τ ≈ 90 seconds
- Settling time (5τ) ≈ 7.5 minutes

Transfer function:
$$
G(s) = \frac{30}{90s + 1}
$$

---

## Next Steps

Once K and τ are identified:

1. **Use for PID tuning** → [tuning_methods.md](tuning_methods.md)
2. **Simulate in MATLAB** → [../matlab/system_model.m](../matlab/system_model.m)
3. **Validate with closed-loop tests**

---

## Troubleshooting

### Problem: R² < 0.85 (Poor Fit)

**Possible causes:**
1. **Dead time present** - Add delay: G(s) = K·e^(-Ls) / (τs + 1)
2. **Second-order dynamics** - Use: G(s) = K / (τ₁s + 1)(τ₂s + 1)
3. **Nonlinear effects** - Heat loss increases with temperature
4. **Insufficient settling time** - Extend data collection

### Problem: Noisy Data

**Solutions:**
- Apply moving average filter before fitting
- Increase sampling interval (reduce measurement noise)
- Use curve fitting toolbox (`fit()` function)

### Problem: Temperature Never Stabilizes

**Causes:**
- Heater too powerful → reduce power or increase water volume
- Poor insulation → add insulation to container
- Measurement drift → check sensor calibration

---

## References

- Ljung, L. (1999). *System Identification: Theory for the User*. Prentice Hall.
- Ogata, K. (2010). *Modern Control Engineering*. Pearson.

---

**See also:**
- [PID Control Theory](theory.md)
- [Tuning Methods](tuning_methods.md)
- [MATLAB Scripts](../matlab/)