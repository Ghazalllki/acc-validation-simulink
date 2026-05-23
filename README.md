# Testing and Validation of Adaptive Cruise Control (ACC)

**Course:** Testing and Validation of Automated Road Vehicles – 2025/2026  
**University:** Università degli Studi di Napoli Federico II  
**Supervisor:** Prof. Angelo Coppola  

**Team:**
- Ghazal Kianfar
- Fatemeh Bakhtiari
- Pooya Zare Baravati

---

## Overview

This project presents a systematic testing and validation framework for an Adaptive Cruise Control (ACC) system using Monte Carlo simulation. The goal is to evaluate ACC performance and robustness under real-world uncertainty in vehicle, road, and driving parameters.

The analysis covers statistical input characterization, KPI-based performance evaluation, correlation analysis, and a robustness check under varying lead vehicle drive cycles. A Multiple Linear Regression (MLR) model is also developed as a lightweight prediction tool.

---

## Methodology

### 1. Input Variables & Sampling

Six input parameters were selected to represent the main real-world uncertainties affecting ACC behavior:

| Parameter | Description |
|---|---|
| Mass (m) | Varies with vehicle load |
| Aerodynamic drag coefficient (Cd) | Affected by shape, roof racks, wind |
| Rolling resistance (Cr) | Depends on tire type, pressure, road surface |
| Road slope | Models uphill and downhill conditions |
| Time gap | Reflects ACC tuning and safety preference |
| Desired speed (v_des) | Represents different driving contexts |

**Latin Hypercube Sampling (LHS)** was used to generate 200 samples. LHS ensures uniform coverage of the input space with uncorrelated, unbiased samples — validated through scatter matrix analysis and comparison with theoretical uniform distributions.

---

### 2. Key Performance Indicators (KPIs)

Three continuous KPIs and one binary KPI were evaluated:

| KPI | Type | Description |
|---|---|---|
| Minimum Relative Distance | Continuous | Safety margin between ego and lead vehicle |
| Minimum Time-to-Collision (TTC) | Continuous | Time available to react before collision |
| Maximum Absolute Jerk | Continuous | Measure of ride comfort (longitudinal smoothness) |
| Collision Flag | Binary | 1 if collision occurred, 0 otherwise |

**Normality testing** (Lilliefors and Kolmogorov–Smirnov) confirmed that KPI distributions are non-Gaussian, as expected for extreme-value outputs. Non-parametric tools (PDFs, CDFs, boxplots, percentile analysis) were therefore used throughout.

---

### 3. Results Summary

**Safety (Min Distance)**
- Median ~3.2 m; compact interquartile range indicating low variability
- Nearly 100% of simulations remain above ~1.2 m
- Zero collisions across all 200 runs

**Safety (Min TTC)**
- ~90% of scenarios maintain Min TTC above 2.4 s (threshold: 1.5 s)
- Median ~3.1 s; no extreme outliers

**Comfort (Max |Jerk|)**
- ~70% of scenarios below 2.5 m/s³; ~90% below 3.2 m/s³
- Rare high-jerk events correspond to sudden speed mismatches under extreme input combinations

---

### 4. Correlation Analysis

Quantitative correlation analysis used Pearson's R and p-value thresholds:
- |R| > 0.3 → practically significant effect
- p < 0.05 → statistically significant effect

**Key finding:** Time gap is the dominant input, with strong influence on both Min Distance and Min TTC. Other inputs contribute variability but do not dominate trends. Max Jerk is primarily driven by transient control actions rather than steady driving conditions.

---

### 5. Multiple Linear Regression (MLR)

A Multiple Linear Regression model was fitted on statistically significant variables for continuous KPIs (collision flag excluded as a binary variable). The model serves as a lightweight prediction tool: given new input values, it estimates expected KPI outputs without re-running the full simulation.

---

### 6. Robustness Check

The ACC system was tested under varying lead vehicle drive cycles to verify that performance generalizes beyond the nominal scenario. Results confirm stable behavior across different driving profiles.

---

## Tools & Technologies

- **MATLAB / Simulink** — ACC system model and simulation
- **Latin Hypercube Sampling** — input space exploration
- **Statistical analysis** — distribution fitting, normality testing, correlation analysis
- **Multiple Linear Regression** — prediction model

---

## Repository Structure

```
├── modelf/          # Simulink ACC model (.slx)
├── mainf/        # MATLAB scripts for LHS, KPI computation, correlation analysis
├── analysisf/        # Output plots (PDFs, CDFs, boxplots, scatter matrices)
└── README.md
```

---

## How to Run

1. Open MATLAB and navigate to the `scripts/` folder
2. Run `main.m` (or the relevant entry-point script) to execute the Monte Carlo simulation
3. Output KPI statistics and plots will be saved to `results/`

> **Note:** Simulink model requires MATLAB R202x or later.
