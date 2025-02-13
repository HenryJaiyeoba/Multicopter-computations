# Multicopter Drone Performance Analysis

This MATLAB script performs comprehensive analysis of multicopter drone capabilities, focusing on performance metrics such as hover time, range, and power requirements based on various propeller configurations.

## Overview

The script analyzes the following key aspects of multicopter drone performance:
- Thrust and power requirements for different propeller configurations
- Battery weight optimization
- Hover time calculations
- Range estimation with bank angle optimization
- Motor power requirements
- Total system weight analysis

## Features

- Calculate and visualize:
  - Total thrust and power requirements
  - Battery weight estimation
  - Hover time capabilities
  - Optimal bank angles for maximum range
  - Speed optimization
  - Range predictions

## Required Parameters

### Drone Configuration
- Number of propellers: 8
- RPM range: 1000-10000 RPM
- Propeller diameter range: 0.2-1.0 meters
- Minimum hover time requirement: 30 minutes

### Physical Parameters
- Frame weight: 4 kg
- Payload weight: 10 kg
- Payload volume: 0.1 cubic meters
- Drag coefficient: 1.2

### Battery Specifications
- Power-to-weight ratio: 5.7 kW/kg (LiPo)
- Energy-to-weight ratio: 450 kJ/kg (LiPo)

### Motor Parameters
- Cruise power-to-weight ratio: 3 kW/kg (including ESCs)

## Outputs

The script generates multiple figures showing:
1. Total thrust and power requirements
2. Battery weight distribution
3. Hover time capabilities
4. Cropped results based on minimum hover time requirement
5. Range optimization results including:
   - Optimal bank angles
   - Optimal speeds
   - Maximum range estimates

## Usage

1. Make sure you have MATLAB installed
2. Clone this repository
3. Open the script in MATLAB
4. Run the script to generate all analysis plots
5. Modify parameters as needed for your specific drone configuration

```matlab
% Example of modifying basic parameters
Nprop = 8;           % Number of propellers
m_payload = 10;      % Payload mass in kg
Thovermin = 30 * 60; % Minimum hover time in seconds
```

## Visualization

The script produces several 3D surface plots showing the relationships between:
- RPM
- Propeller diameter
- Various performance metrics

Each plot includes proper labeling and color scaling for easy interpretation.

