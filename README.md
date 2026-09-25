# DC Motor Drive — Tramway Speed Control

Speed control of a series-excitation DC motor driving the "Carelli 1928" tramway,
modelled as an equivalent separately-excited DC machine. Course project for
*Dynamics of Electrical Machines and Drives* (Politecnico di Milano).

## Overview

A cascade control scheme regulates the tram speed along a 10 km track with varying
slope and speed references. Main features:

- Inner armature-current loop, outer speed loop, and a parallel excitation loop, all tuned with `pidtune`.
- Field weakening above base speed (excitation reference reduced as `E_n / (K·Ω)`).
- Variable torque limit derived from the excitation reference, keeping the armature current within ±156 A.
- Back-calculation anti-windup on all PI controllers.
- Feed-forward of the gravitational torque to reduce transients at slope changes.

## Files

- `DC_Motor.m` — parameter computation and controller tuning (run this first).
- `DC_Motorr.slx` — Simulink model of the drive.
- `DC_Motor_Drive_Report.pdf` — full report (model, tuning, results).

## How to run

1. Open MATLAB and run `DC_Motor.m` to load parameters and gains into the workspace.
2. Open and run the Simulink model `DC_Motorr.slx`.

## Results

The speed tracks the reference along the whole track with small overshoots, the
armature current is limited to ±156 A during transients, and the excitation current
is reduced only in the field-weakening section.

## Author

Golsa Shokri. 
