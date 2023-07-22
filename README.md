# Autonomous Control Systems Assignment - Autonomous Vehicle Dynamics and Control, Cranfield University

<p align=center>
<img src="https://github.com/JohannesAutenrieb/RobustMissileControl/blob/master/img/missile.JPG" alt="Statemachine_main" height=500px>

This repository contains MATLAB/Simulink simulation software for the flight dynamic simulation and robust control of a transonic missile system.

### Description
This Software Project was created as part of the Autonomous Control Systems module, which was held by Dr. Hyo-Sang Shin, Cranfield University, and Dr. Spilios Theodoulis, French-German Research Institute of Saint-Louis for the students of the master's course Autonomous Vehicle Dynamics and Control at Cranfield University. This repository contains the MATLAB/Simulink software in which a transonic missile's longitudinal flight dynamic model has been augmented with uncertainties and afterward analyzed.

<p align=center>
<img src="https://github.com/JohannesAutenrieb/RobustMissileControl/blob/master/img/inner_step_updated.JPG" alt="inner" height=500px>

The problem presented in this assignment was the robust control of a transonic missile model with modeling uncertainties in MATLAB/Simulink. The lecturer has provided the linearised system and information about the uncertainties. This work was mainly on developing the required control law for which the robust control toolbox of MATLAB has been utilized. 

<p align=center>
<img src="https://github.com/JohannesAutenrieb/RobustMissileControl/blob/master/img/bode_mag_weights.svg" alt="Statemachine_main" height=500px>

### Structure
This assignment aims to develop a robust control system for a specified
missile by using MATLAB/Simulink. This general aim can be separated into six primary
objectives:

* Derivation of flight dynamic state-space equations of a missile
* Establishing a parametric uncertain model in Matlab
* Development and tuning of unstructured feedback loop
