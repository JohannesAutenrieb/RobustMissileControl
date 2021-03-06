# Autonomous Control Systems Assignment - Autonomous Vehicle Dynamics and Control, Cranfield University

<p align=center>
<img src="https://github.com/JohannesAutenrieb/RobustMissileControl/blob/master/img/missile.JPG" alt="Statemachine_main" height=500px>

This repository contains a MATLAB/Simulink simulation software for the flight dynamic simulation and robust control of an transsonic missile system.

### Discription
This Software Project was created as part of the Autonomous Control Systems module which was held by Dr. Hyo-Sang Shin, Cranfield University and Dr. Spilios Theodoulis, French-German Research Institute of Saint-Louis for the students of the masters course Autonomous Vehicle Dynamics and Control at Cranfield University. This repository contains the MATLAB/Simulink software in which the longitudinal flight dynamic model of a transsonic missile has been augmented with uncertanties and afterwards analysed.

<p align=center>
<img src="https://github.com/JohannesAutenrieb/RobustMissileControl/blob/master/img/inner_step.JPG" alt="inner" height=500px>

The problem presented in this assignment was the robust conrol of a transsonic missile model with modelling uncertanties in MATLAB/Simulink. The linearised system and information about the uncertanties have been provided by the lecturer. This work was mainly on the development of the required control law for which the robust contol toolbox of MATLAb has been utilised. 

<p align=center>
<img src="https://github.com/JohannesAutenrieb/RobustMissileControl/blob/master/img/bode_mag_weights.svg" alt="Statemachine_main" height=500px>

### Structure
This assignment aims to develope a robust control system for a specified
missile by using MATLAB/Simulink. This general aim can be separated into six primary
objectives:

* Derviation of flight dynamic state-space equations of a missile
* Establishing a parametric uncertain model in Matlab
* Development and tuning of unstructured feedback loop
