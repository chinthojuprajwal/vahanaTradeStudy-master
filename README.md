# EVTOL ATC optimisation and design coupling study
This repository contains the Matlab source code and results corresponding to the configuration trade study blog posts on  [vahana.aero](https://vahana.aero) which is modified to run ATC optimisation and design coupling studies. This study optimises the cost per flight or the overall mass of the system by modelling wing, gearbox, propeller and the motor. The following models are used for modelling these subsystems
1. Wing : Constant lift theory
2. Gearbox : torque based sizing
3. Motor : torque based sizing and efficiency-RPM-torque map
4. Propeller : Blade element momentum theory or actuator disk theory

Following are the design variables optimised in this study:

![design variables](https://github.com/chinthojuprajwal/vahanaTradeStudy-master/blob/main/design_var.png)

The detailed study is published in AIAA Scitech 2024 and is available for access at DOI https://doi.org/10.2514/6.2024-2235
The conference proceeding can also be downloaded from https://yonghoonlee.com/wp-content/uploads/2023/12/Chinthoju_2024_SciTech24_eVTOL.pdf

Code base : 

This repository has two different optimisation formulation implementations. The first one is a monolithic optimisation, which can be performed by calling sizingTradeStudy(range,payload) with a list of ranges in km and payload in kg. This optimises the model for the best cost per flight.

The second optimistation formulation is the ATC which can be performed by running the script EVTOL_ATC_coordination.

The computePerformance() method gives access to the entire model and outputs the cost per flight and all the constraints given the design variable array. This can furthur be used to perform other optimisation studies.
