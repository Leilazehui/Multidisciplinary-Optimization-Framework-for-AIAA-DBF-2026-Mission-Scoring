# README
## Background of AIAA DBF 2026
According to the competition rules of the AIAA DBF 2026, the flight objective of this year was to design, build, and test a banner towing bush plane, conduct charter flights to pay for the airplane in Mission 2, and start a banner towing business which was simulated in Mission 3. 

- Mission 1 (M1) is a delivery flight that demonstrates the aircraft’s flight capabilities without a payload, and the team has to fly 3 laps in a 5-minute time window.

- Mission 2 (M2) is a chartered flight that includes passengers and cargo, where the score will be added up and charged according to the number of passengers and cargoes carried.

- Mission 3 (M3) is a launch flight that demonstrates the stowing, towing, deployment and release of the onboard banner. 
Ground Mission (GM) is a timed demonstration of the aircraft assembly, payload installation, banner mechanism installation, and flight control checks. 

## Purpose of MDO
- A parametric **Multidisciplinary Optimization (MDO)** framework was developed in MATLAB using a deterministic grid-search approach to explore the design space systematically to obtain the predicted performance of the preliminary design that maximizes mission score while satisfying aerodynamic and performance constraints. 

- Several assumptions for the battery selection and empty weight of the aircraft under different conditions were made to provide a realistic scenario in the simulation, with an assumed velocity range based on previous aircraft performances. Then the flight path was estimated by examining the flight motion for three stages: take-off/climb, cruising, landing. 

- The highest possible score for each individual mission was estimated and was compared with the optimized score case where all missions’ requirements were considered to determine the highest score case for the GRIFFIN. For Gound Mission (GM), the time for the fastest possible case was estimated as 25 seconds, based on experience from the team.

## AIAA 2026 MDO Flowchart
<p align="center">
  <img src="https://github.com/Leilazehui/Multidisciplinary-Optimization-Framework-for-AIAA-DBF-2026-Mission-Scoring/blob/main/MDO_Flowchart.png"  with=50%  />
  <p/>
