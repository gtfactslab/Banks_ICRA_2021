# Banks_ICRA_2021
This code supplements the ICRA 2020 submission "Physical Human-UAV Interaction via Differentially Flat OutputGeneration using Admittance Control"

- Authors: Christopher Banks, Antonio Bono, Samuel Coogan
## Abstract:

In  this  paper,  we  propose  a  control  scheme 
thatallows  generic  commercial  multirotor  Unmanned 
Aerial	Vehi-cles  (UAVs)  to  infer  human  intent  from 
physical  interaction.Humans  apply  forces  by  touching 
it  or	pulling  a  nylon  stringattached  to  the  body 
frame  of  the	vehicle.  The  estimations  ofthese 
forces,  obtained  via	a  square  root  unscented 
Kalmanfilter,	are   used   by   an   admittance  
controller   to   generate   areference  trajectory  such 
that  the  vehicle  reacts  as	a  desiredmass-damper 
system.  To  track  this  trajectory,  the 
differentialflatness property of the multirotor dynamic
model is exploited.This,  in  fact,  allows  the 
generation  of	any  possible  kind  ofinput  required	by 
the  on  board	controller  supplied  by  themanufacturer. 
We  validate  our  results  experimentally  on	aParrot 
Bebop  2  quadrotor,  a  commercially  available  UAV. 

## Repository Contains:
- Scripts to run both the admittance controller generated trajectory for a  Bebop 2 quadcopter (including quadcopter dynamics) in simulation and experimenti
- Quadcopter Dynamics
- Script for control input generated from flat outputs of multi-rotor system
- Figures included in the paper
