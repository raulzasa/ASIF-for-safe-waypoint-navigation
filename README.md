# ASIF for safe waypoint navigation
Here you will find the code developed for the ASIF research for a bicycle kinematic model applied to safe waypoint navigation. In addition, the author highlights the loose ends so that if anyone wants to continue the work, there is a starting point. 

## CODE
* Most of the code supports the main program called "dodecaedre_ok.m".  
* There is a study of the monotonicity that meets the delta interval resulting from monitoring based on the invariant set, as you can find it in "monotonia.m". 
* In "proves_intlab.m" you can find some tests usin the INTLAB library. Unfortunately, this library is neither open source nor available under the MATLAB academic licence.

## For those who want to continue...
In the course of the research, certain obstacles have been encountered which are merely possible starting points for further work. The most relevant of these are mentioned below:
* Test the current waypoint navigation in the laboratory for validation. 
* Define the invariant set analytically considering that the steering angle has dynamics.
* Include the state of the velocity, without the velocity being assumed constant.
* Computing the interval of control actions (delta) in interval form or by optimization.
* With the same idea in mind, design an ASIF for the longitudinal case. (It may help to initially consider platooning).
* Introduce another autonomous vehicle with the same ASIF and study their interaction. A further step is to include coordination strategies considering ASIFs to ensure safety. 
