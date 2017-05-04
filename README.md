## IS-95A Simulation ##

This project provides a *simple* simulation of an IS-95A cellular network. It does not emulate full BTS to/from MS packet traffic or signaling. It does not simulate all the features of a full cellular network. The main goal was to compare BTS registration and handoff performance with a Fuzzy Logic controller that updates T_ADD and T_DROP parameters, based on the IEEE paper “A Novel Soft Handoff Algorithm for Fair Network Resources Distribution” by B. AliPanahi, M. Karzand.

Files:
	* IS95A_fuzzycontroller.m	- a function implementing the Fuzzy Inference System (FIS)
	* fuzzycontrollertest.m 	- a script to plot the IS95A_fuzzycontroller.m input/output Membership Functions and rule surface
	* IS95A_simulation.m		- the 'main' script to run the simulation, configuration parameters near the top of the file
	* generateBTS.m 			- a function to generate BTS locations/parameters
	* generateMS.m 				- a function to generate MS locations/parameters
	* updateMS.m 				- a function to update the MS locations/parameters
	* freeChannels.m 			- a helper function to properly free a list of BTS channels 

	
To run a test of the FIS and plot MF's and rule surface, run the fuzzycontrollertest.m script

To run a simulation, run the IS95A_simulation.m script


#### TODO ####
