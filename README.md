## IS-95A Simulation ##

This project provides a "simple" simulation of an IS-95A cellular network. It does not emulate full BTS to/from MS packet traffic or signaling. It does not simulate all the features of a full cellular network. The main goal was to compare BTS registration and handoff performance with a Fuzzy Logic controller that updates T_ADD and T_DROP parameters.


### TODO ###

 * fix RSS calculation - it's reporting between -150 and -250 dB which is just really low for 1000 meter radius.
 * break out into functions for better readability
 * check that fuzzy controller makes sense, document problems with author design (NOBS is always an integer)
 * plot metrics to compare non-Fuzzy to Fuzzy controller
