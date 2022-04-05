# NEUP_utk
SMR model library
Dymola Package containing the models for the reactor module, balance of plant, and power plant system based on the NuScale design.

PowerPlant_testv4_multivalvectrl is the most up to date NPM model with PID controls implemented maintain a constant average core temperature and steam generator pressure.
A feedforward technique utilizing the feedwater pump massflowrate is used to control the reactor power.

Note: the NuScaleModule_v5 was adapted from the SMR Coupling test model in the NHES library developed at INL and contains some initialization transients at the start of the simulation until the 300 second mark approximately which may cause some issues if any changes are made to the models.
