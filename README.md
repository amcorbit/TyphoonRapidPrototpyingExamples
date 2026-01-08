Instructions for Inverter Control Operation
1. Install CCS v8.3.1
2. Import the Buck_Boost_Inverter_PI_v0.3.2 project
3. Build and clean the project
4. Launch debug session
5. Input expression variables for monitoring and control some examples of interest would be: Inv_OPMode, AC_Mag, AC_Freq_Fund, Sys_En, V_DCin
6. Launch Typhoon HIL following the instructions provided in the accompanying book chapter
7. Set the desired inverter operation mode in the expressions list. The operating mode can be changed by modifying the InvOpMode variable (0 = Passive Rectifier; 1 = Islanded; 2 = Grid-Forming; 3 = Grid-Feeding; 4 = Grid-Supporting).
8. Enable the PWN by setting the Sys_En variable to 3
9. Monitor output and check results in Typhoon HIL SCADA
   
