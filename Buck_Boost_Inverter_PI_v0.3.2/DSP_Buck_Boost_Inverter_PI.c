/**
*   Engineers:          Anna Corbitt, Simone Di Bari, and Chris Farnell
*   Contact:            amcorbit@uark.edu
*   Company:            University of Arkansas
*   Website:            https://ncrept.uark.edu/
*
*   Create Date:        28Oct2021
*   Update Date:        08Jan2026
*   Design Name:        sdb_Typhoon_Buck_Boost
*   Project Version:    v0.3
*   Target Devices:     TMS320F28335
*   Hardware:           TI DIM100 DSP for Typhoon HIL
*   Tool Versions:      Code Composer Studio v8.1.3.00011
*                       ControlSuite v3.4.9
*                       Typhoon HIL Control Center v2023.3 sp1
*
*** Brief Description-
*   This project is developed as an educational first approach at programming a DSP to run with the Typhoon HIL software.
*   A buck converter, boost converter, and three phase inverter are simulated in Typhoon HIl and controlled by this DSP code.
*   This program creates the proper PWM signal based on a PI controller and user command input given through LabView.
*   This project part of a series of tutorials for learning to program DSPs.
*   In this program we will expand upon the previous tutorials to create Buck and Boost Converter along with a 3-Phase inverter.
*   All of the aforementioned converters have been modified from previous examples and now run closed-loop.
*   We have also updated the serial input packets to use Fixed-Point representation instead of being based on duty cycles.
*   Reference Parameters will be sent via serial communication ModBus RTU packets and will use 11Q5 Fixed-Point representation.
*   Controller PI Parameters will be sent using 6Q10 Representation for finer resolution.
*   We will also be implementing controls from the "FPUfastRTS" and the "DCL" libraries.
*
*
*** Operational Overview-
*   In this program we will expand on the previous "ADC_PWM_SPI_Serial" Example.
*   We will now incorporate PI control as well as IQ Fixed-Point References.
*   This Example uses I11Q5 Fixed-Point representation for inputs.
*   This allows us to send commands in a decimal format.
*   The 16-bit number sent is divided into two sections, an 11-bit section which represents the integer portion and a 5-bit section which represents the decimal portion.
*   For this example, there is no sign-bit as it is not needed for our purposes.
*   The 11IQ5 representation allows us to enter values from 2047 to 0 in 0.03125 increments.
*   The 6Q10 representation allows us to enter values from 63 to 0 in 0.0009766 increments.
*   The Buck and Boost converters use a simple PI Control method by implementing the Digital Logic Control (DCL) library provided by TI.
*   The 3-phase inverter uses a feed-forward control method which to maintain the output magnitude based on the input voltage.
*
*** Includes Information-
*   This section defines the Includes section for the project.
*   "${CG_TOOL_ROOT}/include"
*   "C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\include"
*   "C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_headers\include"
*   "C:\ti\controlSUITE\libs\math\FPUfastRTS\V100\include"
*   "C:\ti\controlSUITE\libs\control\DCL\v1_00_00_00\include"
*   "C:\ti\controlSUITE\libs\app_libs\solar\v1.2\float\include"
*   "C:\ti\controlSUITE\libs\math\IQmath\v160\include"
*   "${workspace_loc:/${ProjName}/Modbus_Headers}"
*
*
*** Dependencies Information-
*   This section defines the dependencies used for the project.
*   "28335_RAM_lnk.cmd" (For running in Debug Mode)         [C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\cmd]
*   "F28335.cmd" (For running in Flash Mode)                [C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\cmd]
*   "DSP2833x_ADC_cal.asm"                                  [C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\source]
*   "DSP2833x_Adc.c"                                        [C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\source]
*   "DSP2833x_CodeStartBranch.asm"                          [C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\source]
*   "DSP2833x_DefaultIsr.c"                                 [C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\source]
*   "DSP2833x_EPwm.c"                                       [C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\source]
*   "DSP2833x_GlobalVariableDefs.c"                         [C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_headers\source]
*   "DSP2833x_Headers_nonBIOS.cmd"                          [C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_headers\cmd]
*   "DSP2833x_MemCopy.c"                                    [C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\source]
*   "DSP2833x_PieCtrl.c"                                    [C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\source]
*   "DSP2833x_PieVect.c"                                    [C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\source]
*   "DSP2833x_Sci.c"                                        [C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\source]
*   "DSP2833x_SysCtrl.c"                                    [C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\source]
*   "DSP2833x_usDelay.asm"                                  [C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\source]
*   "DSP2833x_Spi.c"                                        [C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\source]
*   "DSP2833x_Mcbsp.c"                                      [C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\source]
*   "DSP2833x_ECap.c"                                       [C:\ti\controlSUITE\device_support\f2833x\v142\DSP2833x_common\source]
*
*   Floating Point Specific:
*   "atan_f32.asm"                      [C:\ti\controlSUITE\libs\math\FPUfastRTS\V100\source]
*   "atan2_f32.asm"                     [C:\ti\controlSUITE\libs\math\FPUfastRTS\V100\source]
*   "cos_f32.asm"                       [C:\ti\controlSUITE\libs\math\FPUfastRTS\V100\source]
*   "div_f32.asm"                       [C:\ti\controlSUITE\libs\math\FPUfastRTS\V100\source]
*   "isqrt_f32.asm"                     [C:\ti\controlSUITE\libs\math\FPUfastRTS\V100\source]
*   "sin_f32.asm"                       [C:\ti\controlSUITE\libs\math\FPUfastRTS\V100\source]
*   "sincos_f32.asm"                    [C:\ti\controlSUITE\libs\math\FPUfastRTS\V100\source]
*   "sqrt_f32.asm"                      [C:\ti\controlSUITE\libs\math\FPUfastRTS\V100\source]
*   "rts2800_fpu32_fast_supplement.lib" [C:\ti\controlSUITE\libs\math\FPUfastRTS\V100\lib]
*
*   IQMATH Specific:
*   "IQmath_fpu32.lib"                  [C:\ti\controlSUITE\libs\math\IQmath\v160\lib\IQmath_fpu32.lib] (Use FPU32 version for mixing IQmath with native floating-point code on devices with the C28x+FPU)
*
*
*   Modbus RTU Specific:
*   "ModbusData.c"
*   "ModbusDataHandler.c"
*   "ModbusSlave.c"
*   "Serial.c"
*   "Timer.c"
*   "CRC.c"
*
*
*   PI Control Specific:
*   "DCL_PI.asm"            [C:\ti\controlSUITE\libs\control\DCL\v1_00_00_00\source]
*
*   PLL Control Specific:
*   "ABC_DQ0_NEG_F.c"       [C:\ti\controlSUITE\libs\app_libs\solar\v1.2\float\source]
*   "ABC_DQ0_POS_F.c"       [C:\ti\controlSUITE\libs\app_libs\solar\v1.2\float\source]
*   "SPLL_3PH_SRF_F.c"      [C:\ti\controlSUITE\libs\app_libs\solar\v1.2\float\source]
*   "SPLL_3PH_DDSRF_F.c"    [C:\ti\controlSUITE\libs\app_libs\solar\v1.2\float\source]
*   "SPLL_1ph_SOGI_F.c"     [C:\ti\controlSUITE\libs\app_libs\solar\v1.2\float\source]
*   "SPLL_1ph_F.c"          [C:\ti\controlSUITE\libs\app_libs\solar\v1.2\float\source]
*
*
*   DataLogger Specific:
*   "DLOG_4CH_F.c"          [C:\ti\controlSUITE\libs\app_libs\solar\v1.2\float\source]
*
*
** PI Implementation [IMPORTANT]
*   We will need to include the DCL functions in the RAM by modifying the "28335_RAM_lnk.cmd" and "F28335.cmd" linker files by including the following line:
*
** The following should be added to the F28335.cmd file.
*  It loads the dclfuncs into flash and defines were it will be loaded into RAM.
*   //Updated by cfarnell for dclfuncs Flash Capability (PI Functions)
*   dclfuncs            : LOAD = FLASHE,                    // Load dclfuncs to Flash Block E
*                         RUN = RAML1,                      // Load dclfuncs to RAML3
*                         LOAD_START(_DclfuncsLoadStart),   // Defines Start
*                         LOAD_END(_DclfuncsLoadEnd),       // Defines End
*                         RUN_START(_DclfuncsRunStart),
*                         LOAD_SIZE(_DclfuncsLoadSize),
*                         PAGE = 0
*
*   After Modifying the F28335.cmd file you must also ensure that the following variables are defined and the code is in the main program.
*
*   ////External Functions
*   // For Ramfuncs
*   extern Uint16 RamfuncsLoadStart;    // Variable for MemCopy for loading data into flash.
*   extern Uint16 RamfuncsLoadEnd;      // Variable for MemCopy for loading data into flash.
*   extern Uint16 RamfuncsRunStart;     // Variable for MemCopy for loading data into flash.
*   //For Dclfuncs
*   extern Uint16 DclfuncsLoadStart;    // Variable for MemCopy for loading data into flash.
*   extern Uint16 DclfuncsLoadEnd;      // Variable for MemCopy for loading data into flash.
*   extern Uint16 DclfuncsRunStart;     // Variable for MemCopy for loading data into flash.
*
*
*  //// Debug or flash mode selection
*  This example defines the "_DEBUG" variable in the "Build Settings" for 'debug' Build Configuration and it is not defined for 'release' mode.
*  Using this we can define when to run the required pragma commands for loading the program into flash for release mode.
*  #ifdef _DEBUG
*      // do nothing, as default debug mode
*  #else
*      MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);        //For loading ISRs
*      MemCopy(&DclfuncsLoadStart, &DclfuncsLoadEnd, &DclfuncsRunStart);        //For Loading Dcl Functions
*      InitFlash();
*  #endif
*
*
*** For running only in Debug/RAM mode you only need to modify "28335_RAM_lnk.cmd" with the following lines:
*   //Added by cfarnell for PI functions
*   dclfuncs : > RAML1, PAGE = 0
*
*
*   We may also need to modify the RAM Block declarations such that the program will fit in the code space by modifying the following lines:
*
*   ////Update RAM Block to support Code Size (cfarnell)
*   //   RAML1      : origin = 0x009000, length = 0x001000    //on-chip RAM block L1
*   //   RAML2      : origin = 0x00A000, length = 0x001000
*   //   RAML3      : origin = 0x00B000, length = 0x001000
*   RAML1      : origin = 0x009000, length = 0x003000
*
*   The above lines combine the RAML1, RAML2, and RAML3 memory allocations into a single block.
*
*   ////Update RAM Block to support Code Size (cfarnell)
*   //RAML4       : origin = 0x00C000, length = 0x001000
*   //RAML5       : origin = 0x00D000, length = 0x001000
*   RAML4       : origin = 0x00C000, length = 0x002000
*   //The above lines combine the RAML4,and RAML5 memory allocations into a single block
*
*   ////Update RAM Block to support Code Size (cfarnell)
*   //RAML6      : origin = 0x00E000, length = 0x001000
*   //RAML7      : origin = 0x00F000, length = 0x001000
*   RAML6      : origin = 0x00E000, length = 0x002000
*   //The above lines combine the RAML6,and RAML7 memory allocations into a single block
*
*   Also need to comment out the following lines in F28335.cmd and/or the 28335_RAM_lnk.cmd since they are used to grow RAML4
*   //DMARAML5         : > RAML5,     PAGE = 1      //Commented since used by RAM L4
*   //DMARAML6         : > RAML6,     PAGE = 1       //Commented since used by RAM L4
*
*   Finally, the following are modified to support a larger stack for Modbus implementation and constants
*   // Allocate uninitialized data sections:
*   //.stack              : > RAMM1       PAGE = 1
*   .stack              : > RAML6       PAGE = 1
*   .ebss               : > RAML4       PAGE = 1
*   //.ebss               : > RAML1       PAGE = 0
*   .esysmem            : > RAMM1       PAGE = 1
*
**** Connected Peripheral Boards-
*   This section defines which Daughter/Peripheral Boards are used in the project and their relative locations.
*
*   IDC_A => PE-Eval PCB        [8 PWM, 8 DSP-ADC, 8 SPI-ADC (off-board)]
*   IDC_B => [Not Used]
*   IDC_C => [Not Used]
*   IDC_D => [Not Used]
*
*
*** Pin Assignments-
*   This section defines the Pin Assignments used for the project.
*
**  PWM:
*   PinD1 => GPIO0/EPWM1A =>    DSP_G1 (IGBT-Q1 Inverter)
*   PinD2 => GPIO1/EPWM1B =>    DSP_G2 (IGBT-Q2 Inverter)
*   PinD3 => GPIO2/EPWM2A =>    DSP_G3 (IGBT-Q3 Inverter)
*   PinE1 => GPIO3/EPWM2B =>    DSP_G4 (IGBT-Q4 Inverter)
*   PinE2 => GPIO4/EPWM3A =>    DSP_G5 (IGBT-Q5 Inverter)
*   PinE3 => GPIO5/EPWM3B =>    DSP_G6 (IGBT-Q6 Inverter)
*   PinF1 => GPIO6/EPWM4A =>    DSP_G7 (IGBT-Q7 Buck)
*   PinF2 => GPIO7/EPWM4B =>    DSP_G8 (IGBT-Q8 Boost)
*   PinF3 => GPIO8/EPWM5A =>    DSP_G9  (Not Used)
*   PinG1 => GPIO9/EPWM5B =>    DSP_G10 (Not Used)
*   PinG2 => GPIO10/EPWM6A =>   DSP_G9  (Not Used)
*   PinG3 => GPIO11/EPWM6B =>   DSP_G10 (Not Used)
*   PinN10 => GPIO24/ECAP1 =>   DSP_G13 (Not Used)      [APWM/ECAP]
*   PinM10 => GPIO25/ECAP2 =>   DSP_G14 (Not Used)      [APWM/ECAP]
*   PinN11 => GPIO27/ECAP4 =>   DSP_G15 (Not Used)      [APWM/ECAP]
*   PinL14 => GPIO48/ECAP5 =>   DSP_G16 (Not Used)      [APWM/ECAP]
*
**  Internal DSP ADCs:
*   // DSP Internal ADC-A (Routed to IDC-A)
*   PinM3 => ADCA0 => ADC-Vin
*   PinM2 => ADCA1 => ADC_Va
*   PinM1 => ADCA2 => ADC_Vb
*   PinL3 => ADCA3 => ADC_Vc
*   PinL2 => ADCA4 => ADC_DSP4 [V_Buck or DC_Link based on JP4 Position]
*   PinL1 => ADCA5 => ADC_VBoost
*   PinK2 => ADCA6 => ADC-Iin-3V (DC Input Current)
*   PinK1 => ADCA7 => ADC-VPot-3V (Potentiometer)
*
*   // DSP Internal ADC-B (Routed to IDC-B) [Not-Used]
*   PinP3 => ADCB0 => (Not Used)
*   PinN3 => ADCB1 => (Not Used)
*   PinM4 => ADCB2 => (Not Used)
*   PinN4 => ADCB3 => (Not Used)
*   PinM5 => ADCB4 => (Not Used)
*   PinN5 => ADCB5 => (Not Used)
*   PinM6 => ADCB6 => (Not Used)
*   PinN6 => ADCB7 => (Not Used)
*
**  SPI ADCs (AD7928):
*   // SPI_ADC0 located on PE-Eval PCB (DSP SPI Module Controlled)
*   VIN-0 => Iin
*   VIN-1 => I_L1
*   VIN-2 => I_L2
*   VIN-3 => I_L3
*   VIN-4 => I_Buck
*   VIN-5 => I_Boost
*   VIN-6 => SPI-ADC-Vin6 (Select Between Ia or I_L4 based on JP3 Jumper Position)
*   VIN-7 => I_L5
*
*   // SPI_ADC1 located at IDC_C on UCB_v1.4a    (Not Used)
*   VIN-0 => (Not Used)
*   VIN-1 => (Not Used)
*   VIN-2 => (Not Used)
*   VIN-3 => (Not Used)
*   VIN-4 => (Not Used)
*   VIN-5 => (Not Used)
*   VIN-6 => (Not Used)
*   VIN-7 => (Not Used)
*
*   // SPI_ADC2 located at IDC_D on UCB_v1.4a    (Not Used)
*   VIN-0 => (Not Used)
*   VIN-1 => (Not Used)
*   VIN-2 => (Not Used)
*   VIN-3 => (Not Used)
*   VIN-4 => (Not Used)
*   VIN-5 => (Not Used)
*   VIN-6 => (Not Used)
*   VIN-7 => (Not Used)
*
**  I/O:
*   PinB2 =>  GPIO31 => Red_Led1 (On DSP Control Card) [Active-Low]
*   PinA9 =>  GPIO34 => Red_Led2 (On DSP Control Card) [Active-Low]
*   PinG2 =>  GPIO10 => LED1 (D19-Green)
*   PinG3 =>  GPIO11 => LED2 (D20-Yellow)
*   PinH1 =>  GPIO12 => LED3 (D21-Orange)
*   PinH2 =>  GPIO13 => LED4 (D22-Red)
*   PinN10 => GPIO24 => LED5 (D23-Green)
*   PinM10 => GPIO25 => LED6 (D24-Yellow)
*   PinP11 => GPIO26 => LED7 (D25-Orange)
*   PinN11 => GPIO27 => LED8 (D26-Red)
*   PinF3 =>  GPIO08 => LED9 (Sw8-Green) [Active-Low]
*   PinG1 =>  GPIO09 => LED10 (Sw8-Red) [Active-Low]
*   PinH3 =>  GPIO14 => SW1
*   PinJ1 =>  GPIO15 => SW2
*   PinG12 => GPIO60 => CPLD PWM Enable along with GPIO 61
*   PinF14 => GPIO61 => CPLD PWM Enable along with GPIO 60
*
*
**  Serial_Comm:
*   PinD10 => GPIO28 => RX;         GPIO28 is SCI_A-RXD
*   PinC1  => GPIO29 => TX;         GPIO29 is SCI_A-TXD
*
**  SPI Communications:
*   // SPI Comm (ADC AD7928) [PE-Eval Board (SPI_ADC0) via DSP-SPI]
*   PinN8 =>    GPIO18 => SPICLK-A
*   PinJ2 =>    GPIO16 => SPISIMO-A
*   PinM8 =>    GPIO19 => SPISTE-A
*   PinJ3 =>    GPIO17 => SPISOMI-A
*
*   // SPI Comm (ADC AD7928) [CPLD Board (SPI_ADC1) via DSP-McBSP PortA] (Not Used for this Project)
*   PinM9 =>    GPIO22 => SPICLK-A
*   PinP9 =>    GPIO20 => SPISIMO-A
*   PinP10 =>   GPIO23 => SPISTE-A
*   PinN9 =>    GPIO21 => SPISOMI-A
*
*   // SPI Comm (ADC AD7928) [CPLD Board (SPI_ADC2) via DSP-McBSP PortB] (Not Used for this Project)
*   PinH3 =>    GPIO14 => SPICLK-A
*   PinH1 =>    GPIO12 => SPISIMO-A
*   PinJ1 =>    GPIO15 => SPISTE-A
*   PinH2 =>    GPIO13 => SPISOMI-A
*
*
*** Register Mapping
*   This section defines the Modbus Registers used for the project.
*
 //// 0-127 Command/Parameters (Write)
*   // System-Level Control/Config
*   Reg[0]   =>  System Enable Word              [Bitwise Operation]                             (Unit16)                    [Bit0=En_PWM; Bit1=En_Inverter; Bit2=En_Buck; Bit3=En_Boost;]
*   Reg[1]   =>  Reserved                        [TBD]                                           (TBD)
*   Reg[2]   =>  Reserved                        [TBD]                                           (TBD)
*   Reg[3]   =>  Reserved                        [TBD]                                           (TBD)
*   Reg[4]   =>  Reserved                        [TBD]                                           (TBD)
*   Reg[5]   =>  Reserved                        [TBD]                                           (TBD)
*   Reg[6]   =>  Reserved                        [TBD]                                           (TBD)
*   Reg[7]   =>  Reserved                        [TBD]                                           (TBD)
*   // Inverter Control/Config
*   Reg[8]   =>  Switching Freq (kHz)            [Range= 0-100  in 0.03125 increments]           (11Q5 Fixed-Point Format)
*   Reg[9]   =>  Inverter Op-Mode                [Range= 0-4]                                    (Uint16)                    [0=Passive; 1=Islanded; 2=Grid-Forming; 3=Grid-Feeding; 4=Grid-Supporting]
*   Reg[10]  =>  AC Magnitude Reference (VRMS)   [Range= 0-255  in 0.03125 increments]           (11Q5 Fixed-Point Format)
*   Reg[11]  =>  AC Frequency Reference (Hz)     [Range= 0-400  in 0.03125 increments]           (11Q5 Fixed-Point Format)
*   Reg[12]  =>  Active Power Reference(P)[PU]   [Range= 0-1    in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[13]  =>  Reactive Power Reference(Q)[PU] [Range= 0-1    in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[14]  =>  Inv_PLL_Kp                      [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[15]  =>  Inv_PLL_Ki                      [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[16]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[17]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[18]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[19]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[20]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[21]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[22]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[23]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[24]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[25]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[26]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[27]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[28]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[29]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[30]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[31]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   // Buck Control/Config
*   Reg[32]  =>  Switching Freq (kHz)            [Range= 0-100  in 0.03125 increments]           (11Q5 Fixed-Point Format)
*   Reg[33]  =>  Buck Op-Mode                    [Range= 0-4]                                    (Uint16)                    [0=Passive; 1=Voltage-Mode; 2=Current-Mode; 3=Dual-Loop]
*   Reg[34]  =>  Buck Reference Voltage          [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[35]  =>  Buck Reference Current          [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[36]  =>  Buck_V_Kp                       [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[37]  =>  Buck_V_Ki                       [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[38]  =>  Buck_I_Kp                       [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[39]  =>  Buck_I_Ki                       [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[40]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[41]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[42]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[43]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[44]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[45]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[46]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[47]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   // Boost Control/Config
*   Reg[48]  =>  Switching Freq (kHz)            [Range= 0-100  in 0.03125 increments]           (11Q5 Fixed-Point Format)
*   Reg[49]  =>  Boost Op-Mode                   [Range= 0-4]                                    (Uint16)                    [0=Passive; 1=Voltage-Mode; 2=Current-Mode; 3=Dual-Loop]
*   Reg[50]  =>  Boost Reference Voltage         [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[51]  =>  Boost Reference Current         [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[52]  =>  Boost_V_Kp                      [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[53]  =>  Boost_V_Ki                      [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[54]  =>  Boost_I_Kp                      [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[55]  =>  Boost_I_Ki                      [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[56]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[57]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[58]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[59]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[60]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[61]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[62]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   Reg[63]  =>  Reserved                        [Range= 0-63   in 0.0009766 increments]         (6Q10 Fixed-Point Format)
*   // Datalogger Control/Config
*   Reg[120] =>  Datalogger Chan1 Input Select   [Range= 0-128 (Reg_FP [Max])]                   (Unit16)
*   Reg[121] =>  Datalogger Chan2 Input Select   [Range= 0-128 (Reg_FP [Max])]                   (Unit16)
*   Reg[122] =>  Datalogger Chan3 Input Select   [Range= 0-128 (Reg_FP [Max])]                   (Unit16)
*   Reg[123] =>  Datalogger Chan4 Input Select   [Range= 0-128 (Reg_FP [Max])]                   (Unit16)
*   Reg[124] =>  Datalogger Chan1 Trigger (High) [Range= Floating Point]                         (Upper Portion of Float32)
*   Reg[125] =>  Datalogger Chan1 Trigger (Low)  [Range= Floating Point]                         (Lower Portion of Float32)
*   Reg[126] =>  Datalogger Prescaler            [Range= 0-65535 (Skip Count)]                   (Unit16)
*   Reg[127] =>  Datalogger Start                [Flag to Start Logging; 1=Start]                (Unit16)
*
*
*   //// 128-191 Raw ADC Data (Read) (Uint16)
*   // DSP ADC_A
*   Reg[128] => ADC_A0          (DC Input Voltage)          [PE-Eval PCB]
*   Reg[129] => ADC_A1          (VAC_PhA Output Voltage)    [PE-Eval PCB]
*   Reg[130] => ADC_A2          (VAC_PhB Output Voltage)    [PE-Eval PCB]
*   Reg[131] => ADC_A3          (VAC_PhC Output Voltage)    [PE-Eval PCB]
*   Reg[132] => ADC_A4          (V_Buck or DC_Link)         [PE-Eval PCB]       (Based on JP4 Position)
*   Reg[133] => ADC_A5          (Boost Output Voltage)      [PE-Eval PCB]
*   Reg[134] => ADC_A6          (DC Input Current)          [PE-Eval PCB]       (3V Scaling)
*   Reg[135] => ADC_A7          (Potentiometer)             [PE-Eval PCB]       (3V Scaling)
*   // DSP ADC_B
*   Reg[136] => ADC_B0          (Not Used)
*   Reg[137] => ADC_B1          (Not Used)
*   Reg[138] => ADC_B2          (Not Used)
*   Reg[139] => ADC_B3          (Not Used)
*   Reg[140] => ADC_B4          (Not Used)
*   Reg[141] => ADC_B5          (Not Used)
*   Reg[142] => ADC_B6          (Not Used)
*   Reg[143] => ADC_B7          (Not Used)
*   // SPI ADC_0 (SPI-ADC on PE-Eval_vv1.6a)
*   Reg[144] => SPI_ADC0_0      (DC Input Current)          [PE-Eval PCB]       (5V Scaling)
*   Reg[145] => SPI_ADC0_1      (I_L1)                      [PE-Eval PCB]
*   Reg[146] => SPI_ADC0_2      (I_L2)                      [PE-Eval PCB]
*   Reg[147] => SPI_ADC0_3      (I_L3)                      [PE-Eval PCB]
*   Reg[148] => SPI_ADC0_4      (I_Buck)                    [PE-Eval PCB]
*   Reg[149] => SPI_ADC0_5      (I_Boost)                   [PE-Eval PCB]
*   Reg[150] => SPI_ADC0_6      (Ia or I_L4)                [PE-Eval PCB]       (Based on JP3 Position)
*   Reg[151] => SPI_ADC0_7      (I_L5)                      [PE-Eval PCB]
*   // SPI ADC_1 (SPI-ADC IDC_C on UCB_v1.4a) [Not Used for this Project]
*   Reg[152] => SPI_ADC1_0      (Not Used)
*   Reg[153] => SPI_ADC1_1      (Not Used)
*   Reg[154] => SPI_ADC1_2      (Not Used)
*   Reg[155] => SPI_ADC1_3      (Not Used)
*   Reg[156] => SPI_ADC1_4      (Not Used)
*   Reg[157] => SPI_ADC1_5      (Not Used)
*   Reg[158] => SPI_ADC1_6      (Not Used)
*   Reg[159] => SPI_ADC1_7      (Not Used)
*   // SPI ADC_2 (SPI-ADC IDC_D on UCB_v1.4a) [Not Used for this Project]
*   Reg[160] => SPI_ADC2_0      (Not Used)
*   Reg[161] => SPI_ADC2_1      (Not Used)
*   Reg[162] => SPI_ADC2_2      (Not Used)
*   Reg[163] => SPI_ADC2_3      (Not Used)
*   Reg[164] => SPI_ADC2_4      (Not Used)
*   Reg[165] => SPI_ADC2_5      (Not Used)
*   Reg[166] => SPI_ADC2_6      (Not Used)
*   Reg[167] => SPI_ADC2_7      (Not Used)
*
*
*   //// 192-255 Status and Information (Read) (Uint16)
*   Reg[192]    => Status Word 1        // General Information (Future Use)
*   Reg[193]    => Status Word 2        // General Information (Future Use)
*   Reg[194]    => Status Word 3        // General Information (Future Use)
*   Reg[195]    => Status Word 4        // General Information (Future Use)
*   Reg[196]    => DL_Status            // Datalogger Status [1 = Waiting Trigger; 2=Collecting;]
*
*   //// 256-511 Calculated Parameters (Read)      128 Entries(32-Bit Floating Point) [Upper then Lower using Datacon.F_Data]
*   // Maps to Reg_FP[] Floating Point Array Referenced Below
*   DataCon.F_Data = Reg_FP[x];
*   mb.holdingRegisters.Mod_Reg[(i*2)+256] = DataCon.Reg_Data[1];   //Upper Word of Float
*   mb.holdingRegisters.Mod_Reg[(i*2)+257] = DataCon.Reg_Data[0];   //Lower Word of Float
*
*   // DSP-ADC_A Values Scaled and Converted to Floating Point Representation (Inverter)
*   Reg_FP[0] = V_DCin;
*   Reg_FP[1] = V_PhA;
*   Reg_FP[2] = V_PhB;
*   Reg_FP[3] = V_PhC;
*   Reg_FP[4] = V_Buck;
*   Reg_FP[5] = V_Boost;
*   Reg_FP[6] = I_DCin;
*   Reg_FP[7]=  V_Pot;
*   // DSP-ADC_B Values Scaled and Converted to Floating Point Representation (Future Use)
*   Reg_FP[8]=  0;                  //Future Use (ADCB0)
*   Reg_FP[9]=  0;                  //Future Use (ADCB1)
*   Reg_FP[10]=  0;                 //Future Use (ADCB2)
*   Reg_FP[11]=  0;                 //Future Use (ADCB3)
*   Reg_FP[12]=  0;                 //Future Use (ADCB4)
*   Reg_FP[13]=  0;                 //Future Use (ADCB5)
*   Reg_FP[14]=  0;                 //Future Use (ADCB6)
*   Reg_FP[15]=  0;                 //Future Use (ADCB7)
*   // SPI-ADC_0 Values Scaled and Converted to Floating Point Representation (PE-Eval PCB)
*   Reg_FP[16]= I_DCin2;            //Updated in SPI Interrupt
*   Reg_FP[17]= I_L1;               //Updated in SPI Interrupt
*   Reg_FP[18]= I_L2;               //Updated in SPI Interrupt
*   Reg_FP[19]= I_L3;               //Updated in SPI Interrupt
*   Reg_FP[20]=I_Buck;              //Updated in SPI Interrupt
*   Reg_FP[21]=I_Boost;             //Updated in SPI Interrupt
*   Reg_FP[22]=Ia;                  //Updated in SPI Interrupt
*   Reg_FP[23]=I_L5;                //Updated in SPI Interrupt
*   // SPI-ADC_1 Values Scaled and Converted to Floating Point Representation (Future Use)
*   Reg_FP[24]= 0;                  //Future Use (SPI-ADC_1-V0)
*   Reg_FP[25]= 0;                  //Future Use (SPI-ADC_1-V1)
*   Reg_FP[26]= 0;                  //Future Use (SPI-ADC_1-V2)
*   Reg_FP[27]= 0;                  //Future Use (SPI-ADC_1-V3)
*   Reg_FP[28]= 0;                  //Future Use (SPI-ADC_1-V4)
*   Reg_FP[29]= 0;                  //Future Use (SPI-ADC_1-V5)
*   Reg_FP[30]= 0;                  //Future Use (SPI-ADC_1-V6)
*   Reg_FP[31]= 0;                  //Future Use (SPI-ADC_1-V7)
*   // SPI-ADC_2 Values Scaled and Converted to Floating Point Representation (Future Use)
*   Reg_FP[32]= 0;                  //Future Use (SPI-ADC_2-V0)
*   Reg_FP[33]= 0;                  //Future Use (SPI-ADC_2-V1)
*   Reg_FP[34]= 0;                  //Future Use (SPI-ADC_2-V2)
*   Reg_FP[35]= 0;                  //Future Use (SPI-ADC_2-V3)
*   Reg_FP[36]= 0;                  //Future Use (SPI-ADC_2-V4)
*   Reg_FP[37]= 0;                  //Future Use (SPI-ADC_2-V5)
*   Reg_FP[38]= 0;                  //Future Use (SPI-ADC_2-V6)
*   Reg_FP[39]= 0;                  //Future Use (SPI-ADC_2-V7)
*   // Calculated Values
*   Reg_FP[40] = V_AB;
*   Reg_FP[41] = V_BC;
*   Reg_FP[42] = V_CA;
*   Reg_FP[43] = Sine1;
*   Reg_FP[44] = Sine2;
*   Reg_FP[45] = Sine3;
*   Reg_FP[46] = Step_sin;
*   Reg_FP[47] = spll1.theta[0];
*   // Average and RMS Values
*   Reg_FP[48] = Avg_V_DC_In;
*   Reg_FP[49] = Avg_I_DC_In;
*   Reg_FP[50] = Avg_V_Buck;
*   Reg_FP[51] = Avg_I_Buck;
*   Reg_FP[52] = Avg_V_Boost;
*   Reg_FP[53] = Avg_I_Boost;
*   Reg_FP[54] = Avg_F_Grid;
*   Reg_FP[55] = RMS_V_AB;
*   Reg_FP[56] = RMS_V_BC;
*   Reg_FP[57] = RMS_V_CA;
*
*   //// 512-2047 Datalogging              4-Channels with 192 Floating Point values per channel (32-Bit Floating Point) [Upper then Lower using Datacon.F_Data]
*   Reg[512-895]    => Channel_1        Channel_1 Data
*   Reg[896-1279]   => Channel_2        Channel_2 Data
*   Reg[1280-1663]  => Channel_3        Channel_3 Data
*   Reg[1664-2047]  => Channel_4        Channel_4 Data
*
*
*** ADC and Voltage Scaling:
*   In this project we will use an ADC and Op-amps, for scaling voltages, to provide feedback.
*   The resistors used for scaling are a 1 MOhm resistor and a 31.6 kOhm, or 13.7, kOhm resistor.
*   The ADC used is an AD7928 which is an 8-Channel, 12-bit Device.
*   Hall-Effect current sensors, ACS714LLCTR-05B-T, are used in this design.
*   Scaling for these sensors is 185mV/A.
*   The Phase_A, Phase_B, and Phase_C voltages are referenced to the Power Ground, as such they are scaled with a 13.7 kOhm resistor and level shifted using a 0.9Vdc reference.
*   For data processing purposes the differential voltages Vab, Vbc, and Vca are used which involves subtracting the Va, Vb, and Vc values accordingly.
*
*   The equation used for the voltage scaling from the Op-amps and resistors is as follows:
*   V_ADC = (31.6k / (31.6k + 1M))*V_Measured; V_ADC is the voltage at the input of the ADC and V_Measured is the voltage to be measured.
*   ADC_Digital = V_ADC * (2^12/5V); ADC_Digital is the digital representation of the voltage as given by the ADC output.
*   Therefore, if a voltage of 50V is applied to the circuit than the voltage at the ADC input will be approximately [(31.6k / (31.6k + 1M))*50V]=1.5316V.
*   The voltage will then be transformed into a digital representation by the ADC which results in [1.5316V * (2^12/5V)]=1255.
*   We will use these representations through-out the code such that we don't make it overly complex.
*
*   Equations:
*   Converting from the digital to analog representation is as follows:
*   For 5V, 12-bit ADC (AD7928):
*   V_Measured = V_ADC/[(31.6k / (31.6k + 1M))] = [ADC_Digital*(5V/2^12)] / [(31.6k / (31.6k + 1M))]
*   For 3V, 12-bit ADC (TI DSP):
*   V_Measured = V_ADC/[(31.6k / (31.6k + 1M))] = [ADC_Digital*(3V/2^12)] / [(31.6k / (31.6k + 1M))]
*
**  SPI ADC Description
*   Register details for ADC7928 (12-bit; 8-channel SPI ADC) \n
*   Control: 12 bits wide, first 12 of 16 bits used \n
*
*    |write|SEQ|don'tcare|ADD2|ADD1|ADD0|PM1|PM0 |SHADOW|DC|RANGE|CODING|
*    | 11  |10 | 9       | 8  | 7  | 6  | 5 | 4  |  3   |2 |  1  |  0   |
*    |1=wr |n1 |         |ch to convert |pwr mode| n1   |  | n2  | n3   |
*
*   n1: selects the mode of channel sampling 00= addressed channel only
*   n2: 0 = 0v to 2*REFin, 1 = 0v to REFin
*   n3: 0 = two's comp output, 1 = raw binary
*   Sequence register is 16 bits (2*8), order of channel sampling
*   #################################################################################
*   --X"831F";      -- Command to read ADC Channel 0
*   --X"FFDF";      -- Command to read All ADC Channels Continuously
*   --X"E3DF";      -- Command to read ADC Channel 0 Continuously
*   --X"E7DF";      -- Command to read ADC Channel 1+2 Continuously
*   --X"FFDF";      -- Command to read All ADC Channels Continuously
*   --X"871F";      -- Command to read ADC Channel 1
*   --X"8B1F";      -- Command to read ADC Channel 2
*   --X"8F1F";      -- Command to read ADC Channel 3
*   --X"931F";      -- Command to read ADC Channel 4
*   --X"971F";      -- Command to read ADC Channel 5
*   --X"9B1F";      -- Command to read ADC Channel 6
*   --X"9F1F";      -- Command to read ADC Channel 7
*   --X"FFFF";      -- Dummy Command for startup
*
*
***  Modbus Serial Information:
*
*   Serial Settings:
*   Baud         115200
*   Parity       None
*   Stop Bits    2
*
*   Implementation Notes from Original Code:
*   Normally this code will be implemented at main.c
*   Include the ModbusSlave.h header
*   Change the configuration files: a. ModbusSettings.h: baudrate, parity, debug modes, buffer size (the max size of a frame)...
*   b. Serial.c & Timer.c: if your CLOCK or LSPCLK are different from 150 e 37500000, change those files
*   Declare the modbus object with ModbusSlave mb;
*   Initialize modbus object with mb = construct_ModbusSlave();
*   At some loop, call modbus object with mb.loopStates(&mb);
*
*   Using Memory Map:
*   To use the memory map you will need to change the ModbusDataMap.h file with the variables that you want.
*   Don't forget that coils/inputs are Uint8 values! By default, the holding registers are meant to be 32 bits (long or float).
*   If you want to use 16 bits variables you can enable that disabling MB_32_BITS_REGISTERS on ModbusSettings.h.

*   Observation:
*   The data map is specified at ModbusDataMap.h but if you want to hold different data maps and just change them
*   before compile the code, you can change the MB_DATA_MAP constant at ModbusSettings.h. By default it is ModbusDataMap.h.
*
*   Notes (cfarnell)-
*   Modbus Headers saved to "Modbus_Headers" folder in workspace... must include folder in "Includes".
*   This version was tested and works with LabVIEW Master Modbus implementation.
*   16-bit Modbus Holding Registers are not offset properly... starting address is 0.
*
*   //Modbus edits by cfarnell for this application
*   Updated ModbusDataMap.h for 2048 registers
*   Updated all instances of "char" Pointers to "Uint16" in ModbusDataHandler.c to cope with registers greater than 96.
*   "char" was too small for use as pointer for Modbus Registers
*   Also, you must initialize the Modbus array before initializing any interrupts that might right to them.
*   Finally, for large implementations (Greater than approximately 96 registers) the stack must be increased.
*   For this example we increased the stack from 0x300 to 0x1800 in order to support the Modbus implementation.
*   This required us to assign the stack to RAML6 from RAMM1 since RAMM1 was much too small.
*   The stack can be found under "Build Settings -> C2000 Linker -> Basic Options".
*   //.stack           : > RAMM1,     PAGE = 1
*   .stack           : > RAML6,     PAGE = 1
*
*
*
*
*** Revision History-
*   0.3.2 Updated Register Mapping for more consistency across projects. Also updated for compatibility with UCB 1.4a.
*   0.3.1 Major Revision, Updated to include "Supplemental_Files" Folder and to use version "UCB_PE-EVAL_v1.6a" Hardware w/ Level Shifting and Datalogging. (4Apr2020)
*         Additional Updates include: Voltage Scaling, Datalogging, RMS, Modbus RTU (2048 Registers) for Serial Comms, Interrupt-based SPI-ADC, Additional ADC inputs, and updated register mapping.
*         "Supplemental_Files" includes Schematics, LabVIEW Interface, and Simulations.
*         Also updated notes, linker files for memory allocation, and added _DEBUG Flag to allow for easy transition from 'debug' to 'release'.
*   0.2.3 Updated "F28335.cmd" and "DSP_Buck_Boost_Inverter_PI.c" to allow for running code in flash [ramfuncs and dclfuncs] (11Jun2019)
*   0.2.2 Updated Serial Interface to allow two bytes for Start Address and ensure overall compatibility with VHDL Implementation (9Jun2019)
*   0.2.1 Major Revision, Removed Doxygen Tags and updated for new boards. (21Oct2018)
*   0.1.0 Updated Documentation again for tutorials (22Nov2015)
*   0.0.4 Updated Documentation for use with Doxygen
*   0.0.3 Updated Documentation
*   0.0.2 First Stable Release
*
*/


////// Defines

// General Purpose
#define Freq_Clk        150e6           // Clock Frequency (150 MHz)
#define Inv_Freq_Sw     30e3            // Set Switching Frequency (30 kHz)
#define Buck_Freq_Sw    30e3            // Set Switching Frequency (30 kHz)
#define Boost_Freq_Sw   30e3            // Set Switching Frequency (30 kHz)
#define Inv_AC_Mag      24.0            // Set Peak Output AC Magnitude Reference in Fixed Point (24 V)
#define Inv_AC_Freq     60.0            // Set Fundamental Output Frequency in Fixed Point (60 Hz)
#define Inv_AC_P        0.0             // Inverter Active Power Reference
#define Inv_AC_Q        0.0             // Inverter Active Power Reference
#define IQ5_STEP        0.03125         // Step size for IQ5 fixed point format (2^-5)
#define IQ10_STEP       0.00097         // Step size for IQ10 fixed point format (2^-10)
#define DLOG_Buff       192             // Size of Datalogger buffers
#define Reg_FP_Max      128             // Max Index for Reg_FP (used for Datalogger)
//#define Deadtime        30              // Deadtime; 30 clk cycles = (150e6^-1)*30 => 200ns
#define Deadtime        50              // Deadtime; 50 clk cycles = (150e6^-1)*50 => 333ns
#define pi              3.1415926535898 // PI
#define pi_2            6.2831853071796 // 2*PI
#define phase_offset    2.0943951023932 // Phase Shift for Sinewave Generator =(2.0/3.0)*pi; (120 Degrees Offset for 3-Phase)
//#define phase_offset    3.1415926535898 // Phase Shift for Sinewave Generator =(2.0/2.0)*pi; (180 Degrees Offset for Split Phase)
#define SPI_Wakeup      0xFFFF          // Wakeup and Configure Command for SPI ADC
#define SPI_Cmd         0xFFDF          // SPI command for ADC continuous read
#define SPI_Dummy       0x0000          // Dummy Command for SPI ADC
#define Vac_Peak_d      66.0           // Peak AC Voltage output, used for normalizing input for PLL

//// PI Initial Parameters
// Inverter
#define Inv_PLL_Kp      1.0             // Inv PLL PI Kp Parameter
#define Inv_PLL_Ki      1.0             // Inv PLL PI Ki Parameter
// Buck Converter
#define Buck_V_Kp       1.0             // BESS VBatt PI Kp Parameter
#define Buck_V_Ki       0.01            // BESS VBatt PI Ki Parameter
#define Buck_I_Kp       0.0             // BESS VBatt PI Kp Parameter
#define Buck_I_Ki       0.0             // BESS VBatt PI Ki Parameter
// Boost Converter
#define Boost_V_Kp      1.0             // BESS IBatt PI Kp Parameter
#define Boost_V_Ki      0.01            // BESS IBatt PI Ki Parameter
#define Boost_I_Kp      0.0             // BESS IBatt PI Kp Parameter
#define Boost_I_Ki      0.0             // BESS IBatt PI Ki Parameter

// Command Limits
#define FreqPWM_Min     5e3             // Minimum Switching Frequency
#define FreqPWM_Max     100e3           // Maximum Switching Frequency
#define Buck_Cmd_Min    0               // Minimum Buck Command Value
#define Buck_Cmd_Max    50              // Maximum Buck Command Value
#define Boost_Cmd_Min   0               // Minimum Boost Command Value
#define Boost_Cmd_Max   50              // Maximum Boost Command Value
#define AC_Mag_Min      0               // Minimum AC Magnitude Value
#define AC_Mag_Max      50              // Maximum AC Magnitude Value
#define AC_Freq_Fund_Min 0.1             // Minimum AC Fundamental Frequency Value
#define AC_Freq_Fund_Max 1e3             // Maximum AC Fundamental Frequency Value


// Scaling
#define ADC_Scale_DSP       0.00073 // DSP ADC Scaling factor (3/4095)
#define ADC_Scale_SPI       0.00122 // DSP ADC Scaling factor (5/4095)
#define I_Scale_ACS714_05B  5.40541 // Scaling factor for Current Sensor (185mV/A)^-1   [ACS714LLCTR-05B-T]
#define R_Div_Scale1        32.6456 // (31.6e3/(1e6+31.6e3))^-1 (Used for ADC Signals (Vin, V_ADC4[V_Buck or DC_Link], and V_Boost)
#define R_Div_Scale2        73.9927 // (13.7e3/(1e6+13.7e3))^-1 (Used for ADC Signals (Va,Vb,and Vc)
#define R_Div_Scale3        1.66667 // (15e3/(10e3+15e3))^-1 (Used for level-shifting ADC signal Iin and VPot from 5V to 3V)
#define V_Scale1            0.02392 // ADC_Scale_DSP * R_Div_Scale1 (Used for ADC Signals (Vin, V_ADC4[V_Buck or DC_Link], and V_Boost)
#define V_Scale2            0.05421 // ADC_Scale_DSP * R_Div_Scale2 (Used for ADC Signals (Va,Vb,and Vc)
#define V_Offset_ABC        66.5934 // 0.9V * R_Div_Scale2; Offset from 0.9V reference used for level shift (Used for ADC Signals (Va,Vb,and Vc)
#define I_Scale1            0.00396 // ADC_Scale_DSP * I_Scale_ACS714_05B; Current scaled to 3V signal (Used for ADC Signals (I_DCin)
#define I_Scale2            0.00660 // ADC_Scale_SPI * I_Scale_ACS714_05B (Used for ADC Signals (I_DCin2,I_L1,I_L2,I_L3, I_Buck,I_Boost,(I_L4 or Ia),and I_L5 )
#define I_Offset_ACS714_05B 13.5135 // 2.5V * I_Scale_ACS714-05B; Offset from device, 2.5 V = 0A


/*
//// For Larger-Scale Projects using LEM Voltage/Current Sensors and Thermocouples (Left here for future Reference)
/// For Voltage Scaling
// 25 mA RMS from LEM Sensor (Scaled from 500/1000/1500 Vrms depending on resistor configuration)
// Using 1500 Vrms scaled to 25 mA Irms in BAPS configuration
// Then level shifted and scaled to a 0 - 3V signal for 12-bit ADC
// ((ADC_Read * ADC_Scale_DSP) - V_Offset_LEM) * V_Scale_LEM

/// For Current Scaling
// Using the LEM LF 2005-S/SP23 which has a +/- 2000 A maximum and a scaling ration of 1:5000.
// A burden resistor of 6.19 Ohm converts this +/- 400 mA output to +/- 2.476 V.
// This voltage is then scaled from +/- 2.476 V to 0 V to 3 V.
// ((ADC_Read * ADC_Scale_DSP) - I_Offset_LEM) * I_Scale_LEM

/// For K-Type Thermocouple Scaling
// Vout = (3/5) *  [(TMJ x (5 mV / C)) + Vref]; TMJ = Junction Temp, Vref = 2.5 V; Voltage scaled from 5 V to 3 V output with 1.5 V midpoint.
// (((ADC_Read * ADC_Scale_SPI) * (5/3) ) - T_Offset_Therm) * T_Scale_Therm ; The (5/3) factor is to account for 5 V to 3 V scaling.

#define V_Offset_LEM        1.5     // Offset voltage for ADC Scaling
//#define V_Scale_LEM       952.33  // Scaling Coefficient for LEM LF LV 25-P/SP5 Voltage Sensors (140 Ohm Burden Resistor)
#define V_Scale_LEM         1428.5  // Scaling Coefficient for LEM LF LV 25-P/SP5 Voltage Sensors (140 Ohm Burden Resistor)
#define I_Offset_LEM        1.5     // Offset voltage for ADC Scaling
#define I_Scale_LEM         2692.51 // Scaling Coefficient for LEM LF 2005-S/SP23 Current Sensors (6.19 Ohm Burden Resistor)
#define T_Offset_Therm      2.5     // Offset voltage for ADC Scaling
#define T_Scale_Therm       200     // 1 degree C / 5 mV

*/


// Includes
#include "DSP28x_Project.h"
#include "math.h"
#include "C28x_FPU_FastRTS.h"
#include "DCL.h"
#include "Solar_F.h"
#include "ModbusSlave.h"
#include "IQmathLib.h"


// ADC start parameters
#if (CPU_FRQ_150MHZ)     // Default - 150 MHz SYSCLKOUT
  #define ADC_MODCLK 0x3 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz
#endif
#if (CPU_FRQ_100MHZ)
  #define ADC_MODCLK 0x2 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 100/(2*2)   = 25.0 MHz
#endif
#define ADC_CKPS   0x1   // ADC module clock = HSPCLK/2*ADC_CKPS   = 25.0MHz/(1*2) = 12.5MHz


////Prototype statements for functions
void Gpio_Init(void);       // Initialize GPIO Outputs and Inputs
void ADC_Init(void);        // Initialize ADC
void InitEPWM1(void);       // Initialize EPWM1 Module
void InitEPWM2(void);       // Initialize EPWM2 Module
void InitEPWM3(void);       // Initialize EPWM3 Module
void InitEPWM4(void);       // Initialize EPWM4 Module
void InitEPWM5(void);       // Initialize EPWM5 Module
void InitEPWM6(void);       // Initialize EPWM6 Module
void InitAPWM(void);        // Initialize ECAP Module for PWM Operation (ECAP 1-4)
void InitSPI(void);         // Initialize SPI for SPI Master Operation (SPI ADC0)
void InitMcBSP(void);       // Initialize McBSP Ports A & B for SPI Master Operation (SPI ADC1 & ADC2)
void InitFlash();           // Initialize Flash Module
void MemCopy();             // Flash Memory Copy function

////Interrupts
interrupt void adc_isr(void);       // Interrupt routine for ADC
interrupt void spiTxFifoIsr(void);  // Interrupt for SPI ADC TX
interrupt void spiRxFifoIsr(void);  // Interrupt for SPI ADC RX
interrupt void Mcbsp_TxINTA_ISR(void);  // Interrupt for McBSP Port-A TX
interrupt void Mcbsp_RxINTA_ISR(void);  // Interrupt for McBSP Port-A RX
interrupt void Mcbsp_TxINTB_ISR(void);  // Interrupt for McBSP Port-B TX
interrupt void Mcbsp_RxINTB_ISR(void);  // Interrupt for McBSP Port-B RX

////External Functions
#ifdef _DEBUG
////debug mode
#else
// For Ramfuncs
extern Uint16 RamfuncsLoadStart;    // Variable for MemCopy for loading data into flash.
extern Uint16 RamfuncsLoadEnd;      // Variable for MemCopy for loading data into flash.
extern Uint16 RamfuncsRunStart;     // Variable for MemCopy for loading data into flash.
//For Dclfuncs
extern Uint16 DclfuncsLoadStart;    // Variable for MemCopy for loading data into flash.
extern Uint16 DclfuncsLoadEnd;      // Variable for MemCopy for loading data into flash.
extern Uint16 DclfuncsRunStart;     // Variable for MemCopy for loading data into flash.
//For .econst
extern Uint16 EconstLoadStart;    // Variable for MemCopy for loading data into flash.
extern Uint16 EconstLoadEnd;      // Variable for MemCopy for loading data into flash.
extern Uint16 EconstRunStart;     // Variable for MemCopy for loading data into flash.
//For FPUmathTables
extern Uint16 FPUmathTablesLoadStart;    // Variable for MemCopy for loading data into flash.
extern Uint16 FPUmathTablesLoadEnd;      // Variable for MemCopy for loading data into flash.
extern Uint16 FPUmathTablesRunStart;     // Variable for MemCopy for loading data into flash.
//For .Cinit
//extern Uint16 CinitLoadStart;    // Variable for MemCopy for loading data into flash.
//extern Uint16 CinitLoadEnd;      // Variable for MemCopy for loading data into flash.
//extern Uint16 CinitRunStart;     // Variable for MemCopy for loading data into flash.
////For .Text
//extern Uint16 TextLoadStart;    // Variable for MemCopy for loading data into flash.
//extern Uint16 TextLoadEnd;      // Variable for MemCopy for loading data into flash.
//extern Uint16 TextRunStart;     // Variable for MemCopy for loading data into flash.
#endif




#ifdef _DEBUG
////debug mode
#else
#pragma CODE_SECTION(adc_isr, "ramfuncs"); // flash mode
#pragma CODE_SECTION(spiTxFifoIsr, "ramfuncs"); // flash mode
#pragma CODE_SECTION(spiRxFifoIsr, "ramfuncs"); // flash mode
//#pragma CODE_SECTION(SPLL_3ph_DDSRF_F_FUNC, "ramfuncs"); // flash mode
#pragma CODE_SECTION(SPLL_3ph_SRF_F_FUNC, "ramfuncs"); // flash mode
#pragma CODE_SECTION(ABC_DQ0_POS_F_FUNC, "ramfuncs"); // flash mode
#pragma CODE_SECTION(ABC_DQ0_NEG_F_FUNC, "ramfuncs"); // flash mode
#pragma CODE_SECTION(DLOG_4CH_F_FUNC, "ramfuncs"); // flash mode
#pragma CODE_SECTION(Mcbsp_TxINTA_ISR, "ramfuncs"); // flash mode
#pragma CODE_SECTION(Mcbsp_RxINTA_ISR, "ramfuncs"); // flash mode
#pragma CODE_SECTION(Mcbsp_TxINTB_ISR, "ramfuncs"); // flash mode
#pragma CODE_SECTION(Mcbsp_RxINTB_ISR, "ramfuncs"); // flash mode
#endif





////Global variables
Uint16 ADC_DSP[16];               //  ADC Register Values
ModbusSlave mb;                 // Modbus Variables and Registers (Struct)
float Prd_Sw;                   // Switching cycles per fundamental period Ex:(30 kHz / 60Hz = 500)
int i;                          // Generic Counter variable
Uint16 Cmd_Temp;                // Temporary Command Variable for Modbus Commands in Main Loop
float Cmd_FTemp;                // Temporary Command Variable for Modbus Commands in Main Loop

////ADC Values (Real-World Float)
//DSP ADC Values
float V_DCin;               // Converted ADC Value for VDC Input
float V_PhA;                // Converted ADC Value for Phase_A for 3-phase inverter
float V_PhB;                // Converted ADC Value for Phase_B for 3-phase inverter
float V_PhC;                // Converted ADC Value for Phase_C for 3-phase inverter
float V_Buck;               // Converted ADC Value for Buck
float V_Boost;              // Converted ADC Value for Boost
float I_DCin;               // Converted ADC Value for IDC input [Scaled to 3V]
float V_Pot;                // Converted ADC Value for Potentiometer [Scaled to 3V]
float V_AB, V_BC, V_CA;     // Line to Line Values for Voltages

//SPI ADC Values
float I_DCin2;  // Converted SPI ADC Value for VDC Current Input
float I_L1;     // Converted SPI ADC Value for Phase_A for 3-phase inverter
float I_L2;     // Converted SPI ADC Value for Phase_B for 3-phase inverter
float I_L3;     // Converted SPI ADC Value for Phase_C for 3-phase inverter
float I_Buck;   // Converted SPI ADC Value for Buck
float I_Boost;  // Converted SPI ADC Value for Boost
float Ia;       // Converted SPI ADC Value for Ia or I_L4 Current [Jumper JP3]
float I_L5;     // Converted SPI ADC Value for Potentiometer



//Temp (Delete Later)
float I_Ind, I_XFMR_Pri1, I_XFMR_Pri2;
long TempCnt;


//// SPI ADC Variables
//SPI ADC_0 (Using DSP SPI Module and SPI Located on PE-Eval Board) [Can read 8 Channels per Frame]
Uint16 SPI_ADC0_rdata[8];   // Array for saving SPI ADC Data
Uint16 SPI_ADC0_temp;       // Temp received data from SPI
int SPI_ADC0_Cmd_Cnt =0;    //Counter for indexing SPI Commands for SPI ADC

//SPI ADC_1 (Using McBSP_Port-A in SPI Configuration and UCB Board's ADC1) [Can read 1 Channel per Frame]
Uint16 SPI_ADC1_rdata[8];   // Array for saving SPI ADC Data
Uint16 SPI_ADC1_temp;       // Temp received data from SPI
int SPI_ADC1_Cmd_Cnt =0;    //Counter for indexing SPI Commands for SPI ADC

//SPI ADC_2 (Using McBSP_Port-B in SPI Configuration and UCB Board's ADC2) [Can read 1 Channel per Frame]
Uint16 SPI_ADC2_rdata[8];   // Array for saving SPI ADC Data
Uint16 SPI_ADC2_temp;       // Temp received data from SPI
int SPI_ADC2_Cmd_Cnt =0;    //Counter for indexing SPI Commands for SPI ADC


//// Command Variables
int Sys_En;

// Inverter Variables
float INV_FreqPWM;
Uint16 Inv_OPMode;  // Operating Mode for Inverter  [0=Passive Rectifier; 1=Islanded; 2=Grid-Forming; 3=Grid-Feeding; 4=Grid-Supporting]
float AC_Mag;       // AC Magnitude reference (V)
float AC_Freq_Fund; // AC Fundamental Frequency Ref (Hz)
float Step_sin;     // Used in calculating the sin output for 3-phase inverter; Step_sin = (Sin_Counter/Prd_Sw)*pi_2;
float Sine1;        // Duty to generate Duty Cycle for 3-Phase PWM
float Sine2;        // Duty to generate Duty Cycle for 3-Phase PWM
float Sine3;        // Duty to generate Duty Cycle for 3-Phase PWM
float Duty_AC;      // Duty Cycle for 3-Phase PWM; Based on AC Magnitude reference and PWM Frequency.
Uint16 Sin_Counter; // Used to calculate the value of sin output; Based on number of interrupts

// Buck Converter Variables
float Buck_FreqPWM;
Uint16 Buck_OPMode;         // [0=Passive; 1=Voltage-Mode; 2=Current-Mode; 3=Dual-Loop]
float Buck_V_Cmd;
float Buck_I_Cmd;
PI Buck_V_PI = PI_DEFAULTS;         // Buck PI Voltage Control Variables
float Buck_V_PI_uk;                 // Buck PI Voltage Control Output
PI Buck_I_PI = PI_DEFAULTS;         // Buck PI Current Control Variables
float Buck_I_PI_uk;                 // Buck PI Current Control Output


// Boost Converter Variables
float Boost_FreqPWM;
Uint16 Boost_OPMode;         // [0=Passive; 1=Voltage-Mode; 2=Current-Mode; 3=Dual-Loop]
float Boost_V_Cmd;
float Boost_I_Cmd;
PI Boost_V_PI = PI_DEFAULTS;         // Boost PI Voltage Control Variables
float Boost_V_PI_uk;                 // Boost PI Voltage Control Output
PI Boost_I_PI = PI_DEFAULTS;         // Boost PI Current Control Variables
float Boost_I_PI_uk;                 // Boost PI Current Control Output


//// For PWM Switching Freq
Uint16 INV_PWM_TBPRD;           //(1/INV_FreqPWM)/(2*(1/Freq_Clk))-1;
Uint16 Conv_PWM_TBPRD;          //(1/Conv_FreqPWM)/(2*(1/Freq_Clk))-1;
Uint16 PWM_TBPRD = 2499;        // Default for 30 kHz;

////PLL Variables
float Vab_Norm;
float Vbc_Norm;
float Vca_Norm;
ABC_DQ0_POS_F abc_dq0_pos1;
ABC_DQ0_NEG_F abc_dq0_neg1;
//SPLL_3ph_DDSRF_F spll1;
SPLL_3ph_SRF_F spll1;


////Variables for Calculating and Storing RMS and Average Values
int RMS_Cnt;
float Avg_V_DC_In;
float Avg_V_DC_In_Sum;
float Avg_I_DC_In;
float Avg_I_DC_In_Sum;
float Avg_V_Buck;
float Avg_V_Buck_Sum;
float Avg_I_Buck;
float Avg_I_Buck_Sum;
float Avg_V_Boost;
float Avg_V_Boost_Sum;
float Avg_I_Boost;
float Avg_I_Boost_Sum;
float Avg_F_Grid;
float Avg_F_Grid_Sum;
float RMS_V_AB;
float RMS_V_AB_Sum_Square;
float RMS_V_BC;
float RMS_V_BC_Sum_Square;
float RMS_V_CA;
float RMS_V_CA_Sum_Square;





//// 4-Channel Data logger
DLOG_4CH_F dlog_4chA;
float32 DBUFF_4CHA_1[DLOG_Buff];
float32 DBUFF_4CHA_2[DLOG_Buff];
float32 DBUFF_4CHA_3[DLOG_Buff];
float32 DBUFF_4CHA_4[DLOG_Buff];
float32 dval1;
float32 dval2;
float32 dval3;
float32 dval4;

float32 Reg_FP[Reg_FP_Max];     // Registers to hold floating point values for Datalogger
Uint16 DL_Index1 = 27;          // Index1 to select Data for Datalogger
Uint16 DL_Index2 = 24;          // Index2 to select Data for Datalogger
Uint16 DL_Index3 = 25;          // Index3 to select Data for Datalogger
Uint16 DL_Index4 = 26;          // Index4 to select Data for Datalogger
Uint16 DL_Prescalar = 5;        // Prescaler for Datalogger (Skip Count)
Uint16 DL_Status = 0;           // Status Bit for Datalogger
float32 DL_Trig = 2.5;          // Trigger Level for Datalogger
Uint16 DL_Start =0;             // Used to start Datalogger State Machine
Uint16 DL_State =0;             // Variable for Datalogger State Machine
Uint16 DL_Toggle =0;            // Toggled upon successful collection of data from Datalogger, used to indicate when to transfer data to registers
Uint16 DL_Data_Rdy = 0;         // Flag to denote Data is ready to transfer to Modbus Registers. Set in ISR and Cleared in main loop.

//// Define Variables and Union for data manipulation (Float to two 16-Bit Registers)
union {
       float F_Data;
       Uint16 Reg_Data[2];
}DataCon;


////Main function
void main(void)
{

    #ifdef _DEBUG
    ////debug mode
    #else
        //MemCopy(&TextLoadStart, &TextLoadEnd, &TextRunStart);
        //MemCopy(&CinitLoadStart, &CinitLoadEnd, &CinitRunStart);
        MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
        MemCopy(&DclfuncsLoadStart, &DclfuncsLoadEnd, &DclfuncsRunStart);
        MemCopy(&EconstLoadStart, &EconstLoadEnd, &EconstRunStart);
        MemCopy(&FPUmathTablesLoadStart, &FPUmathTablesLoadEnd, &FPUmathTablesRunStart);


        InitFlash();
    #endif




    //Initialize Datalogger ChA
    DLOG_4CH_F_init(&dlog_4chA);
    dlog_4chA.input_ptr1 = &dval1;
    dlog_4chA.input_ptr2 = &dval2;
    dlog_4chA.input_ptr3 = &dval3;
    dlog_4chA.input_ptr4 = &dval4;
    dlog_4chA.output_ptr1 = &DBUFF_4CHA_1[0];
    dlog_4chA.output_ptr2 = &DBUFF_4CHA_2[0];
    dlog_4chA.output_ptr3 = &DBUFF_4CHA_3[0];
    dlog_4chA.output_ptr4 = &DBUFF_4CHA_4[0];
    dlog_4chA.size = DLOG_Buff;
    dlog_4chA.pre_scalar = 5;
    dlog_4chA.trig_value = 2;
    dlog_4chA.status = 2;



    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    InitSysCtrl();

    EALLOW;
    SysCtrlRegs.HISPCP.all = ADC_MODCLK;    // HSPCLK = SYSCLKOUT/ADC_MODCLK
    EDIS;

    // Step 2. Initialize GPIO:
    // InitGpio();

    // For this example, only init the pins for the SCI-A port.
    // This function is found in the DSP2833x_Sci.c file.
    //InitSciaGpio();

    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the DSP2803x_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    //// Initialize Modbus
    mb = construct_ModbusSlave();

    for(i=0; i<sizeof(mb.holdingRegisters.Mod_Reg);i++)
    {
        mb.holdingRegisters.Mod_Reg[i]=0;
    }


    //// Set initial values
    mb.holdingRegisters.Mod_Reg[0] = 0x0000;                                //Disable PWM and Subsytems
    //Inverter Specific
    mb.holdingRegisters.Mod_Reg[8] = (_IQ5(Inv_Freq_Sw/1.0e3) & 0xFFFF);    //Set Switching Frequency (in  kHz)
    mb.holdingRegisters.Mod_Reg[9] = 0x000;                                 // Set Inverter Op-Mode [0=Passive Rectifier; 1=Islanded; 2=Grid-Forming; 3=Grid-Feeding; 4=Grid-Supporting]
    mb.holdingRegisters.Mod_Reg[10] = (_IQ5(Inv_AC_Mag) & 0xFFFF);          // Set AC Mag Reference
    mb.holdingRegisters.Mod_Reg[11] = (_IQ5(Inv_AC_Freq) & 0xFFFF);         // Set AC Fund Reference
    mb.holdingRegisters.Mod_Reg[12] = (_IQ10(Inv_AC_P) & 0xFFFF);           // Inverter Active Power Reference
    mb.holdingRegisters.Mod_Reg[13] = (_IQ10(Inv_AC_Q) & 0xFFFF);           // Inverter Reactive Power Reference
    // Buck Converter Specific
    mb.holdingRegisters.Mod_Reg[32] = (_IQ5(Buck_Freq_Sw/1.0e3) & 0xFFFF);  // Set Buck Switching Frequency (in  kHz)
    mb.holdingRegisters.Mod_Reg[33] = 0x000;                                // Set Buck Op-Mode [0=Passive; 1=Voltage-Mode; 2=Current-Mode; 3=Dual-Loop]
    mb.holdingRegisters.Mod_Reg[34] = (_IQ10(0.0) & 0xFFFF);                // Set Buck Reference Voltage (Command)
    mb.holdingRegisters.Mod_Reg[35] = (_IQ10(0.0) & 0xFFFF);                // Set Buck Reference Current (Command)
    mb.holdingRegisters.Mod_Reg[36] = (_IQ10(Buck_V_Kp) & 0xFFFF);          // Buck Voltage PI Kp Term
    mb.holdingRegisters.Mod_Reg[37] = (_IQ10(Buck_V_Ki) & 0xFFFF);          // Buck Voltage PI Ki Term
    mb.holdingRegisters.Mod_Reg[38] = (_IQ10(Buck_I_Kp) & 0xFFFF);          // Buck Current PI Kp Term
    mb.holdingRegisters.Mod_Reg[39] = (_IQ10(Buck_I_Ki) & 0xFFFF);          // Buck Current PI Ki Term
    // Boost Converter Specific
    mb.holdingRegisters.Mod_Reg[48] = (_IQ5(Boost_Freq_Sw/1.0e3) & 0xFFFF); // Set Boost Switching Frequency (in  kHz)
    mb.holdingRegisters.Mod_Reg[49] = 0x000;                                // Set Boost Op-Mode [0=Passive; 1=Voltage-Mode; 2=Current-Mode; 3=Dual-Loop]
    mb.holdingRegisters.Mod_Reg[50] = (_IQ10(0.0) & 0xFFFF);                // Set Boost Reference Voltage (Command)
    mb.holdingRegisters.Mod_Reg[51] = (_IQ10(0.0) & 0xFFFF);                // Set Boost Reference Current (Command)
    mb.holdingRegisters.Mod_Reg[52] = (_IQ10(Boost_V_Kp) & 0xFFFF);         // Boost Voltage PI Kp Term
    mb.holdingRegisters.Mod_Reg[53] = (_IQ10(Boost_V_Ki) & 0xFFFF);         // Boost Voltage PI Ki Term
    mb.holdingRegisters.Mod_Reg[54] = (_IQ10(Boost_I_Kp) & 0xFFFF);         // Boost Current PI Kp Term
    mb.holdingRegisters.Mod_Reg[55] = (_IQ10(Boost_I_Ki) & 0xFFFF);         // Boost Current PI Ki Term
    // Datalogger Specific
    mb.holdingRegisters.Mod_Reg[120] = 47;                                  // Datalogger Channel 1 Reg_FP Index
    mb.holdingRegisters.Mod_Reg[121] = 40;                                  // Datalogger Channel 2 Reg_FP Index
    mb.holdingRegisters.Mod_Reg[122] = 45;                                  // Datalogger Channel 3 Reg_FP Index
    mb.holdingRegisters.Mod_Reg[123] = 46;                                  // Datalogger Channel 4 Reg_FP Index
    DataCon.F_Data = 1.0;                                                   // Floating Point Trigger Value (Struct)
    mb.holdingRegisters.Mod_Reg[124] = DataCon.Reg_Data[1];
    mb.holdingRegisters.Mod_Reg[125] = DataCon.Reg_Data[0];
    mb.holdingRegisters.Mod_Reg[126] = 10;                                  //Datalogger Prescaler (Skip Count)


    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    EALLOW;  // This is needed to write to EALLOW protected registers
    //PieVectTable.EPWM1_INT = &epwm1_isr;
    //PieVectTable.EPWM2_INT = &epwm2_isr;
    //PieVectTable.EPWM3_INT = &epwm3_isr;
    PieVectTable.ADCINT = &adc_isr;
    PieVectTable.SPIRXINTA = &spiRxFifoIsr;
    PieVectTable.SPITXINTA = &spiTxFifoIsr;
    //PieVectTable.MRINTA= &Mcbsp_RxINTA_ISR;
    //PieVectTable.MXINTA= &Mcbsp_TxINTA_ISR;
    //PieVectTable.MRINTB= &Mcbsp_RxINTB_ISR;
    //PieVectTable.MXINTB= &Mcbsp_TxINTB_ISR;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    INV_PWM_TBPRD = 2499;
    Conv_PWM_TBPRD = 2499;

    // Initialize PI Parameters
    Buck_V_PI.Umin = 0;
    Buck_I_PI.Umin = 0;
    Boost_V_PI.Umin = 0;
    Boost_I_PI.Umin = 0;

    Gpio_Init();        //Initialize GPIO
    ADC_Init();         // Initialize ADC
    //InitAPWM();         // Initialize ECAP Modules for APWM Operation


    // Step 5. User specific code, enable interrupts:


    ////Update Values For Initialization
    INV_FreqPWM = (_IQ5toF(mb.holdingRegisters.Mod_Reg[8])) * 1e3;              // Switching Freq in Hz
    AC_Freq_Fund = (_IQ5toF(mb.holdingRegisters.Mod_Reg[11]));              // Fundamental AC Frequency
    INV_PWM_TBPRD = (1/INV_FreqPWM)/(2*(1/Freq_Clk))-1;                             // Set initial PWM Period
    Prd_Sw = INV_FreqPWM/AC_Freq_Fund;

    //Initialize PLL
    ABC_DQ0_POS_F_init(&abc_dq0_pos1);
    ABC_DQ0_NEG_F_init(&abc_dq0_neg1);
    //SPLL_3ph_DDSRF_F_init(AC_Freq_Fund,((float)(1.0/FreqPWM)), (float)(0.00933678), (float)(-0.9813264),&spll1);
    SPLL_3ph_SRF_F_init(AC_Freq_Fund,((float)(1.0/INV_FreqPWM)),&spll1);
    spll1.lpf_coeff.B0_lf = 222.58;
    spll1.lpf_coeff.B1_lf = -221.741;

    ////Initialization for SPI ADC Configurations
    InitSPI();      // Initialize SPI for (ADC0)Configuration
    //InitMcBSP();    // Initialize McBSP Ports A&B SPI for (ADC1 & ADC2)Configuration

    ////Main Loop
    for(;;)
    {


        TempCnt++;

        //// Update Command and Configuration Values from Modbus Comms

        // Check Modbus Status and perform read and write commands as needed.
        mb.loopStates(&mb);

        /// Read and Check each command value to ensure it is within the appropriate range.

        // Update Sys Enable
        Sys_En = mb.holdingRegisters.Mod_Reg[0];


        // Update Inverter Switching Frequency
        Cmd_FTemp = (_IQ5toF(mb.holdingRegisters.Mod_Reg[8])) * 1e3;                           //Switching Freq in Hz
        if(Cmd_FTemp < FreqPWM_Min)
            INV_FreqPWM = FreqPWM_Min;
        else if(Cmd_FTemp > FreqPWM_Max)
            INV_FreqPWM = FreqPWM_Max;
        else
            INV_FreqPWM = Cmd_FTemp;

        // Update Inverter Operation Mode
        Cmd_Temp = mb.holdingRegisters.Mod_Reg[9];                                          //Inverter Operation Mode
        if(Cmd_Temp < 0x005)
            Inv_OPMode = Cmd_Temp;
        else
            Inv_OPMode = 0x0000;                                                            //Passive Mode


        // Update AC Magnitude Command
        Cmd_FTemp =  (_IQ5toF(mb.holdingRegisters.Mod_Reg[10]));                             //AC Magnitude
        if(Cmd_FTemp < AC_Mag_Min)
            AC_Mag = AC_Mag_Min;
        else if(Cmd_FTemp > AC_Mag_Max)
            AC_Mag = AC_Mag_Max;
        else
            AC_Mag = Cmd_FTemp;


        // Update AC Fundamental Frequency Command
        Cmd_FTemp = (_IQ5toF(mb.holdingRegisters.Mod_Reg[11]));                    //Set AC Fundamental Freq
        if(Cmd_FTemp < AC_Freq_Fund_Min)
            AC_Freq_Fund = AC_Freq_Fund_Min;
        else if(Cmd_FTemp > AC_Freq_Fund_Max)
            AC_Freq_Fund = AC_Freq_Fund_Max;
        else
            AC_Freq_Fund = Cmd_FTemp;



        // Update Buck Converter Switching Frequency
        Cmd_FTemp = (_IQ5toF(mb.holdingRegisters.Mod_Reg[32])) * 1e3;                           //Switching Freq in Hz
        if(Cmd_FTemp < FreqPWM_Min)
            Buck_FreqPWM = FreqPWM_Min;
        else if(Cmd_FTemp > FreqPWM_Max)
            Buck_FreqPWM = FreqPWM_Max;
        else
            Buck_FreqPWM = Cmd_FTemp;

        // Update Buck Converter Operation Mode
        Cmd_Temp = mb.holdingRegisters.Mod_Reg[33];                                          //Buck Operation Mode
        if(Cmd_Temp < 0x004)
            Buck_OPMode = Cmd_Temp;
        else
            Buck_OPMode = 0x0000;                                                            //Passive Mode

        Buck_V_Cmd = _IQ10toF(mb.holdingRegisters.Mod_Reg[34]);
        Buck_I_Cmd = _IQ10toF(mb.holdingRegisters.Mod_Reg[35]);
        Buck_V_PI.Kp = _IQ10toF(mb.holdingRegisters.Mod_Reg[36]);
        Buck_V_PI.Ki = _IQ10toF(mb.holdingRegisters.Mod_Reg[37]);
        Buck_I_PI.Kp = _IQ10toF(mb.holdingRegisters.Mod_Reg[38]);
        Buck_I_PI.Ki = _IQ10toF(mb.holdingRegisters.Mod_Reg[39]);

        // Update Boost Converter Switching Frequency (Only Buck switching frequency actually used)
       Cmd_FTemp = (_IQ5toF(mb.holdingRegisters.Mod_Reg[48])) * 1e3;                           //Switching Freq in Hz
       if(Cmd_FTemp < FreqPWM_Min)
           Boost_FreqPWM = FreqPWM_Min;
       else if(Cmd_FTemp > FreqPWM_Max)
           Boost_FreqPWM = FreqPWM_Max;
       else
           Boost_FreqPWM = Cmd_FTemp;

       // Update Buck Converter Operation Mode
       Cmd_Temp = mb.holdingRegisters.Mod_Reg[49];                                          //Boost Operation Mode
       if(Cmd_Temp < 0x004)
           Boost_OPMode = Cmd_Temp;
       else
           Boost_OPMode = 0x0000;                                                            //Passive Mode

       Boost_V_Cmd = _IQ10toF(mb.holdingRegisters.Mod_Reg[50]);
       Boost_I_Cmd = _IQ10toF(mb.holdingRegisters.Mod_Reg[51]);
       Boost_V_PI.Kp = _IQ10toF(mb.holdingRegisters.Mod_Reg[52]);
       Boost_V_PI.Ki = _IQ10toF(mb.holdingRegisters.Mod_Reg[53]);
       Boost_I_PI.Kp = _IQ10toF(mb.holdingRegisters.Mod_Reg[54]);
       Boost_I_PI.Ki = _IQ10toF(mb.holdingRegisters.Mod_Reg[55]);



                // Calculate based on received values
       INV_PWM_TBPRD = (1/INV_FreqPWM)/(2*(1/Freq_Clk))-1;                        //Set PWM Period
       Prd_Sw = INV_FreqPWM/AC_Freq_Fund;                                         //Switching cycles per fundamental period Ex:(30 kHz / 60Hz = 500)
       Conv_PWM_TBPRD= (1/Buck_FreqPWM)/(2*(1/Freq_Clk))-1;                       //Set PWM Period for both Buck and Boost Converters

       //Update PLL for Switching and Fundamental
       //SPLL_3ph_DDSRF_F_init(AC_Freq_Fund,((float)(1.0/FreqPWM)), (float)(0.00933678), (float)(-0.9813264),&spll1);
       SPLL_3ph_SRF_F_init(AC_Freq_Fund,((float)(1.0/INV_FreqPWM)),&spll1);

       // Set Max Duty Cycles
       Buck_V_PI.Umax  = (Uint16) Conv_PWM_TBPRD;
       Buck_I_PI.Umax  = (Uint16) Conv_PWM_TBPRD;
       Boost_V_PI.Umax = (Uint16) (Conv_PWM_TBPRD * 0.9);
       Boost_I_PI.Umax = (Uint16) (Conv_PWM_TBPRD * 0.9);


        //Update PWM Freq
        EPwm1Regs.TBPRD = (Uint16) INV_PWM_TBPRD;
        EPwm2Regs.TBPRD = (Uint16) INV_PWM_TBPRD;
        EPwm3Regs.TBPRD = (Uint16) INV_PWM_TBPRD;
        EPwm4Regs.TBPRD = (Uint16) Conv_PWM_TBPRD;

        //// Update Modbus Registers with "FP_Reg" Array
       for(i=0; i< Reg_FP_Max; i++)
       {
           DataCon.F_Data = Reg_FP[i];
           mb.holdingRegisters.Mod_Reg[(i*2)+256] = DataCon.Reg_Data[1];
           mb.holdingRegisters.Mod_Reg[(i*2)+257] = DataCon.Reg_Data[0];
       }


        //// Update Datalogger Values/Parameters
        // Update Channel 1 Signal from Reg_FP Floating Point Registers
        Cmd_Temp = mb.holdingRegisters.Mod_Reg[120];
        if(Cmd_Temp < Reg_FP_Max)
            DL_Index1 = Cmd_Temp;

        // Update Channel 2 Signal from Reg_FP Floating Point Registers
        Cmd_Temp = mb.holdingRegisters.Mod_Reg[121];
        if(Cmd_Temp < Reg_FP_Max)
            DL_Index2 = Cmd_Temp;

        // Update Channel 3 Signal from Reg_FP Floating Point Registers
        Cmd_Temp = mb.holdingRegisters.Mod_Reg[122];
        if(Cmd_Temp < Reg_FP_Max)
            DL_Index3 = Cmd_Temp;

        // Update Channel 4 Signal from Reg_FP Floating Point Registers
        Cmd_Temp = mb.holdingRegisters.Mod_Reg[123];
        if(Cmd_Temp < Reg_FP_Max)
            DL_Index4 = Cmd_Temp;

        // Update Trigger Value (Floating Point)
        DataCon.Reg_Data[1] = mb.holdingRegisters.Mod_Reg[124];
        DataCon.Reg_Data[0] = mb.holdingRegisters.Mod_Reg[125];
        DL_Trig = DataCon.F_Data;

        // Update Prescaler Value (Skip Count)
        DL_Prescalar = mb.holdingRegisters.Mod_Reg[126];

        // Update Start Datalogger Command
        DL_Start = mb.holdingRegisters.Mod_Reg[127];


        // Check if Datalogger is ready; If yes transfer data to Modbus Registers
        if(DL_Data_Rdy == 1)
        {
            for(i=0; i<DLOG_Buff;i++)
            {
                //Channel 1 XFER
                DataCon.F_Data = DBUFF_4CHA_1[i];
                mb.holdingRegisters.Mod_Reg[512 + 2*i]     = DataCon.Reg_Data[1];
                mb.holdingRegisters.Mod_Reg[512 + 2*i + 1] = DataCon.Reg_Data[0];

                //Channel 2 XFER
                DataCon.F_Data = DBUFF_4CHA_2[i];
                mb.holdingRegisters.Mod_Reg[896 + 2*i]     = DataCon.Reg_Data[1];
                mb.holdingRegisters.Mod_Reg[896 + 2*i + 1] = DataCon.Reg_Data[0];

                //Channel 3 XFER
                DataCon.F_Data = DBUFF_4CHA_3[i];
                mb.holdingRegisters.Mod_Reg[1280 + 2*i]     = DataCon.Reg_Data[1];
                mb.holdingRegisters.Mod_Reg[1280 + 2*i + 1] = DataCon.Reg_Data[0];

                //Channel 4 XFER
                DataCon.F_Data = DBUFF_4CHA_4[i];
                mb.holdingRegisters.Mod_Reg[1664 + 2*i]     = DataCon.Reg_Data[1];
                mb.holdingRegisters.Mod_Reg[1664 + 2*i + 1] = DataCon.Reg_Data[0];
            }

            DL_Data_Rdy = 0;    // Reset Flag to 0 for next operation
        }
        //// End Datalogger XFER
    }
}

void ADC_Init(void)
{
    // Configure ADC
    EALLOW;
    PieVectTable.ADCINT = &adc_isr;
    EDIS;

    InitAdc();

    IER |= M_INT1;                      // Enable CPU Interrupt 1
    //PieCtrlRegs.PIEIER1.bit.INTx1 = 1;    // Enable INT 1.1 in the PIE
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
    EINT;                               // Enable Global interrupt INTM
    ERTM;                               // Enable Global realtime interrupt DBGM

    EALLOW;
    // Specific ADC setup for this example:
        //Sample and Hold window
        //AdcRegs.ADCTRL1.bit.ACQ_PS = 0x1;       //Window length(1+#of Cycles)
        AdcRegs.ADCTRL1.bit.ACQ_PS = 0x0F;       //Increased Window Length for more reliable sampling
        //ADC Clock Rate
        AdcRegs.ADCTRL3.bit.ADCCLKPS = ADC_CKPS; //0 works faster but may result in errors  // 12.5 MHz 80ns
        AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;        // 1  Cascaded mode
        AdcRegs.ADCTRL1.bit.SEQ_OVRD = 0;
        //Specify # of Conversions
        //AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 0x07; // convert only used channels (16)

        //AdcRegs.ADCTRL3.bit.SMODE_SEL = 0; //setup ADC for sequential sampling

        // Simultaneous Sampling Cascaded Sequencer Mode Example From Tech Manual
        AdcRegs.ADCTRL3.bit.SMODE_SEL = 0x1;    // Setup simultaneous sampling mode
        AdcRegs.ADCTRL1.bit.SEQ_CASC = 0x1;     // Setup cascaded sequencer mode
        AdcRegs.ADCMAXCONV.all = 0x0007;        // 8 double conv's (16 total)
        AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;  // Setup conv from channels ADCINA0 and ADCINB0
        AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1;  // Setup conv from channels ADCINA1 and ADCINB1
        AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2;  // Setup conv from channels ADCINA2 and ADCINB2
        AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3;  // Setup conv from channels ADCINA3 and ADCINB3
        AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x4;  // Setup conv from channels ADCINA4 and ADCINB4
        AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0x5;  // Setup conv from channels ADCINA5 and ADCINB5
        AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x6;  // Setup conv from channels ADCINA6 and ADCINB6
        AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 0x7;  // Setup conv from channels ADCINA7 and ADCINB7

        /*
        //Convert used channels
        AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;    // Setup ADCINA0 as 1st SEQ1 conv.
        AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1;    // Setup ADCINA1 as 2nd SEQ1 conv.
        AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2;    // Setup ADCINA2 as 3rd SEQ1 conv.
        AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3;    // Setup ADCINA3 as 4th SEQ1 conv.

        AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x4;    // Setup ADCINA4 as 5th SEQ2 conv.
        AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0x5;    // Setup ADCINA5 as 6th SEQ2 conv.
        AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x6;    // Setup ADCINA6 as 7th SEQ2 conv.
        AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 0x7;    // Setup ADCINA7 as 8th SEQ2 conv.



        AdcRegs.ADCCHSELSEQ3.bit.CONV08 = 0x8;    // Setup ADCINB0 as 9th SEQ3 conv.
        AdcRegs.ADCCHSELSEQ3.bit.CONV09 = 0x9;    // Setup ADCINB1 as 10th SEQ3 conv
        AdcRegs.ADCCHSELSEQ3.bit.CONV10 = 0xA;    // Setup ADCINB2 as 11th SEQ3 conv
        AdcRegs.ADCCHSELSEQ3.bit.CONV11 = 0xB;    // Setup ADCINB3 as 12th SEQ3 conv

        AdcRegs.ADCCHSELSEQ4.bit.CONV12 = 0xC;    // Setup ADCINB4 as 13th SEQ4 conv
        AdcRegs.ADCCHSELSEQ4.bit.CONV13 = 0xD;    // Setup ADCINB5 as 14th SEQ4 conv
        AdcRegs.ADCCHSELSEQ4.bit.CONV14 = 0xE;    // Setup ADCINB6 as 15th SEQ4 conv
        AdcRegs.ADCCHSELSEQ4.bit.CONV15 = 0xF;    // Setup ADCINB7 as 16th SEQ4 conv
        */



        //Setup ADC Triggering
        AdcRegs.ADCTRL1.bit.CONT_RUN = 0;       // Setup start/stop run
        AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;// Enable SOCA from ePWM to start SEQ1
        AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;  // Enable SEQ1 interrupt (every End Of Sequence)
        AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0;  //
    EDIS;

    //ADC Triggering Setup
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;
    EPwm1Regs.ETSEL.bit.SOCASEL = 0x2;//4; // 2=Select SOC from CPMA on Period; 4=Select SOC from from CPMA on upcount
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;        // Generate pulse every event
}

void Gpio_Init(void)
{
    // Step 2. Initalize GPIO:
    // Initialize GPIO
    EALLOW;

    GpioCtrlRegs.GPAMUX1.all = 0x0000;     // GPIO functionality GPIO0-GPIO15
    GpioCtrlRegs.GPAMUX2.all = 0x0000;     // GPIO functionality GPIO16-GPIO31
    GpioCtrlRegs.GPBMUX1.all = 0x0000;     // GPIO functionality GPIO32-GPIO47

    GpioCtrlRegs.GPADIR.all = 0xFFFFFFFF;  // GPIO0-GPIO31 are outputs
    GpioCtrlRegs.GPBDIR.all = 0xFFFFFFFF;  // GPIO32-GPIO63 are outputs

    GpioCtrlRegs.GPAQSEL1.all = 0x0000;    // GPIO0-GPIO15 Synch to SYSCLKOUT
    GpioCtrlRegs.GPAQSEL2.all = 0x0000;    // GPIO16-GPIO31 Synch to SYSCLKOUT
    GpioCtrlRegs.GPBQSEL1.all = 0x0000;    // GPIO32-GPIO47 Synch to SYSCLKOUT

    GpioCtrlRegs.GPAPUD.all = 0xFFFFFFFF;  // Pullup's disabled GPIO0-GPIO31
    GpioCtrlRegs.GPBPUD.all = 0xFFFFFFFF;  // Pullup's disabled GPIO32-GPIO63

    ////Serial RX/TX Config
    GpioCtrlRegs.GPADIR.bit.GPIO28 = 0; // Configure as input
    GpioCtrlRegs.GPADIR.bit.GPIO29 = 1; // Configure as output
    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;  // Enable pull-up for GPIO28 (SCIRXDA)
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;  // Enable pull-up for GPIO29 (SCITXDA)
    // Set qualification for selected pins to asynch only
    // Inputs are synchronized to SYSCLKOUT by default.
    // This will select asynch (no qualification) for the selected pins.
    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure GPIO28 to SCIRXDA
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO29 to SCITXDA
    // End Serial Config


    //Switch Setup
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 0; // Configure as input (SW1)
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 0; // Configure as input (SW2)
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; // Disable internal pull-up (SW1)
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1; // Disable internal pull-up (SW2)

    ////Configure PWM/LED/Relay CPLD Enable Signals (For Boot-Up and Fault Mitigation)
    // Signals passed by CPLD when GPIO_60 = '1' and GPIO_61 = '0'
    GpioCtrlRegs.GPBDIR.bit.GPIO60 = 1; // Configure as output
    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 1; // disable internal pull-up
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;   //initialize to zero
    GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1; // Configure as output
    GpioCtrlRegs.GPBPUD.bit.GPIO61 = 0; // Enable internal pull-up
    GpioDataRegs.GPBSET.bit.GPIO61 = 1;   //initialize to one
    // End Enable CPLD Signals


    ////Configure LEDs
    //Control Card LEDs
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1; // Configure as output (Red_Led1 On Board)
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 1; // Disable internal pull-up
    GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;   //initialize to zero
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1; // Configure as output (Red_Led2 On Board)
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 1; // Disable internal pull-up
    GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;   //initialize to zero

    ////Demo Board LEDs
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;    // Configure as output (LED9 On Demo Board)
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;    // Disable internal pull-up
    GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;  //initialize to zero
    GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;    // Configure as output (LED10 On Demo Board)
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;    // Disable internal pull-up
    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;  //initialize to zero
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1; // Configure as output (LED1 On Demo Board)
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1; // Disable internal pull-up
    GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;   //initialize to zero
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1; // Configure as output (LED2 On Demo Board)
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1; // Disable internal pull-up
    GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;   //initialize to zero
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1; // Configure as output (LED3 On Demo Board)
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1; // Disable internal pull-up
    GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;   //initialize to zero
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 1; // Configure as output (LED4 On Demo Board)
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 1; // Disable internal pull-up
    GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;   //initialize to zero
    GpioCtrlRegs.GPADIR.bit.GPIO24 = 1; // Configure as output (LED5 On Demo Board)
    GpioCtrlRegs.GPAPUD.bit.GPIO24 = 1; // Disable internal pull-up
    GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;   //initialize to zero
    GpioCtrlRegs.GPADIR.bit.GPIO25 = 1; // Configure as output (LED6 On Demo Board)
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 1; // Disable internal pull-up
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;   //initialize to zero
    GpioCtrlRegs.GPADIR.bit.GPIO26 = 1; // Configure as output (LED7 On Demo Board)
    GpioCtrlRegs.GPAPUD.bit.GPIO26 = 1; // Disable internal pull-up
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;   //initialize to zero
    GpioCtrlRegs.GPADIR.bit.GPIO27 = 1; // Configure as output (LED8 On Demo Board)
    GpioCtrlRegs.GPAPUD.bit.GPIO27 = 1; // Disable internal pull-up
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;   //initialize to zero


    ////Configure DSP-SPI (ADC0) [Inverter]
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;   // Enable pull-up on GPIO16 (SPISIMOA)
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;   // Enable pull-up on GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;   // Enable pull-up on GPIO18 (SPICLKA)
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;   // Enable pull-up on GPIO19 (SPISTEA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // Asynch input GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; // Configure GPIO16 as SPISIMOA
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // Configure GPIO17 as SPISOMIA
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; // Configure GPIO18 as SPICLKA
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1; // Configure GPIO19 as SPISTEA
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1; // Configure as output
    GpioCtrlRegs.GPADIR.bit.GPIO16 = 1; // Configure as output
    GpioCtrlRegs.GPADIR.bit.GPIO18 = 1; // Configure as output
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 0; // Configure as input
    // End DSP-SPI (ADC0) Config

    /*
    //// Configure DSP McBSP PortA for SPI-ADC (ADC-1) [On-Board CPLD]
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;   // Enable pull-up on GPIO20 (SPISIMOA)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;   // Enable pull-up on GPIO21 (SPISOMIA)
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;   // Enable pull-up on GPIO22 (SPICLKA)
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;   // Enable pull-up on GPIO23 (SPISTEA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 3; // Asynch input GPIO21 (SPISOMIA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 2; // Configure GPIO20 as SPISIMOA
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 2; // Configure GPIO21 as SPISOMIA
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 2; // Configure GPIO22 as SPICLKA
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 2; // Configure GPIO23 as SPISTEA
    GpioCtrlRegs.GPADIR.bit.GPIO23 = 1; // Configure as output
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 1; // Configure as output
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 1; // Configure as output
    GpioCtrlRegs.GPADIR.bit.GPIO21 = 0; // Configure as input
    // End DSP-SPI (ADC1) Config [McBSP]
     */

    /*
    //// Configure DSP McBSP PortB for SPI-ADC (ADC-2) [On-Board CPLD]
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;   // Enable pull-up on GPIO12 (SPISIMOA)
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;   // Enable pull-up on GPIO13 (SPISOMIA)
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;   // Enable pull-up on GPIO14 (SPICLKA)
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;   // Enable pull-up on GPIO15 (SPISTEA)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 3; // Asynch input GPIO13 (SPISOMIA)
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 3; // Configure GPIO12 as SPISIMOA
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 3; // Configure GPIO13 as SPISOMIA
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 3; // Configure GPIO14 as SPICLKA
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 3; // Configure GPIO15 as SPISTEA
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1; // Configure as output
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1; // Configure as output
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 1; // Configure as output
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 0; // Configure as input
    // End DSP-SPI (ADC2) Config [McBSP]
    */


    //// Configure EPWMs
    //EPWM1 Configure
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
    //EPWM2 Configure
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B
    //EPWM3 Configure
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B
    //EPWM4 Configure
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // Disable pull-up
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;    // Disable pull-up
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO7 as EPWM4B

    ////EPWM5 Configure
    //GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;    // Disable pull-up
    //GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;    // Disable pull-up
    //GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure GPIO8 as EPWM5A
    //GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // Configure GPIO9 as EPWM5B
    ////EPWM6 Configure
    //GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;    // Disable pull-up
    //GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;    // Disable pull-up
    //GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO10 as EPWM6A
    //GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // Configure GPIO11 as EPWM6B
    // End EPWM Config


    /*
    //// Configure ECAP PWMs (APWMs)
    GpioCtrlRegs.GPAPUD.bit.GPIO24 = 1;    // Disable pull-up
    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 1;   // Configure GPIO24 as ECAP1
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 1;    // Disable pull-up
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 1;   // Configure GPIO25 as ECAP2
    GpioCtrlRegs.GPAPUD.bit.GPIO27 = 1;    // Disable pull-up
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 1;   // Configure GPIO27 as ECAP4
    GpioCtrlRegs.GPBPUD.bit.GPIO48 = 1;    // Disable pull-up
    GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 1;   // Configure GPIO48 as ECAP5
    // End Configure ECAP PWMs
     */



    EDIS;


    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    InitEPWM1();
    InitEPWM2();
    InitEPWM3();
    InitEPWM4();

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}

interrupt void  adc_isr(void)
{
    int i;

    /*
    // Sample ADCs and converter variables to real world values
    V_ADC[0] = AdcRegs.ADCRESULT0 >>4;
    V_ADC[1] = AdcRegs.ADCRESULT1 >>4;
    V_ADC[2] = AdcRegs.ADCRESULT2 >>4;
    V_ADC[3] = AdcRegs.ADCRESULT3 >>4;
    V_ADC[4] = AdcRegs.ADCRESULT4 >>4;
    V_ADC[5] = AdcRegs.ADCRESULT5 >>4;
    V_ADC[6] = AdcRegs.ADCRESULT6 >>4;
    V_ADC[7] = AdcRegs.ADCRESULT7 >>4;
    V_ADC[8] = AdcRegs.ADCRESULT8 >>4;
    V_ADC[9] = AdcRegs.ADCRESULT9 >>4;
    V_ADC[10] = AdcRegs.ADCRESULT10 >>4;
    V_ADC[11] = AdcRegs.ADCRESULT11 >>4;
    V_ADC[12] = AdcRegs.ADCRESULT12 >>4;
    V_ADC[13] = AdcRegs.ADCRESULT13 >>4;
    V_ADC[14] = AdcRegs.ADCRESULT14 >>4;
    V_ADC[15] = AdcRegs.ADCRESULT15 >>4;
    */

    // Sample ADCs and converter variables to real world values (For Simultaneous Sampling Cascaded Sequencer Mode)
    ADC_DSP[0] = AdcRegs.ADCRESULT0 >>4;
    ADC_DSP[8] = AdcRegs.ADCRESULT1 >>4;
    ADC_DSP[1] = AdcRegs.ADCRESULT2 >>4;
    ADC_DSP[9] = AdcRegs.ADCRESULT3 >>4;
    ADC_DSP[2] = AdcRegs.ADCRESULT4 >>4;
    ADC_DSP[10] = AdcRegs.ADCRESULT5 >>4;
    ADC_DSP[3] = AdcRegs.ADCRESULT6 >>4;
    ADC_DSP[11] = AdcRegs.ADCRESULT7 >>4;
    ADC_DSP[4] = AdcRegs.ADCRESULT8 >>4;
    ADC_DSP[12] = AdcRegs.ADCRESULT9 >>4;
    ADC_DSP[5] = AdcRegs.ADCRESULT10 >>4;
    ADC_DSP[13] = AdcRegs.ADCRESULT11 >>4;
    ADC_DSP[6] = AdcRegs.ADCRESULT12 >>4;
    ADC_DSP[14] = AdcRegs.ADCRESULT13 >>4;
    ADC_DSP[7] = AdcRegs.ADCRESULT14 >>4;
    ADC_DSP[15] = AdcRegs.ADCRESULT15 >>4;

    //Store ADC Values in Register Bank and Modbus Holding Registers

    for(i=0;i<16;i++)
    {
        mb.holdingRegisters.Mod_Reg[i+128]=ADC_DSP[i];
    }



    // Reconstruct ADC values to real world values
    V_DCin = mb.holdingRegisters.Mod_Reg[128] * V_Scale1;                                // ADC_DSP*(3/4095) * (31.6e3/(1e6+31.6e3))^-1
    V_DCin = 24.0;
    V_PhA = mb.holdingRegisters.Mod_Reg[129] * V_Scale2 - V_Offset_ABC;                  // ADC_DSP*(3/4095) * (13.7e3/(1e6+13.7e3))^-1
    V_PhB = mb.holdingRegisters.Mod_Reg[130] * V_Scale2 - V_Offset_ABC;                  // ADC_DSP*(3/4095) * (13.7e3/(1e6+13.7e3))^-1
    V_PhC = mb.holdingRegisters.Mod_Reg[131] * V_Scale2 - V_Offset_ABC;                  // ADC_DSP*(3/4095) * (13.7e3/(1e6+13.7e3))^-1
    V_Buck = mb.holdingRegisters.Mod_Reg[132] * V_Scale1;                                // ADC_DSP*(3/4095) * (31.6e3/(1e6+31.6e3))^-1
    V_Boost = mb.holdingRegisters.Mod_Reg[133] * V_Scale1;                               // ADC_DSP*(3/4095) * (31.6e3/(1e6+31.6e3))^-1
    I_DCin = (mb.holdingRegisters.Mod_Reg[134] * I_Scale2)-I_Offset_ACS714_05B;          // [ADC_DSP*(5/4095) * (185mV/A)^-1] -13.5
    V_Pot = mb.holdingRegisters.Mod_Reg[135] * ADC_Scale_DSP * R_Div_Scale3;             // ADC_DSP*(3/4095)
    V_AB = V_PhA-V_PhB;
    V_BC = V_PhB-V_PhC;
    V_CA = V_PhC-V_PhA;


    //// Update PLL


    // Normalize Voltages
    Vab_Norm = (float) (V_AB/Vac_Peak_d);
    Vbc_Norm = (float) (V_BC/Vac_Peak_d);
    Vca_Norm = (float) (V_CA/Vac_Peak_d);

    /*
    // The Section below is the process for the DDSRF which is more accurate for unbalanced 3-phase Loads.
    // It is not used here due to the large number of clock cycles needed to compute.

    //ABC-to-DQ Positive
    abc_dq0_pos1.a = Vab_Norm;
    abc_dq0_pos1.b = Vbc_Norm;
    abc_dq0_pos1.c = Vca_Norm;
    abc_dq0_pos1.sin = (float)sin((spll1.theta[1]));
    abc_dq0_pos1.cos = (float)cos((spll1.theta[1]));
    ABC_DQ0_POS_F_FUNC(&abc_dq0_pos1);

    //ABC-to-DQ Negative
    abc_dq0_neg1.a = Vab_Norm;
    abc_dq0_neg1.b = Vbc_Norm;
    abc_dq0_neg1.c = Vca_Norm;
    abc_dq0_neg1.sin = (float)sin((spll1.theta[1]));
    abc_dq0_neg1.cos = (float)cos((spll1.theta[1]));
    ABC_DQ0_NEG_F_FUNC(&abc_dq0_neg1);

    spll1.d_p = abc_dq0_pos1.d;
    spll1.q_p = abc_dq0_pos1.q;
    spll1.d_n = abc_dq0_neg1.d;
    spll1.q_n = abc_dq0_neg1.q;
    spll1.cos_2theta=(float)cos((2)*spll1.theta[0]);
    spll1.sin_2theta=(float)sin((2)*spll1.theta[0]);
    // SPLL call
    SPLL_3ph_DDSRF_F_FUNC(&spll1);
    */


    // This routine assumes a balanced 3-phase system. It is not as resource intensive as the DDSRF routine above.
    abc_dq0_pos1.a = (Vab_Norm);
    abc_dq0_pos1.b = (Vbc_Norm);
    abc_dq0_pos1.c = (Vca_Norm);
    abc_dq0_pos1.sin = (float)sin((spll1.theta[1]));
    abc_dq0_pos1.cos = (float)cos((spll1.theta[1]));
    //ABC_DQ0_POS_F_MACRO(abc_dq0_pos1);
    ABC_DQ0_POS_F_FUNC(&abc_dq0_pos1);
    spll1.v_q[0] = (abc_dq0_pos1.q);
    // SPLL call
    SPLL_3ph_SRF_F_FUNC(&spll1);

    //// End PLL


    // Update Floating Point Registers for Datalogger use.
    // DSP-ADC_A Values Scaled and Converted to Floating Point Representation (Inverter)
    Reg_FP[0] = V_DCin;
    Reg_FP[1] = V_PhA;
    Reg_FP[2] = V_PhB;
    Reg_FP[3] = V_PhC;
    Reg_FP[4] = V_Buck;
    Reg_FP[5] = V_Boost;
    Reg_FP[6] = I_DCin;
    Reg_FP[7]=  V_Pot;
    // DSP-ADC_B Values Scaled and Converted to Floating Point Representation (Future Use)
    Reg_FP[8]=  0;                  //Future Use (ADCB0)
    Reg_FP[9]=  0;                  //Future Use (ADCB1)
    Reg_FP[10]=  0;                 //Future Use (ADCB2)
    Reg_FP[11]=  0;                 //Future Use (ADCB3)
    Reg_FP[12]=  0;                 //Future Use (ADCB4)
    Reg_FP[13]=  0;                 //Future Use (ADCB5)
    Reg_FP[14]=  0;                 //Future Use (ADCB6)
    Reg_FP[15]=  0;                 //Future Use (ADCB7)
    // SPI-ADC_0 Values Scaled and Converted to Floating Point Representation (PE-Eval PCB)
    Reg_FP[16]= I_DCin2;            //Updated in SPI Interrupt
    Reg_FP[17]= I_L1;               //Updated in SPI Interrupt
    Reg_FP[18]= I_L2;               //Updated in SPI Interrupt
    Reg_FP[19]= I_L3;               //Updated in SPI Interrupt
    Reg_FP[20]=I_Buck;              //Updated in SPI Interrupt
    Reg_FP[21]=I_Boost;             //Updated in SPI Interrupt
    Reg_FP[22]=Ia;                  //Updated in SPI Interrupt
    Reg_FP[23]=I_L5;                //Updated in SPI Interrupt
    // SPI-ADC_1 Values Scaled and Converted to Floating Point Representation (Future Use)
    Reg_FP[24]= 0;                  //Future Use (SPI-ADC_1-V0)
    Reg_FP[25]= 0;                  //Future Use (SPI-ADC_1-V1)
    Reg_FP[26]= 0;                  //Future Use (SPI-ADC_1-V2)
    Reg_FP[27]= 0;                  //Future Use (SPI-ADC_1-V3)
    Reg_FP[28]= 0;                  //Future Use (SPI-ADC_1-V4)
    Reg_FP[29]= 0;                  //Future Use (SPI-ADC_1-V5)
    Reg_FP[30]= 0;                  //Future Use (SPI-ADC_1-V6)
    Reg_FP[31]= 0;                  //Future Use (SPI-ADC_1-V7)
    // SPI-ADC_2 Values Scaled and Converted to Floating Point Representation (Future Use)
    Reg_FP[32]= 0;                  //Future Use (SPI-ADC_2-V0)
    Reg_FP[33]= 0;                  //Future Use (SPI-ADC_2-V1)
    Reg_FP[34]= 0;                  //Future Use (SPI-ADC_2-V2)
    Reg_FP[35]= 0;                  //Future Use (SPI-ADC_2-V3)
    Reg_FP[36]= 0;                  //Future Use (SPI-ADC_2-V4)
    Reg_FP[37]= 0;                  //Future Use (SPI-ADC_2-V5)
    Reg_FP[38]= 0;                  //Future Use (SPI-ADC_2-V6)
    Reg_FP[39]= 0;                  //Future Use (SPI-ADC_2-V7)
    // Calculated Values
    Reg_FP[40] = V_AB;
    Reg_FP[41] = V_BC;
    Reg_FP[42] = V_CA;
    Reg_FP[43] = Sine1;
    Reg_FP[44] = Sine2;
    Reg_FP[45] = Sine3;
    Reg_FP[46] = Step_sin;
    Reg_FP[47] = spll1.theta[0];




    ////For RMS and Average Calculations
    if(RMS_Cnt < Prd_Sw*3)  // Average over 3 fundamental cycles
    {
        Avg_V_DC_In_Sum = V_DCin + Avg_V_DC_In_Sum;
        Avg_I_DC_In_Sum = I_DCin + Avg_I_DC_In_Sum;
        Avg_V_Buck_Sum = V_Buck + Avg_V_Buck_Sum;
        Avg_I_Buck_Sum = I_Buck + Avg_I_Buck_Sum;
        Avg_V_Boost_Sum = V_Boost + Avg_V_Boost_Sum;
        Avg_I_Boost_Sum = I_Boost + Avg_I_Boost_Sum;
        Avg_F_Grid_Sum = spll1.fo + Avg_F_Grid_Sum;
        RMS_V_AB_Sum_Square = (V_AB*V_AB)+RMS_V_AB_Sum_Square;
        RMS_V_BC_Sum_Square = (V_BC*V_BC)+RMS_V_BC_Sum_Square;
        RMS_V_CA_Sum_Square = (V_CA*V_CA)+RMS_V_CA_Sum_Square;
        RMS_Cnt++;
    }
    else
    {
        Avg_V_DC_In = (Avg_V_DC_In_Sum/RMS_Cnt);
        Avg_V_DC_In_Sum = 0;
        Avg_I_DC_In = (Avg_I_DC_In_Sum/RMS_Cnt);
        Avg_I_DC_In_Sum = 0;
        Avg_V_Buck = (Avg_V_Buck_Sum/RMS_Cnt);
        Avg_V_Buck_Sum = 0;
        Avg_I_Buck = (Avg_I_Buck_Sum/RMS_Cnt);
        Avg_I_Buck_Sum = 0;
        Avg_V_Boost = (Avg_V_Boost_Sum/RMS_Cnt);
        Avg_V_Boost_Sum = 0;
        Avg_I_Boost = (Avg_I_Boost_Sum/RMS_Cnt);
        Avg_I_Boost_Sum = 0;
        Avg_F_Grid = (Avg_F_Grid_Sum/RMS_Cnt);
        Avg_F_Grid_Sum = 0;
        RMS_V_AB = sqrt(RMS_V_AB_Sum_Square/RMS_Cnt);
        RMS_V_AB_Sum_Square = 0;
        RMS_V_BC = sqrt(RMS_V_BC_Sum_Square/RMS_Cnt);
        RMS_V_BC_Sum_Square = 0;
        RMS_V_CA = sqrt(RMS_V_CA_Sum_Square/RMS_Cnt);
        RMS_V_CA_Sum_Square = 0;

        RMS_Cnt =0;


        // Update Registers with measured/calculated values
        Reg_FP[48] = Avg_V_DC_In;
        Reg_FP[49] = Avg_I_DC_In;
        Reg_FP[50] = Avg_V_Buck;
        Reg_FP[51] = Avg_I_Buck;
        Reg_FP[52] = Avg_V_Boost;
        Reg_FP[53] = Avg_I_Boost;
        Reg_FP[54] = Avg_F_Grid;
        Reg_FP[55] = RMS_V_AB;
        Reg_FP[56] = RMS_V_BC;
        Reg_FP[57] = RMS_V_CA;
    }
    //// End RMS and Average Calculations



    ////Datalogger State Machine
    switch(DL_State)
    {

        case 0:                                             // Datalogger Initial Ready State
            if(DL_Start == 1)                               // If DL_Start then go to state 1.
                DL_State = 1;
            else
                DL_State = 0;
            break;

        case 1:                                             // Datalogger Waiting for Trigger
            mb.holdingRegisters.Mod_Reg[127] = 0;            // Clear DL_Start Modbus Register
            DL_Start = 0;                                   // Clear Start Flag
            dlog_4chA.pre_scalar = DL_Prescalar;            // Set Prescaler (How many clock cycles to skip)
            dlog_4chA.trig_value = DL_Trig;                 // Set Trigger (Floating Point Value)
            dval1= (Reg_FP[DL_Index1]);                     // Set Sampled Value for Channel 1
            dval2= (Reg_FP[DL_Index2]);                     // Set Sampled Value for Channel 2
            dval3= (Reg_FP[DL_Index3]);                     // Set Sampled Value for Channel 3
            dval4= (Reg_FP[DL_Index4]);                     // Set Sampled Value for Channel 4
            DLOG_4CH_F_FUNC(&dlog_4chA);
            DL_Status = dlog_4chA.status;

            // Check DL_Status Flag ('1' means "waiting for trigger" and '2' denotes "logging")
            if(DL_Status == 1)                              // If DL_Status = '1' (waiting) then stay in state 1
                DL_State = 1;
            else                                            // If DL_Status = '2' (logging) then go to state 2
                DL_State = 2;
            break;


        case 2:                                             // Datalogger Logging Data
            DL_Start = 0;                                   // Clear Start Flag
            dlog_4chA.pre_scalar = DL_Prescalar;            // Set Prescaler (How many clock cycles to skip)
            dlog_4chA.trig_value = DL_Trig;                 // Set Trigger (Floating Point Value)
            dval1= (Reg_FP[DL_Index1]);                     // Set Sampled Value for Channel 1
            dval2= (Reg_FP[DL_Index2]);                     // Set Sampled Value for Channel 2
            dval3= (Reg_FP[DL_Index3]);                     // Set Sampled Value for Channel 3
            dval4= (Reg_FP[DL_Index4]);                     // Set Sampled Value for Channel 4
            DLOG_4CH_F_FUNC(&dlog_4chA);
            DL_Status = dlog_4chA.status;

            // Check DL_Status Flag ('1' means "waiting for trigger" and '2' denotes "logging")
            if(DL_Status == 2)                              // If DL_Status = '2' (logging) then stay in state 2 until complete
                DL_State = 2;
            else                                            // If DL_Status = '1' (waiting) then return to state 0 [Capture Complete]
            {
                DL_State = 0;                               // Return to State 0
                DL_Toggle = ~DL_Toggle;                     // Toggle DL_Toggle, used to show data has changed and should be uploaded
                DL_Data_Rdy = 1;                            // Flag so data can be transferred in Main Loop. Cleared in Main Loop after Transfer.
            }
            break;



        default:
            DL_State = 0;
            DL_Start = 0;                                   // Clear Start Flag
            break;
    }


    mb.holdingRegisters.Mod_Reg[196] = DL_State;            //Update DL_State for GUI Indication

    ////End Datalogger State Machine


    // Create an AC reference
    if(Sin_Counter >= Prd_Sw)   //here we count to a number based on how many switching cycles occur for every fundamental cycle
    {
        Sin_Counter = 0;
    }
    else
    {
        Sin_Counter++;
    }

    //Feedforward Command to maintain magnitude of Inverter output as DC Input is varied.
    Duty_AC = AC_Mag/V_DCin;

    if(Duty_AC > 0.9)
    {
        Duty_AC = 0.9;
    }
    else if(Duty_AC < 0.0)
    {
        Duty_AC = 0.0;
    }

    Step_sin = (Sin_Counter/Prd_Sw)*pi_2;
    Duty_AC = Duty_AC*INV_PWM_TBPRD;

    // Check SW1 Status and base motor rotation on reading
    if(GpioDataRegs.GPADAT.bit.GPIO14 == 1)
    {
        Sine1 = ((sin(Step_sin+0)+1)/2)*Duty_AC;
        Sine2 = ((sin(Step_sin-phase_offset)+1)/2)*Duty_AC;
        Sine3 = ((sin(Step_sin+phase_offset)+1)/2)*Duty_AC;
    }
    else
    {
        Sine1 = ((sin(Step_sin+0)+1)/2)*Duty_AC;
        Sine3 = ((sin(Step_sin-phase_offset)+1)/2)*Duty_AC;
        Sine2 = ((sin(Step_sin+phase_offset)+1)/2)*Duty_AC;
    }


    //// Check System Enable and enable CPLD to pass PWM/Relay/LED output signals
    // Signals passed by CPLD when GPIO_60 = '1' and GPIO_61 = '0'
    if((Sys_En & 0x0001) == 0x0001)
    {
        GpioDataRegs.GPBSET.bit.GPIO60 = 1;
        GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
    }
    else
    {
        GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
        GpioDataRegs.GPBSET.bit.GPIO61 = 1;
    }


    //// Update PWM Values and Check if PWM Enable and Inverter Enable are asserted
    if((Sys_En & 0x0003) == 0x0003)
    {
        switch(Inv_OPMode)          //[0=Passive Rectifier; 1=Islanded; 2=Grid-Forming; 3=Grid-Feeding; 4=Grid-Supporting]
        {
            case 0x0000:            // Passive Rectifier
                EPwm1Regs.CMPA.half.CMPA = 0x0000;
                EPwm2Regs.CMPA.half.CMPA = 0x0000;
                EPwm3Regs.CMPA.half.CMPA = 0x0000;
                break;
            case 0x0001:            // Islanded
                EPwm1Regs.CMPA.half.CMPA = (Uint16) Sine1;
                EPwm2Regs.CMPA.half.CMPA = (Uint16) Sine2;
                EPwm3Regs.CMPA.half.CMPA = (Uint16) Sine3;
                break;
            case 0x0002:            // Grid-Forming
                EPwm1Regs.CMPA.half.CMPA = (Uint16) Sine1;
                EPwm2Regs.CMPA.half.CMPA = (Uint16) Sine2;
                EPwm3Regs.CMPA.half.CMPA = (Uint16) Sine3;
                break;
            case 0x0003:            // Grid-Feeding
                EPwm1Regs.CMPA.half.CMPA = 0x0000;
                EPwm2Regs.CMPA.half.CMPA = 0x0000;
                EPwm3Regs.CMPA.half.CMPA = 0x0000;
                break;
            case 0x0004:            // Grid-Supporting
                EPwm1Regs.CMPA.half.CMPA = 0x0000;
                EPwm2Regs.CMPA.half.CMPA = 0x0000;
                EPwm3Regs.CMPA.half.CMPA = 0x0000;
                break;

            default:
                EPwm1Regs.CMPA.half.CMPA = 0x0000;
                EPwm2Regs.CMPA.half.CMPA = 0x0000;
                EPwm3Regs.CMPA.half.CMPA = 0x0000;
                break;
        }

    }
    else
    {
        EPwm1Regs.CMPA.half.CMPA = 0x0000;
        EPwm2Regs.CMPA.half.CMPA = 0x0000;
        EPwm3Regs.CMPA.half.CMPA = 0x0000;
    }



    //// Update PWM Values and Check if PWM Enable and Buck Enable are asserted
    if((Sys_En & 0x0005) == 0x0005)
    {

       switch(Buck_OPMode)          // [0=Passive; 1=Voltage-Mode; 2=Current-Mode; 3=Dual-Loop]
       {
           case 0x0000:            // Passive
                // Update Buck Duty Cycle
                EPwm4Regs.CMPA.half.CMPA= 0x0000;

                //// Reset Integral Storage on unused PI controllers
                Buck_V_PI.i10 = 0;
                Buck_I_PI.i10 = 0;
               break;

           case 0x0001:            // Voltage-Mode
               // Run Buck Voltage PI Controller
               Buck_V_PI_uk = DCL_runPI(&Buck_V_PI, Buck_V_Cmd, V_Buck);
               // Update Buck Duty Cycle
               EPwm4Regs.CMPA.half.CMPA= (Uint16) (Buck_V_PI_uk);

               //// Reset Integral Storage on unused PI controllers
               //Buck_V_PI.i10 = 0;
               Buck_I_PI.i10 = 0;
               break;


           case 0x0002:            // Current-Mode
                // Run Buck Current PI Controller
                Buck_I_PI_uk = DCL_runPI(&Buck_I_PI, Buck_I_Cmd, I_Buck);
                // Update Buck Duty Cycle
                EPwm4Regs.CMPA.half.CMPA= (Uint16) (Buck_I_PI_uk);

                //// Reset Integral Storage on unused PI controllers
                Buck_V_PI.i10 = 0;
                //Buck_I_PI.i10 = 0;
               break;

           case 0x0003:            // Dual-Loop (Not Implemented)
               // Update Buck Duty Cycle
               EPwm4Regs.CMPA.half.CMPA= 0x0000;

               //// Reset Integral Storage on unused PI controllers
               Buck_V_PI.i10 = 0;
               Buck_I_PI.i10 = 0;
               break;

           default:
               // Update Buck Duty Cycle
               EPwm4Regs.CMPA.half.CMPA= 0x0000;

               //// Reset Integral Storage on unused PI controllers
               Buck_V_PI.i10 = 0;
               Buck_I_PI.i10 = 0;
               break;
       }

    }
    else
    {
       // Update Buck Duty Cycle
       EPwm4Regs.CMPA.half.CMPA= 0x0000;

       //// Reset Integral Storage on unused PI controllers
       Buck_V_PI.i10 = 0;
       Buck_I_PI.i10 = 0;
    }





    //// Update PWM Values and Check if PWM Enable and Boost Enable are asserted
    if((Sys_En & 0x0009) == 0x0009)
    {

      switch(Boost_OPMode)          // [0=Passive; 1=Voltage-Mode; 2=Current-Mode; 3=Dual-Loop]
      {
          case 0x0000:            // Passive
              // Update Boost Duty Cycle
              EPwm4Regs.CMPB= 0x0000;

              //// Reset Integral Storage on unused PI controllers
              Boost_V_PI.i10 = 0;
              Boost_I_PI.i10 = 0;
              break;

          case 0x0001:            // Voltage-Mode
              // Run Boost Voltage PI Controller
              Boost_V_PI_uk = DCL_runPI(&Boost_V_PI, Boost_V_Cmd, V_Boost);

              // Update Boost Duty Cycle
              EPwm4Regs.CMPB = (Uint16) (Boost_V_PI_uk);

              //// Reset Integral Storage on unused PI controllers
              //Boost_V_PI.i10 = 0;
              Boost_I_PI.i10 = 0;
              break;


          case 0x0002:            // Current-Mode (Not Tested)
              /*
                // Run Boost Current PI Controller
                Boost_I_PI_uk = DCL_runPI(&Boost_I_PI, Boost_I_Cmd, I_Boost);

                // Update Boost Duty Cycle
                EPwm4Regs.CMPB = (Uint16) (Boost_I_PI_uk);

                //// Reset Integral Storage on unused PI controllers
                Boost_V_PI.i10 = 0;
                //Boost_I_PI.i10 = 0;
              */
              // Update Boost Duty Cycle
              EPwm4Regs.CMPB= 0x0000;

              //// Reset Integral Storage on unused PI controllers
              Boost_V_PI.i10 = 0;
              Boost_I_PI.i10 = 0;
              break;

          case 0x0003:            // Dual-Loop (Not Implemented)
              // Update Boost Duty Cycle
              EPwm4Regs.CMPB= 0x0000;

              //// Reset Integral Storage on unused PI controllers
              Boost_V_PI.i10 = 0;
              Boost_I_PI.i10 = 0;
              break;

          default:
              // Update Boost Duty Cycle
              EPwm4Regs.CMPB= 0x0000;

              //// Reset Integral Storage on unused PI controllers
              Boost_V_PI.i10 = 0;
              Boost_I_PI.i10 = 0;
              break;
      }

    }
    else
    {
      // Update Boost Duty Cycle
      EPwm4Regs.CMPB= 0x0000;

      //// Reset Integral Storage on unused PI controllers
      Boost_V_PI.i10 = 0;
      Boost_I_PI.i10 = 0;
    }


    // Reinitialize for next ADC sequence
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE


    return;
}


interrupt void spiTxFifoIsr(void)
{
    int i;

    // Send Startup/Configuration Commands to SPI ADC
    if(SPI_ADC0_Cmd_Cnt == 0)
    {

        //Setup SPI ADC for Continuous Read
        SpiaRegs.SPITXBUF = SPI_Wakeup; //Send Dummy Data to SPI
        SpiaRegs.SPITXBUF = SPI_Wakeup; //Send Dummy Data to SPI
        SpiaRegs.SPITXBUF = SPI_Cmd;    //Send read all cmd to SPI
        SpiaRegs.SPITXBUF = SPI_Dummy;  //Send dummy cmd to SPI
        SpiaRegs.SPITXBUF = SPI_Dummy;  //Send dummy cmd to SPI
        SpiaRegs.SPITXBUF = SPI_Dummy;  //Send dummy cmd to SPI
        SpiaRegs.SPITXBUF = SPI_Dummy;  //Send dummy cmd to SPI
        SpiaRegs.SPITXBUF = SPI_Dummy;  //Send dummy cmd to SPI
    }
    else
    {
        for(i=0;i<8;i++)
        {

            SpiaRegs.SPITXBUF = SPI_Dummy;  //Send dummy cmd to SPI
        }
    }


    // Non Critical Check For LEDs
    if(SPI_ADC0_Cmd_Cnt == 1000)
    {
        // Check Potentiometer and update LEDs based on voltage
         if(V_Pot<1.0)
         {
             GpioDataRegs.GPASET.bit.GPIO31 = 1;     //turn off 1st Red LED (Active Low)
             GpioDataRegs.GPBSET.bit.GPIO34 = 1;     //turn off 2nd Red LED (Active Low)
             GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;   //turn off Board LED1
             GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;   //turn off Board LED2
             GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;   //turn off Board LED3
             GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;   //turn off Board LED4
             GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;   //turn off Board LED5
             GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;   //turn off Board LED6
             GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;   //turn off Board LED7
             GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;   //turn off Board LED8
             GpioDataRegs.GPASET.bit.GPIO8 = 1;   //turn off Board LED9 (Sw8 Active Low)
             GpioDataRegs.GPASET.bit.GPIO9 = 1;   //turn off Board LED10 (Sw8 Active Low)
         }
         else if(V_Pot>1.0 && V_Pot<2.0 )
         {
             GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;   //turn on 1st Red LED
             GpioDataRegs.GPBSET.bit.GPIO34 = 1;     //turn off 2nd Red LED
             GpioDataRegs.GPASET.bit.GPIO10 = 1;   //turn on Board LED1
             GpioDataRegs.GPASET.bit.GPIO11 = 1;   //turn on Board LED2
             GpioDataRegs.GPASET.bit.GPIO12 = 1;   //turn on Board LED3
             GpioDataRegs.GPASET.bit.GPIO13 = 1;   //turn on Board LED4
             GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;   //turn off Board LED5
             GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;   //turn off Board LED6
             GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;   //turn off Board LED7
             GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;   //turn off Board LED8
             GpioDataRegs.GPASET.bit.GPIO8 = 1;   //turn off Board LED9 (Sw8 Active Low)
             GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;   //turn on Board LED10 (Sw8 Active Low)
         }
         else if(V_Pot>2.0 && V_Pot<2.5 )
         {
             GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;   //turn on 1st Red LED
             GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;   //turn on 2nd Red LED
             GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;   //turn off Board LED1
             GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;   //turn off Board LED2
             GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;   //turn off Board LED3
             GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;   //turn off Board LED4
             GpioDataRegs.GPASET.bit.GPIO24 = 1;   //turn on Board LED5
             GpioDataRegs.GPASET.bit.GPIO25 = 1;   //turn on Board LED6
             GpioDataRegs.GPASET.bit.GPIO26 = 1;   //turn on Board LED7
             GpioDataRegs.GPASET.bit.GPIO27 = 1;   //turn on Board LED8
             GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;   //turn on Board LED9 (Sw8 Active Low)
             GpioDataRegs.GPASET.bit.GPIO9 = 1;   //turn off Board LED10 (Sw8 Active Low)
         }
         else
         {
             GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;   //turn on 1st Red LED
             GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;   //turn on 2nd Red LED
             GpioDataRegs.GPASET.bit.GPIO10 = 1;   //turn on Board LED1
             GpioDataRegs.GPASET.bit.GPIO11 = 1;   //turn on Board LED2
             GpioDataRegs.GPASET.bit.GPIO12 = 1;   //turn on Board LED3
             GpioDataRegs.GPASET.bit.GPIO13 = 1;   //turn on Board LED4
             GpioDataRegs.GPASET.bit.GPIO24 = 1;   //turn on Board LED5
             GpioDataRegs.GPASET.bit.GPIO25 = 1;   //turn on Board LED6
             GpioDataRegs.GPASET.bit.GPIO26 = 1;   //turn on Board LED7
             GpioDataRegs.GPASET.bit.GPIO27 = 1;   //turn on Board LED8
             GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;   //turn on Board LED9 (Sw8 Active Low)
             GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;   //turn on Board LED10 (Sw8 Active Low)
         }
    }




    if(SPI_ADC0_Cmd_Cnt < 1000)
        SPI_ADC0_Cmd_Cnt++;
    else
        SPI_ADC0_Cmd_Cnt = 0;


    SpiaRegs.SPIFFTX.bit.TXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=PIEACK_GROUP6;       // Issue PIE ACK

}

interrupt void spiRxFifoIsr(void)
{
    int i;

    for(i=0;i<8;i++)
    {
        SPI_ADC0_temp = SpiaRegs.SPIRXBUF;

        switch(SPI_ADC0_temp & 0xF000)   //Check Address and Switch based on this
        {
            case 0x0000:
                SPI_ADC0_rdata[0] = (SPI_ADC0_temp & 0x0FFF);         //Remove address data
                mb.holdingRegisters.Mod_Reg[144] =   SPI_ADC0_rdata[0];
                I_DCin2 = (mb.holdingRegisters.Mod_Reg[144]*I_Scale2)-I_Offset_ACS714_05B;           // [V_ADC*(5/4095) * (185mV/A)^-1] -13.5
                break;
            case 0x1000:
                SPI_ADC0_rdata[1] = (SPI_ADC0_temp & 0x0FFF);         //Remove address data
                mb.holdingRegisters.Mod_Reg[145] =   SPI_ADC0_rdata[1];
                I_L1 = (mb.holdingRegisters.Mod_Reg[145]*I_Scale2)-I_Offset_ACS714_05B;              // [V_ADC*(5/4095) * (185mV/A)^-1] -13.5
                break;
            case 0x2000:
                SPI_ADC0_rdata[2] = (SPI_ADC0_temp & 0x0FFF);         //Remove address data
                mb.holdingRegisters.Mod_Reg[146] =   SPI_ADC0_rdata[2];
                I_L2 = (mb.holdingRegisters.Mod_Reg[146]*I_Scale2)-I_Offset_ACS714_05B;              // [V_ADC*(5/4095) * (185mV/A)^-1] -13.5
                break;
            case 0x3000:
                SPI_ADC0_rdata[3] = (SPI_ADC0_temp & 0x0FFF);         //Remove address data
                mb.holdingRegisters.Mod_Reg[147] =   SPI_ADC0_rdata[3];
                I_L3 = (mb.holdingRegisters.Mod_Reg[147]*I_Scale2)-I_Offset_ACS714_05B;              // [V_ADC*(5/4095) * (185mV/A)^-1] -13.5
                break;
            case 0x4000:
                SPI_ADC0_rdata[4] = (SPI_ADC0_temp & 0x0FFF);         //Remove address data
                mb.holdingRegisters.Mod_Reg[148] =   SPI_ADC0_rdata[4];
                I_Buck = (mb.holdingRegisters.Mod_Reg[148]*I_Scale2)-I_Offset_ACS714_05B;            // [V_ADC*(5/4095) * (185mV/A)^-1] -13.5
                break;
            case 0x5000:
                SPI_ADC0_rdata[5] = (SPI_ADC0_temp & 0x0FFF);         //Remove address data
                mb.holdingRegisters.Mod_Reg[149] =   SPI_ADC0_rdata[5];
                I_Boost = (mb.holdingRegisters.Mod_Reg[149]*I_Scale2)-I_Offset_ACS714_05B;           // [V_ADC*(5/4095) * (185mV/A)^-1] -13.5
                break;
            case 0x6000:
                SPI_ADC0_rdata[6] = (SPI_ADC0_temp & 0x0FFF);         //Remove address data
                mb.holdingRegisters.Mod_Reg[150] =   SPI_ADC0_rdata[6];
                Ia = (mb.holdingRegisters.Mod_Reg[150]*I_Scale2)-I_Offset_ACS714_05B;                // [V_ADC*(5/4095) * (185mV/A)^-1] -13.5
                break;
            case 0x7000:
                SPI_ADC0_rdata[7] = (SPI_ADC0_temp & 0x0FFF);         //Remove address data
                mb.holdingRegisters.Mod_Reg[151] =   SPI_ADC0_rdata[7];
                I_L5 =(mb.holdingRegisters.Mod_Reg[151]*I_Scale2)-I_Offset_ACS714_05B;               // [V_ADC*(5/4095) * (185mV/A)^-1] -13.5
                break;

            default:
                break;
        }

    }


    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ack

}

interrupt void Mcbsp_TxINTA_ISR(void)
{

    switch(SPI_ADC1_Cmd_Cnt)   //Check Counter and Switch based on this
            {
                case 0:
                    McbspaRegs.DXR1.all = SPI_Wakeup;
                    SPI_ADC1_Cmd_Cnt++;
                    break;
                case 1:
                    McbspaRegs.DXR1.all = SPI_Wakeup;
                    SPI_ADC1_Cmd_Cnt++;
                    break;
                case 2:
                    McbspaRegs.DXR1.all = SPI_Cmd;
                    SPI_ADC1_Cmd_Cnt++;
                    break;
                case 1000:
                    McbspaRegs.DXR1.all = SPI_Dummy;
                    SPI_ADC1_Cmd_Cnt =0;
                    break;
                default:
                    McbspaRegs.DXR1.all = SPI_Dummy;
                    SPI_ADC1_Cmd_Cnt++;
                    break;
            }

    //McbspaRegs.DXR1.all= sdata;
    //sdata = (sdata+1)& 0x00FF ;

    // To receive more interrupts from this PIE group, acknowledge this interrupt
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP6;

}

interrupt void Mcbsp_RxINTA_ISR(void)
{
    SPI_ADC1_temp = McbspaRegs.DRR1.all;


    switch(SPI_ADC1_temp & 0xF000)   //Check Address and Switch based on this
            {
                case 0x0000:
                    SPI_ADC1_rdata[0] = (SPI_ADC1_temp & 0x0FFF);         //Remove address data
                    mb.holdingRegisters.Mod_Reg[152] =   SPI_ADC1_rdata[0];
                    //PV_V_PV = SPI_ADC1_rdata[0]*V_Scale3;
                    break;
                case 0x1000:
                    SPI_ADC1_rdata[1] = (SPI_ADC1_temp & 0x0FFF);         //Remove address data
                    mb.holdingRegisters.Mod_Reg[153] =   SPI_ADC1_rdata[1];
                    //PV_V_Bst = SPI_ADC1_rdata[1]*V_Scale3;
                    //ECap1Regs.CAP4 = PV_Bst_DC;                 // Set Compare Shadow Register (On-Time) value
                    //ECap2Regs.CAP4 = PV_Bst_DC;                 // Set Compare Shadow Register (On-Time) value
                    //PV_Run_Flag = 1;
                    break;
                case 0x2000:
                    SPI_ADC1_rdata[2] = (SPI_ADC1_temp & 0x0FFF);         //Remove address data
                    mb.holdingRegisters.Mod_Reg[154] =   SPI_ADC1_rdata[2];
                    //PV_V_Bus = SPI_ADC1_rdata[2]*V_Scale5;
                    break;
                case 0x3000:
                    SPI_ADC1_rdata[3] = (SPI_ADC1_temp & 0x0FFF);         //Remove address data
                    mb.holdingRegisters.Mod_Reg[155] =   SPI_ADC1_rdata[3];
                    //PV_I_PV = (SPI_ADC1_rdata[3]*I_Scale_ACS714_30A_5V)-I_Offset_ACS714_30A_3V;        // [V_ADC*(5/4095) * (66mV/A)^-1] - 37.8788
                    break;
                case 0x4000:
                    SPI_ADC1_rdata[4] = (SPI_ADC1_temp & 0x0FFF);         //Remove address data
                    mb.holdingRegisters.Mod_Reg[156] =   SPI_ADC1_rdata[4];
                    //PV_I_L1 = (SPI_ADC1_rdata[4]*I_Scale_ACS714_30A_5V)-I_Offset_ACS714_30A_3V;        // [V_ADC*(5/4095) * (66mV/A)^-1] - 37.8788
                    break;
                case 0x5000:
                    SPI_ADC1_rdata[5] = (SPI_ADC1_temp & 0x0FFF);         //Remove address data
                    mb.holdingRegisters.Mod_Reg[157] =   SPI_ADC1_rdata[5];
                    //PV_I_L2 = (SPI_ADC1_rdata[5]*I_Scale_ACS714_30A_5V)-I_Offset_ACS714_30A_3V;        // [V_ADC*(5/4095) * (66mV/A)^-1] - 37.8788
                    break;
                case 0x6000:
                    SPI_ADC1_rdata[6] = (SPI_ADC1_temp & 0x0FFF);         //Remove address data
                    mb.holdingRegisters.Mod_Reg[158] =   SPI_ADC1_rdata[6];
                    //PV_I_Bst = (SPI_ADC1_rdata[6]*I_Scale_ACS714_30A_5V)-I_Offset_ACS714_30A_3V;        // [V_ADC*(5/4095) * (66mV/A)^-1] - 37.8788
                    break;
                case 0x7000:
                    SPI_ADC1_rdata[7] = (SPI_ADC1_temp & 0x0FFF);         //Remove address data
                    mb.holdingRegisters.Mod_Reg[159] =   SPI_ADC1_rdata[7];
                    //PV_I_Bus =(SPI_ADC1_rdata[7]*I_Scale_ACS714_30A_5V)-I_Offset_ACS714_30A_3V;        // [V_ADC*(5/4095) * (66mV/A)^-1] - 37.8788
                    break;

                default:
                    break;
            }
    //rdata_point = (rdata_point+1) & 0x00FF;
    // To receive more interrupts from this PIE group, acknowledge this interrupt
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP6;
}



interrupt void Mcbsp_TxINTB_ISR(void)
{


    switch(SPI_ADC2_Cmd_Cnt)   //Check Counter and Switch based on this
            {
                case 0:
                    McbspbRegs.DXR1.all = SPI_Wakeup;
                    SPI_ADC2_Cmd_Cnt++;
                    break;
                case 1:
                    McbspbRegs.DXR1.all = SPI_Wakeup;
                    SPI_ADC2_Cmd_Cnt++;
                    break;
                case 2:
                    McbspbRegs.DXR1.all = SPI_Cmd;
                    SPI_ADC2_Cmd_Cnt++;
                    break;
                case 1000:
                    McbspbRegs.DXR1.all = SPI_Dummy;
                    SPI_ADC2_Cmd_Cnt =0;
                    break;
                default:
                    McbspbRegs.DXR1.all = SPI_Dummy;
                    SPI_ADC2_Cmd_Cnt++;
                    break;
            }

    //McbspaRegs.DXR1.all= sdata;
    //sdata = (sdata+1)& 0x00FF ;

    // To receive more interrupts from this PIE group, acknowledge this interrupt
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP6;
}

interrupt void Mcbsp_RxINTB_ISR(void)
{
    SPI_ADC2_temp = McbspbRegs.DRR1.all;


    switch(SPI_ADC2_temp & 0xF000)   //Check Address and Switch based on this
            {
                case 0x0000:
                    SPI_ADC2_rdata[0] = (SPI_ADC2_temp & 0x0FFF);         //Remove address data
                    mb.holdingRegisters.Mod_Reg[160] =   SPI_ADC2_rdata[0];
                    //Therm_T0 = (SPI_ADC2_rdata[0]*T_Scale_Therm)-T_Offset_Therm;
                    break;
                case 0x1000:
                    SPI_ADC2_rdata[1] = (SPI_ADC2_temp & 0x0FFF);         //Remove address data
                    mb.holdingRegisters.Mod_Reg[161] =   SPI_ADC2_rdata[1];
                    //Therm_T1 = (SPI_ADC2_rdata[1]*T_Scale_Therm)-T_Offset_Therm;
                    break;
                case 0x2000:
                    SPI_ADC2_rdata[2] = (SPI_ADC2_temp & 0x0FFF);         //Remove address data
                    mb.holdingRegisters.Mod_Reg[162] =   SPI_ADC2_rdata[2];
                    //Therm_T2 = (SPI_ADC2_rdata[2]*T_Scale_Therm)-T_Offset_Therm;
                    break;
                case 0x3000:
                    SPI_ADC2_rdata[3] = (SPI_ADC2_temp & 0x0FFF);         //Remove address data
                    mb.holdingRegisters.Mod_Reg[163] =   SPI_ADC2_rdata[3];
                    //Therm_T3 = (SPI_ADC2_rdata[3]*T_Scale_Therm)-T_Offset_Therm;
                    break;
                case 0x4000:
                    SPI_ADC2_rdata[4] = (SPI_ADC2_temp & 0x0FFF);         //Remove address data
                    mb.holdingRegisters.Mod_Reg[164] =   SPI_ADC2_rdata[4];
                    //Therm_T4 = (SPI_ADC2_rdata[4]*T_Scale_Therm)-T_Offset_Therm;
                    break;
                case 0x5000:
                    SPI_ADC2_rdata[5] = (SPI_ADC2_temp & 0x0FFF);         //Remove address data
                    mb.holdingRegisters.Mod_Reg[165] =   SPI_ADC2_rdata[5];
                    //Therm_T5 = (SPI_ADC2_rdata[5]*T_Scale_Therm)-T_Offset_Therm;
                    break;
                case 0x6000:
                    SPI_ADC2_rdata[6] = (SPI_ADC2_temp & 0x0FFF);         //Remove address data
                    mb.holdingRegisters.Mod_Reg[166] =   SPI_ADC2_rdata[6];
                    //Therm_T6 = (SPI_ADC2_rdata[6]*T_Scale_Therm)-T_Offset_Therm;
                    break;
                case 0x7000:
                    SPI_ADC2_rdata[7] = (SPI_ADC2_temp & 0x0FFF);         //Remove address data
                    mb.holdingRegisters.Mod_Reg[167] =   SPI_ADC2_rdata[7];
                    //Therm_T7 = (SPI_ADC2_rdata[7]*T_Scale_Therm)-T_Offset_Therm;
                    break;

                default:
                    break;
            }



    //rdata_point = (rdata_point+1) & 0x00FF;
    // To receive more interrupts from this PIE group, acknowledge this interrupt
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP6;
}


void InitEPWM1()
{
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm1Regs.TBPRD = INV_PWM_TBPRD;                    // Period = 2* 1000 TBCLK counts => 30 kHz

    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Enable phase loading
    EPwm1Regs.TBPHS.half.TBPHS = 0;
    EPwm1Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;     //Sync down-stream module
    EPwm1Regs.TBCTR = 0x0000;                       // Clear counter
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;     // Load registers every ZERO
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Setup compare
    EPwm1Regs.CMPA.half.CMPA = 0;
    EPwm1Regs.CMPB = 0;

    // Set actions
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;


    // Setup Deadband
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBRED = Deadtime;  //30;
    EPwm1Regs.DBFED = Deadtime;  //30;


    //ADC Triggering Setup
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;
    EPwm1Regs.ETSEL.bit.SOCASEL = 0x2;//2;//0x2;       // Select SOC from from CPMA on Period
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;        // Generate pulse every event
}

void InitEPWM2()
{
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm2Regs.TBPRD = INV_PWM_TBPRD;                        // Period = 2*1000 TBCLK counts => 30 kHz
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;             // Enable phase loading
    EPwm2Regs.TBPHS.half.TBPHS = 0;
    EPwm2Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
    EPwm2Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Setup compare
    EPwm2Regs.CMPA.half.CMPA = 0; //fixed 50% duty cycle
    EPwm2Regs.CMPB = 0;

    // Set actions qualifiers
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;

    //EPwm2Regs.ETSEL.bit.SOCAEN = 0;        // Disable SOC on A group


    // Setup Deadband
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBRED = Deadtime;
    EPwm2Regs.DBFED = Deadtime;

}

void InitEPWM3()
{
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm3Regs.TBPRD = INV_PWM_TBPRD;               // Period = 2*1000 TBCLK counts => 30 kHz
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm3Regs.TBPHS.half.TBPHS = 0;
    EPwm3Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;  // sync flow-through
    EPwm3Regs.TBCTR = 0x0000;                   // Clear counter
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;    // Clock ratio to SYSCLKOUT
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Setup shadow register load on ZERO
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set Compare values
    EPwm3Regs.CMPA.half.CMPA = 0;    // Set compare A value
    EPwm3Regs.CMPB = 0;

    // Set action qualifiers
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm3Regs.AQCTLB.bit.CBD = AQ_SET;

    //EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
    //EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    //EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;
    //EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    // Setup Deadband
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm3Regs.DBRED = Deadtime;
    EPwm3Regs.DBFED = Deadtime;

}


void InitEPWM4()
{
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm4Regs.TBPRD = Conv_PWM_TBPRD;               // Period = 2*1000 TBCLK counts => 30 kHz
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm4Regs.TBPHS.half.TBPHS = 0;
    EPwm4Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;  // sync flow-through
    EPwm4Regs.TBCTR = 0x0000;                   // Clear counter
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;    // Clock ratio to SYSCLKOUT
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Setup shadow register load on ZERO
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set Compare values
    EPwm4Regs.CMPA.half.CMPA = 0;    // Set compare A value
    EPwm4Regs.CMPB = 0;

    // Set action qualifiers
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm4Regs.AQCTLB.bit.CBD = AQ_SET;

    //EPwm4Regs.AQCTLA.bit.CAU = AQ_SET;
    //EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    //EPwm4Regs.AQCTLB.bit.CBU = AQ_SET;
    //EPwm4Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    // Setup Deadband
//      EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
//      EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
//      EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;
//      EPwm4Regs.DBRED = Deadtime;
//      EPwm4Regs.DBFED = Deadtime;

}


void InitEPWM5()
{
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm5Regs.TBPRD = PWM_TBPRD;               // Period = 2*1000 TBCLK counts => 30 kHz
    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm5Regs.TBPHS.half.TBPHS = 0;
    EPwm5Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm5Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;  // sync flow-through
    EPwm5Regs.TBCTR = 0x0000;                   // Clear counter
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;    // Clock ratio to SYSCLKOUT
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Setup shadow register load on ZERO
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set Compare values
    EPwm5Regs.CMPA.half.CMPA = 0;    // Set compare A value
    EPwm5Regs.CMPB = 0;

    // Set action qualifiers
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm5Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm5Regs.AQCTLB.bit.CBD = AQ_SET;

    //EPwm5Regs.AQCTLA.bit.CAU = AQ_SET;
    //EPwm5Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    //EPwm5Regs.AQCTLB.bit.CBU = AQ_SET;
    //EPwm5Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    // Setup Deadband
    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm5Regs.DBRED = Deadtime;
    EPwm5Regs.DBFED = Deadtime;

}

void InitEPWM6()
{
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm6Regs.TBPRD = PWM_TBPRD;               // Period = 2*1000 TBCLK counts => 30 kHz
    EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm6Regs.TBPHS.half.TBPHS = 0;
    EPwm6Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm6Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;  // sync flow-through
    EPwm6Regs.TBCTR = 0x0000;                   // Clear counter
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;    // Clock ratio to SYSCLKOUT
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Setup shadow register load on ZERO
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set Compare values
    EPwm6Regs.CMPA.half.CMPA = 0;    // Set compare A value
    EPwm6Regs.CMPB = 0;

    // Set action qualifiers
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm6Regs.AQCTLB.bit.CBD = AQ_SET;

    //EPwm6Regs.AQCTLA.bit.CAU = AQ_SET;
    //EPwm6Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    //EPwm6Regs.AQCTLB.bit.CBU = AQ_SET;
    //EPwm6Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    // Setup Deadband
    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm6Regs.DBRED = Deadtime;
    EPwm6Regs.DBFED = Deadtime;

}


void InitAPWM()
{
    //// When ECAP Modules are configured in APWM mode:
    // CAP1 is used to set the initial period and CAP2 is using to set the initial compare (on-time for active-high configuration)
    // During Operation, CAP3 is used to set the period and CAP4 is used to set the compare as "Shadow" Registers
    // TPWM = (CAP1+1)*T_TSCTR       [FPWM = 1/TPWM]
    // Modules ECAP1 through ECAP3 and ECAP4 through ECAP6 are able to be synchronized


    //// Dual-Interleaved Boost Converter APWM Setup (ECAP1 and ECAP2 Synchronized)
    // Setup APWM mode on CAP1, set period(CAP1) and compare(CAP2) registers
    ECap1Regs.ECCTL2.bit.CAP_APWM = 1;  // Enable APWM mode
    //ECap1Regs.CAP1 = 1500;            // Set period value     [((1/150e6)*1500)^-1 = 100 kHz]
    //ECap1Regs.CAP2 = 150;             // Set Compare (On-Time) value [10%]
    ECap1Regs.CAP1 = PWM_TBPRD;      // Set period value     [((1/150e6)*3000)^-1 = 50 kHz]
    ECap1Regs.CAP2 = 0;                 // Set Compare (On-Time) value [0%]
    //ECap1Regs.CAP4 = 0;                 // Set "Shadow" Compare (On-Time) value [0%]
    ECap1Regs.CTRPHS = 0;               // Reference Phase (0 for Master)
    ECap1Regs.ECCTL2.bit.APWMPOL = 0;   // Active-High Mode
    ECap1Regs.ECCTL2.bit.SYNCI_EN = 0;  // No sync in for Master
    ECap1Regs.ECCTL2.bit.SYNCO_SEL = 1; // Configure Sync-Out for Master
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1; // Allow TSCTR to run (Free-Running)

    // Setup APWM mode on CAP2, set period and compare registers
    ECap2Regs.ECCTL2.bit.CAP_APWM = 1;  // Enable APWM mode
    //ECap2Regs.CAP1 = 1500;            // Set period value     [((1/150e6)*1500)^-1 = 100 kHz]
    //ECap2Regs.CAP2 = 150;             // Set Compare (On-Time) value [10%]
    ECap2Regs.CAP1 = PWM_TBPRD;      // Set period value     [((1/150e6)*3000)^-1 = 50 kHz]
    ECap2Regs.CAP2 = 0;                 // Set Compare (On-Time) value [0%]
    //ECap2Regs.CAP4 = 0;                 // Set "Shadow" Compare (On-Time) value [0%]
    //ECap2Regs.CTRPHS = 750;           // Reference Phase (1500 - 750 => 180 degrees)
    ECap2Regs.CTRPHS = PWM_TBPRD/2;  // Reference Phase (3000 - 1500 => 180 degrees)
    ECap2Regs.ECCTL2.bit.APWMPOL = 0;   // Active-High Mode

    ECap2Regs.ECCTL2.bit.SYNCI_EN = 1;  // Enable Sync-In
    ECap2Regs.ECCTL2.bit.SYNCO_SEL = 2; // Disable Sync-Out Signal (End of Chain)
    ECap2Regs.ECCTL2.bit.TSCTRSTOP = 1; // Allow TSCTR to run (Free-Running)
    //End Dual-Interleaved Boost Converter APWM Setup

    //// Resonant Converter APWM Setup (ECAP4 and ECAP5 Synchronized)
    // Setup APWM mode on CAP4, set period(CAP1) and compare(CAP2) registers
    ECap4Regs.ECCTL2.bit.CAP_APWM = 1;  // Enable APWM mode
    ECap4Regs.CAP1 = 3000;              // Set period value     [((1/150e6)*3000)^-1 = 50 kHz]
    ECap4Regs.CAP2 = 1500;              // Set Compare (On-Time) value [50%]
    //ECap4Regs.CAP1 = 1500;              // Set period value     [((1/150e6)*1500)^-1 = 100 kHz]
    //ECap4Regs.CAP2 = 750;              // Set Compare (On-Time) value [50%]
    //ECap4Regs.CAP2 = 0;
    //ECap4Regs.CAP1 = 1000;              // Set period value     [((1/150e6)*1000)^-1 = 150 kHz]
    //ECap4Regs.CAP2 = 500;              // Set Compare (On-Time) value [50%]
    //ECap4Regs.CAP1 = 750;              // Set period value     [((1/150e6)*750)^-1 = 200 kHz]
    //ECap4Regs.CAP2 = 375;              // Set Compare (On-Time) value [50%]
    //ECap4Regs.CAP1 = 600;              // Set period value     [((1/150e6)*600)^-1 = 250 kHz]
    //ECap4Regs.CAP2 = 300;              // Set Compare (On-Time) value [50%]
    //ECap4Regs.CAP2 = 0;              // Set Compare (On-Time) value [0%] for testing
    ECap4Regs.CTRPHS = 0;               // Reference Phase (0 for Master)
    ECap4Regs.ECCTL2.bit.APWMPOL = 0;   // Active-High Mode
    ECap4Regs.ECCTL2.bit.SYNCI_EN = 0;  // No sync in for Master
    ECap4Regs.ECCTL2.bit.SYNCO_SEL = 1; // Configure Sync-Out for Master
    ECap4Regs.ECCTL2.bit.TSCTRSTOP = 1; // Allow TSCTR to run (Free-Running)

    // Setup APWM mode on CAP5, set period and compare registers (Set this Shadow register 1st to allow appropriate deadtime)
    ECap5Regs.ECCTL2.bit.CAP_APWM = 1;  // Enable APWM mode
    ECap5Regs.CAP1 = 3000;              // Set period value     [((1/150e6)*3000)^-1 = 50 kHz]
    ECap5Regs.CAP2 = 1550;              // Set Compare (On-Time) value [50%] (Slightly "More" to account for Shoot-Through Protection [50 cycles each side])
    //ECap5Regs.CAP1 = 1500;              // Set period value     [((1/150e6)*1500)^-1 = 100 kHz]
    //ECap5Regs.CAP2 = 850;              // Set Compare (On-Time) value [50%] (Slightly "More" to account for Shoot-Through Protection [50 cycles each side])
    //ECap5Regs.CAP2 = 1500;
    //ECap5Regs.CAP1 = 1000;              // Set period value     [((1/150e6)*1000)^-1 = 150 kHz]
    //ECap5Regs.CAP2 = 600;              // Set Compare (On-Time) value [50%] (Slightly "More" to account for Shoot-Through Protection [25 cycles each side])
    //ECap5Regs.CAP1 = 750;              // Set period value     [((1/150e6)*750)^-1 = 200 kHz]
    //ECap5Regs.CAP2 = 425;              // Set Compare (On-Time) value [50%] (Slightly "More" to account for Shoot-Through Protection [25 cycles each side])
    //ECap5Regs.CAP1 = 600;              // Set period value     [((1/150e6)*600)^-1 = 250 kHz]
    //ECap5Regs.CAP2 = 350;              // Set Compare (On-Time) value [50%] (Slightly "More" to account for Shoot-Through Protection [25 cycles each side])
    ECap5Regs.CTRPHS = 25;              // Minor Phase-Shift for Shoot-Through Protection (Shoot-Through Protection also handled in Hardware)
    //ECap5Regs.CTRPHS = 0;
    ECap5Regs.ECCTL2.bit.APWMPOL = 1;   // Active-Low Mode
    ECap5Regs.ECCTL2.bit.SYNCI_EN = 1;  // Enable Sync-In
    ECap5Regs.ECCTL2.bit.SYNCO_SEL = 2; // Disable Sync-Out Signal (End of Chain)
    ECap5Regs.ECCTL2.bit.TSCTRSTOP = 1; // Allow TSCTR to run (Free-Running)
    // End Resonant Converter APWM Setup

    // Start counters
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;
    ECap2Regs.ECCTL2.bit.TSCTRSTOP = 1;
    ECap4Regs.ECCTL2.bit.TSCTRSTOP = 1;
    ECap5Regs.ECCTL2.bit.TSCTRSTOP = 1;
}

void InitSPI()
{
    ////SPI (ADC0)Configuration
    EALLOW;
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;     // SPI software reset

    //SpiaRegs.SPIBRR = 0x007F;             // SPI Clock scaler
    SpiaRegs.SPIBRR = 0x0008;               //SPI Clock scaler (Approximate 3.5 MHz CLock) [219.27 kSPs or 27 kSps per channel]
    SpiaRegs.SPICCR.bit.SPICHAR = 0xF;      //SPI character length 16 bit
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;   //SPI (Master)
    SpiaRegs.SPICTL.bit.TALK = 1;           //SPI Enable Comm
    SpiaRegs.SPICCR.bit.SPILBK = 0;         //Disable loopback
    //SpiaRegs.SPIFFTX.bit.SPIFFENA    = 1;   //Enable SPI FIFO
    //SpiaRegs.SPIFFTX.bit.SPIRST      = 1;   //SPI FIFO can TX (or) RX
    //SpiaRegs.SPIFFTX.bit.TXFIFO      = 1;   //Release TX FIFO from reset
    //SpiaRegs.SPIFFTX.bit.TXFFIENA    = 1;   //Enable TX FIFO interrupt
    SpiaRegs.SPIFFTX.all=0xC028;      // Enable FIFO's, set TX FIFO level to 8
    //SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;   //Reset FIFO pointer to 0
    //SpiaRegs.SPIFFRX.bit.RXFFIENA    = 1;   //Enable RX FIFO interrupt
    //SpiaRegs.SPIFFRX.bit.RXFFIL    = 0x08;//Generate RX FIFO interrupt after receiving 8 words
    //SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;   //Enable RX FIFO operation
    SpiaRegs.SPIFFRX.all=0x0028;      // Set RX FIFO level to 8


    SpiaRegs.SPIFFCT.all=0x0002;    // Add Delay of 2 SPI Clk Cycles between transmitted words so CS Pin can release (at least a value of 1 is needed for SPISTE Pin to toggle)
    SpiaRegs.SPIPRI.all=0x0010;

    SpiaRegs.SPICCR.bit.CLKPOLARITY=0;
    SpiaRegs.SPICTL.bit.CLK_PHASE=0;
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;     //Release SPI software reset
    EDIS;

    // Begin SPI ADC Interrupt Routines
    SpiaRegs.SPIFFTX.bit.TXFIFO=1;
    SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;

    // Enable interrupts required for SPI
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER6.bit.INTx1=1;     // Enable PIE Group 6, INT 1
    PieCtrlRegs.PIEIER6.bit.INTx2=1;     // Enable PIE Group 6, INT 2
    IER |= 0x20;                            // Enable CPU INT6
    EINT;                                // Enable Global Interrupts
    ////End SPI (ADC0) Configuration
}

void InitMcBSP()
{
    //// McBSP-A SPI (ADC1) Configuration
    //Configure the McBSP-A as an SPI Master

    // Hold McBSP-A in reset when configuring/changing parameters
    McbspaRegs.SPCR2.bit.FRST=0; // Frame Sync generator reset
    McbspaRegs.SPCR2.bit.GRST=0; // Sample Rate generator Reset
    McbspaRegs.SPCR2.bit.XRST=0; // Transmitter reset
    McbspaRegs.SPCR1.bit.RRST=0; // Receiver reset
    ////McbspaRegs.SPCR1.bit.DLB = 1;       // Digital Loopback mode enabled
    McbspaRegs.SPCR1.bit.DLB = 0;       // Digital Loopback mode disabled
    //McbspaRegs.SPCR1.bit.RJUST = 0;     // Right justify the data and zero fill
    //McbspaRegs.SPCR1.bit.CLKSTP = 2;    // High inactive state without delay (From Table 12-15)
    McbspaRegs.SPCR1.bit.CLKSTP = 3;  // High inactive state with delay (From Table 12-15)
    McbspaRegs.PCR.bit.CLKXP = 1;       // (From Table 12-15)
    McbspaRegs.PCR.bit.CLKRP = 1;       // (From Table 12-15)
    McbspaRegs.PCR.bit.CLKXM = 1;       // MCLKX Pin is an output (From Table 12-16)
    McbspaRegs.PCR.bit.SCLKME = 0;      // Clock Generated by sample rate generator is derived from CPU Clock (From Table 12-16)
    McbspaRegs.SRGR2.bit.CLKSM = 1;     // From Table 12-16
    McbspaRegs.SRGR1.bit.CLKGDV = 15;   // Clock Divider; input clock of sample rate generator is divided by (CLKGDV+1) (From Table 12-16)
    //McbspaRegs.SRGR1.bit.CLKGDV = 0;   // Clock Divider; input clock of sample rate generator is divided by (CLKGDV+1) (From Table 12-16)
    McbspaRegs.PCR.bit.FSXM = 1;        // The FSX pin is an output pin driven according to the FSGM bit (From Table 12-16)
    McbspaRegs.SRGR2.bit.FSGM = 0;      // The transmitter drives a frame-synchronization pulse on the FSX pin every time data is transferred from DXR1 to XSR1. (From Table 12-16)
    McbspaRegs.PCR.bit.FSXP = 1;        // The FSX pin is active low. (From Table 12-16)

    McbspaRegs.RCR2.bit.RDATDLY= 1;     // FSX setup time 1 in master mode. 0 for slave mode (Receive) (From Table 12-16)
    McbspaRegs.XCR2.bit.XDATDLY= 1;     // FSX setup time 1 in master mode. 0 for slave mode (Transmit) (From Table 12-16)

    McbspaRegs.SPCR1.bit.DXENA = 0;     //DX delay enabler off

    McbspaRegs.RCR1.bit.RWDLEN1=2;      // 16-bit word
    McbspaRegs.XCR1.bit.XWDLEN1=2;      // 16-bit word
    McbspaRegs.RCR2.bit.RPHASE = 0;     // Single-phase frame
    McbspaRegs.XCR2.bit.XPHASE = 0;     // Single-phase frame
    McbspaRegs.RCR1.bit.RFRLEN1 = 0;    // 1 Word (Channel) per Frame
    McbspaRegs.XCR1.bit.XFRLEN1 = 0;    // 1 Word (Channel) per Frame

    // Enable McBSP-A Interrupts
    McbspaRegs.MFFINT.bit.XINT = 1; // Enable Transmit Interrupts
    McbspaRegs.MFFINT.bit.RINT = 1; // Enable Receive Interrupts

    // Enable McBSP-A (Remove from Reset Once Configured)
    McbspaRegs.SPCR2.bit.GRST=1;     //Enable Sample rate generator
    delay_loop();                   // Wait at least 2 SRG clock cycles
    McbspaRegs.SPCR2.bit.XRST=1;     //Enable TX
    McbspaRegs.SPCR1.bit.RRST=1;     //Enable RX
    McbspaRegs.SPCR2.bit.FRST=1;     //Frame Sync generator reset

    // Enable interrupts required for McBSP-A
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER6.bit.INTx5=1;     // Enable PIE Group 6, INT 5   (McBSP Port-A)
    PieCtrlRegs.PIEIER6.bit.INTx6=1;     // Enable PIE Group 6, INT 6   (McBSP Port-A)
    IER |= 0x20;                            // Enable CPU INT6
    EINT;                                // Enable Global Interrupts

    ////End McBSP-A SPI (ADC1) Configuration



    //// McBSP-B SPI (ADC2) Configuration
    //Configure the McBSP-B as an SPI Master

    // Hold McBSP-B in reset when configuring/changing parameters
    McbspbRegs.SPCR2.bit.FRST=0; // Frame Sync generator reset
    McbspbRegs.SPCR2.bit.GRST=0; // Sample Rate generator Reset
    McbspbRegs.SPCR2.bit.XRST=0; // Transmitter reset
    McbspbRegs.SPCR1.bit.RRST=0; // Receiver reset
    ////McbspbRegs.SPCR1.bit.DLB = 1;       // Digital Loopback mode enabled
    McbspbRegs.SPCR1.bit.DLB = 0;       // Digital Loopback mode disabled
    //McbspbRegs.SPCR1.bit.RJUST = 0;     // Right justify the data and zero fill
    //McbspbRegs.SPCR1.bit.CLKSTP = 2;    // High inactive state without delay (From Table 12-15)
    McbspbRegs.SPCR1.bit.CLKSTP = 3;  // High inactive state with delay (From Table 12-15)
    McbspbRegs.PCR.bit.CLKXP = 1;       // (From Table 12-15)
    McbspbRegs.PCR.bit.CLKRP = 1;       // (From Table 12-15)
    McbspbRegs.PCR.bit.CLKXM = 1;       // MCLKX Pin is an output (From Table 12-16)
    McbspbRegs.PCR.bit.SCLKME = 0;      // Clock Generated by sample rate generator is derived from CPU Clock (From Table 12-16)
    McbspbRegs.SRGR2.bit.CLKSM = 1;     // From Table 12-16
    //McbspbRegs.SRGR1.bit.CLKGDV = 15;   // Clock Divider; input clock of sample rate generator is divided by (CLKGDV+1) (From Table 12-16)
    McbspbRegs.SRGR1.bit.CLKGDV = 255;   // Clock Divider; input clock of sample rate generator is divided by (CLKGDV+1) (From Table 12-16)
    McbspbRegs.PCR.bit.FSXM = 1;        // The FSX pin is an output pin driven according to the FSGM bit (From Table 12-16)
    McbspbRegs.SRGR2.bit.FSGM = 0;      // The transmitter drives a frame-synchronization pulse on the FSX pin every time data is transferred from DXR1 to XSR1. (From Table 12-16)
    McbspbRegs.PCR.bit.FSXP = 1;        // The FSX pin is active low. (From Table 12-16)

    McbspbRegs.RCR2.bit.RDATDLY= 1;     // FSX setup time 1 in master mode. 0 for slave mode (Receive) (From Table 12-16)
    McbspbRegs.XCR2.bit.XDATDLY= 1;     // FSX setup time 1 in master mode. 0 for slave mode (Transmit) (From Table 12-16)

    McbspbRegs.SPCR1.bit.DXENA = 0;     //DX delay enabler off

    McbspbRegs.RCR1.bit.RWDLEN1=2;      // 16-bit word
    McbspbRegs.XCR1.bit.XWDLEN1=2;      // 16-bit word
    McbspbRegs.RCR2.bit.RPHASE = 0;     // Single-phase frame
    McbspbRegs.XCR2.bit.XPHASE = 0;     // Single-phase frame
    McbspbRegs.RCR1.bit.RFRLEN1 = 0;    // 1 Word (Channel) per Frame
    McbspbRegs.XCR1.bit.XFRLEN1 = 0;    // 1 Word (Channel) per Frame

    // Enable McBSP-B Interrupts
    McbspbRegs.MFFINT.bit.XINT = 1; // Enable Transmit Interrupts
    McbspbRegs.MFFINT.bit.RINT = 1; // Enable Receive Interrupts

    // Enable McBSP-B (Remove from Reset Once Configured)
    McbspbRegs.SPCR2.bit.GRST=1;     //Enable Sample rate generator
    delay_loop();                   // Wait at least 2 SRG clock cycles
    McbspbRegs.SPCR2.bit.XRST=1;     //Enable TX
    McbspbRegs.SPCR1.bit.RRST=1;     //Enable RX
    McbspbRegs.SPCR2.bit.FRST=1;     //Frame Sync generator reset

    // Enable interrupts required for McBSP-B
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER6.bit.INTx3=1;     // Enable PIE Group 6, INT 3   (McBSP Port-B)
    PieCtrlRegs.PIEIER6.bit.INTx4=1;     // Enable PIE Group 6, INT 4   (McBSP Port-B)
    IER |= 0x20;                            // Enable CPU INT6
    EINT;                                // Enable Global Interrupts

    ////End McBSP-B SPI (ADC2) Configuration
}

