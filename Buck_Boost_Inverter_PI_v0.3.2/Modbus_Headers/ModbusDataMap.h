/*
 * ModbusVarMap.h
 *
 *  Created on: 08/10/2014
 *      Author: bds
 */

#ifndef MODBUSVARMAP_H_
#define MODBUSVARMAP_H_

#include "ModbusSettings.h"
#include "Solar_F.h"    //Included for Uint8 declaration to prevent conflict in "DataTypes.h" redeclaration.

#if MB_COILS_ENABLED
typedef struct ModbusCoilsMap ModbusCoilsMap;
struct ModbusCoilsMap{
	Uint8 dummy0;
	Uint8 dummy1;
//	Uint8 dummy2;
//	Uint8 dummy3;
//	Uint8 dummy4;
//	Uint8 dummy5;
//	Uint8 dummy6;
//	Uint8 dummy7;
//	Uint8 dummy8;
//	Uint8 dummy9;
//	Uint8 dummy10;
//	Uint8 dummy11;
//	Uint8 dummy12;
//	Uint8 dummy13;
//	Uint8 dummy14;
//	Uint8 dummy15;
//	Uint8 dummy16;
//	Uint8 dummy17;
//	Uint8 dummy18;
};

ModbusCoilsMap construct_ModbusCoilsMap();
#endif

#if MB_INPUTS_ENABLED
typedef struct ModbusInputsMap ModbusInputsMap;
struct ModbusInputsMap{
	Uint8 dummy0;
	Uint8 dummy1;
	Uint8 dummy2;
	Uint8 dummy3;
	Uint8 dummy4;
	Uint8 dummy5;
	Uint8 dummy6;
	Uint8 dummy7;
	Uint8 dummy8;
	Uint8 dummy9;
	Uint8 dummy10;
	Uint8 dummy11;
	Uint8 dummy12;
	Uint8 dummy13;
	Uint8 dummy14;
	Uint8 dummy15;
	Uint8 dummy16;
	Uint8 dummy17;
	Uint8 dummy18;
};

ModbusInputsMap construct_ModbusInputsMap();
#endif

#if MB_HOLDING_REGISTERS_ENABLED
typedef struct ModbusHoldingRegistersMap ModbusHoldingRegistersMap;
struct ModbusHoldingRegistersMap {

    /*
	int16 dummy0;
	int16 dummy1;
	int16 dummy2;
	int16 dummy3;
	int16 dummy4;
	int16 dummy5;
	int16 dummy6;
	int16 dummy7;
	int16 dummy8;
	int16 dummy9;
	int16 dummy10;
	int16 dummy11;
	int16 dummy12;
	int16 dummy13;
	int16 dummy14;
	*/
	Uint16 Mod_Reg[2048];
	//Uint16 Mod_Reg_Ch1[96];
};

ModbusHoldingRegistersMap construct_ModbusHoldingRegistersMap();
#endif

#if MB_INPUT_REGISTERS_ENABLED
typedef struct ModbusInputRegistersMap ModbusInputRegistersMap;
struct ModbusInputRegistersMap {
	//float32 dummy0;
	//float32 dummy1;
	//float32 dummy2;
	//float32 dummy3;
	//float32 dummy4;
    Uint16 Mod_Reg2[96];
};

ModbusInputRegistersMap construct_ModbusInputRegistersMap();
#endif

#endif /* MODBUSVARMAP_H_ */
