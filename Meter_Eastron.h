/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __METER_EASTRON_H
#define __METER_EASTRON_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "ModBus-Master.h"

/* Meter settings */

/* Meter settings */
#define	EASTRON_ADRRESS_TYPE2_1			                    1
#define	EASTRON_ADRRESS_TYPE2_2			                    2
#define	EASTRON_ADRRESS_SCHOKO_1			                3
#define	EASTRON_ADRRESS_SCHOKO_2			                4

#define	MASTER_EASTRON_MAX_SLAVES							4

/*  ---------------------------- sdm54 3ph------------------------------ */
typedef enum
{
    sdm54_Voltage_1           					=0x00,
    sdm54_Voltage_2           					=0x02,
    sdm54_Voltage_3           					=0x04,
    
    sdm54_Current_1          			        =0x06,
    sdm54_Current_2          			        =0x08,  
    sdm54_Current_3          			        =0x0a,  
    
    sdm54_power_1							    =0x0C,
    sdm54_power_2							    =0x0E,
    sdm54_power_3							    =0x10,
    
    sdm54_Export_active_energy              	=0x4A,  /*  74  */
}sdm54_registers_t;

typedef struct
{
    float 	Voltage1;
    float 	Voltage2;
    float 	Voltage3;
    float 	Current1;
    float 	Current2;
    float 	Current3;
    float 	Active_power1;
    float 	Active_power2;
    float 	Active_power3;
	float	Export_active_energy;
}sdm54_params_t;

typedef struct{
    uint16_t 		coil[5];
    uint16_t 		discreteInput[5];
    uint16_t 		inputRegister[100];     /* last address is 88 */ 
    uint16_t 	    holdingRegister[100];   /* last address is 86 */ 
}sdm54_memory_t;

typedef struct
{
    uint16_t            address;
    sdm54_memory_t 	    memory;
    sdm54_params_t      modbus_param;
    MODBUS_SLAVES_t	    METER_SLAVE;
}eastron_sdm54_t;

/*  ---------------------------- sdm120 1ph------------------------------ */
typedef enum
{
    sdm120_Voltage           					=0x00,      
    sdm120_Current          			        =0x06,  
    sdm120_Active_power							=0x0C,

    sdm120_Export_active_energy              	=0x4A,  /* 74 */
}sdm120_registers_t;

typedef struct
{
    float 	Voltage;
    float 	Current;
    float 	Active_power;
	float	Export_active_energy;
}sdm120_params_t;

typedef struct{
    uint16_t 		coil[5];
    uint16_t 		discreteInput[5];
    uint16_t 		inputRegister[255]; /* last address is 254 */ 
    uint16_t 	    holdingRegister[5];
}sdm120_memory_t;

typedef struct
{
    uint16_t            address;
    sdm54_memory_t 	    memory;
    sdm120_params_t     modbus_param;
    MODBUS_SLAVES_t	    METER_SLAVE;
}eastron_sdm120_t;

//void eastron_init(void);
void eastron_sdm54_read(eastron_sdm54_t *eastron,sdm54_registers_t modbus_registers);
void eastron_sdm120_read(eastron_sdm120_t *eastron,sdm120_registers_t modbus_registers);
#ifdef __cplusplus
}
#endif

#endif /* __METER_EASTRON_H */
