/*******************************************************************************
 This code is writtn by Mr Arash yousefi at 2025.05.01
 # AC Wallbox EV MCU Code
 # Accourding to IEC 61851-1 
 # This code controles Eastron Meter	
*******************************************************************************/
/* includes */
#include "Meter_Eastron.h"
#include "string.h"

/* variables */
/******************************************************************************
@brief  This is function is for sends an amount of data in DMA mode.
@param      pData Pointer to data buffer (u8 or u16 data elements).
@param      Size  Amount of data elements (u8 or u16) to be sent
@retval   STATUS_OK=0
******************************************************************************/
void eastron_init(void)
{

}

/******************************************************************************
@brief  This is function is for sends an amount of data in DMA mode.
@param      pData Pointer to data buffer (u8 or u16 data elements).
@param      Size  Amount of data elements (u8 or u16) to be sent
@retval   STATUS_OK=0
******************************************************************************/
void eastron_sdm54_read(eastron_sdm54_t *eastron,sdm54_registers_t modbus_registers)
{
    uint32_t temp_u32 = 0;
    
    switch ( modbus_registers )
    {
        case sdm54_Voltage_1: 
        {
            master_read_input_register(&eastron->METER_SLAVE, eastron->address, modbus_registers, 2);
            temp_u32 = ( (eastron->memory.inputRegister[modbus_registers] << 16) | (eastron->memory.inputRegister[modbus_registers + 1]) );
            memcpy(&eastron->modbus_param.Voltage1,&temp_u32,4);  
        }break;   
        case sdm54_Voltage_2: 
        {
            master_read_input_register(&eastron->METER_SLAVE, eastron->address, modbus_registers, 2);
            temp_u32 = ( (eastron->memory.inputRegister[modbus_registers] << 16) | (eastron->memory.inputRegister[modbus_registers + 1]) );
            memcpy(&eastron->modbus_param.Voltage2,&temp_u32,4);  
        }break;
        case sdm54_Voltage_3: 
        {
            master_read_input_register(&eastron->METER_SLAVE, eastron->address, modbus_registers, 2);
            temp_u32 = ( (eastron->memory.inputRegister[modbus_registers] << 16) | (eastron->memory.inputRegister[modbus_registers + 1]) );
            memcpy(&eastron->modbus_param.Voltage3,&temp_u32,4);  
        }break;        
        case sdm54_Current_1:      
        {
            master_read_input_register(&eastron->METER_SLAVE, eastron->address, modbus_registers, 2);
            temp_u32 = ( (eastron->memory.inputRegister[modbus_registers] << 16) | (eastron->memory.inputRegister[modbus_registers + 1]) );
            memcpy(&eastron->modbus_param.Current1,&temp_u32,4);  
        }break;  
        case sdm54_Current_2:      
        {
            master_read_input_register(&eastron->METER_SLAVE, eastron->address, modbus_registers, 2);
            temp_u32 = ( (eastron->memory.inputRegister[modbus_registers] << 16) | (eastron->memory.inputRegister[modbus_registers + 1]) );
            memcpy(&eastron->modbus_param.Current2,&temp_u32,4);  
        }break;
        case sdm54_Current_3:      
        {
            master_read_input_register(&eastron->METER_SLAVE, eastron->address, modbus_registers, 2);
            temp_u32 = ( (eastron->memory.inputRegister[modbus_registers] << 16) | (eastron->memory.inputRegister[modbus_registers + 1]) );
            memcpy(&eastron->modbus_param.Current3,&temp_u32,4);  
        }break;        
        case sdm54_power_1:	
        {
            master_read_input_register(&eastron->METER_SLAVE, eastron->address, modbus_registers, 2);
            temp_u32 = ( (eastron->memory.inputRegister[modbus_registers] << 16) | (eastron->memory.inputRegister[modbus_registers + 1]) );
            memcpy(&eastron->modbus_param.Active_power1,&temp_u32,4);  
        }break;   
        case sdm54_power_2:	
        {
            master_read_input_register(&eastron->METER_SLAVE, eastron->address, modbus_registers, 2);
            temp_u32 = ( (eastron->memory.inputRegister[modbus_registers] << 16) | (eastron->memory.inputRegister[modbus_registers + 1]) );
            memcpy(&eastron->modbus_param.Active_power2,&temp_u32,4);  
        }break;
        case sdm54_power_3:	
        {
            master_read_input_register(&eastron->METER_SLAVE, eastron->address, modbus_registers, 2);
            temp_u32 = ( (eastron->memory.inputRegister[modbus_registers] << 16) | (eastron->memory.inputRegister[modbus_registers + 1]) );
            memcpy(&eastron->modbus_param.Active_power3,&temp_u32,4);  
        }break;                  
        case sdm54_Export_active_energy: 
        {
            master_read_input_register(&eastron->METER_SLAVE, eastron->address, modbus_registers, 2);
            temp_u32 = ( (eastron->memory.inputRegister[modbus_registers] << 16) | (eastron->memory.inputRegister[modbus_registers + 1]) );
            memcpy(&eastron->modbus_param.Export_active_energy,&temp_u32,4);  
        }break;                       
    }    
}

/******************************************************************************
@brief  This is function is for sends an amount of data in DMA mode.
@param      pData Pointer to data buffer (u8 or u16 data elements).
@param      Size  Amount of data elements (u8 or u16) to be sent
@retval   STATUS_OK=0
******************************************************************************/
void eastron_sdm120_read(eastron_sdm120_t *eastron,sdm120_registers_t modbus_registers)
{
    uint32_t temp_u32 = 0;
    
    switch ( modbus_registers )
    {
        case sdm120_Voltage: 
        {
            master_read_input_register(&eastron->METER_SLAVE, eastron->address, modbus_registers, 2);
            temp_u32 = ( (eastron->memory.inputRegister[modbus_registers] << 16) | (eastron->memory.inputRegister[modbus_registers + 1]) );
            memcpy(&eastron->modbus_param.Voltage,&temp_u32,4);  
        }break;            
        case sdm120_Current:      
        {
            master_read_input_register(&eastron->METER_SLAVE, eastron->address, modbus_registers, 2);
            temp_u32 = ( (eastron->memory.inputRegister[modbus_registers] << 16) | (eastron->memory.inputRegister[modbus_registers + 1]) );
            memcpy(&eastron->modbus_param.Current,&temp_u32,4);  
        }break;            
        case sdm120_Active_power:	
        {
            master_read_input_register(&eastron->METER_SLAVE, eastron->address, modbus_registers, 2);
            temp_u32 = ( (eastron->memory.inputRegister[modbus_registers] << 16) | (eastron->memory.inputRegister[modbus_registers + 1]) );
            memcpy(&eastron->modbus_param.Active_power,&temp_u32,4);  
        }break;                        
        case sdm120_Export_active_energy: 
        {
            master_read_input_register(&eastron->METER_SLAVE, eastron->address, modbus_registers, 2);
            temp_u32 = ( (eastron->memory.inputRegister[modbus_registers] << 16) | (eastron->memory.inputRegister[modbus_registers + 1]) );
            memcpy(&eastron->modbus_param.Export_active_energy,&temp_u32,4);  
        }break;                       
    }    
}
