/*******************************************************************************
* @file  ModBus.c
* @author  EVIDC
* @brief   ModBus Source file.

  This Source is a Source file for using in Modbus protocol. 
********************************************************************************
* @attention
* MAPNA Electric & Control Engineering & Manufacturing Co. 
  MECO. All right reserved.
*
*******************************************************************************/
  
/*Inclusion of system headers*/
#include <stdio.h>    
#include "main.h"  
/*Inclusion of user defined headers*/
#include "ModBus-Master.h"

//----------------------- Function codes -----------------------//
#define MODBUS_READ_SINGLE_COIL                         0x01 
#define MODBUS_READ_DISCRETE_INPUT                      0x02 
#define MODBUS_READ_HOLDING_REGISTERS                   0x03 
#define MODBUS_READ_INPUT_REGISTER                      0x04 
#define MODBUS_WRITE_SINGLE_COIL                        0x05 
#define MODBUS_WRITE_SINGLE_REGISTER                    0x06 
#define MODBUS_WRITE_MULTIPLE_COIL                      0x0F 
#define MODBUS_WRITE_MULTIPLE_REGISTER                  0x10 

#define RESULT_OK                                       0x00 
#define RESULT_ERROR                                    0xFF 

/* 	Master general setting
	configure response time                   	<-tx---------> tx 
  ||||||______________||||||______________||||||______________||||||
*/
#define SLAVE_RESPONSE_TIME                             100          	// ms , calculate max response time 
#define TIME_OUT_VALUE                                  1000         	// ms
#define MASTER_MAX_RETRY                                1           	// MASTER_MAX_RETRY < (TIME_OUT_VALUE/SLAVE_RESPONSE_TIME) +1

/*public variable definition*/
MODBUS_PORT_t		 	*MODBUS_PORT;

TIM_HandleTypeDef  		*htim_DEV;
			   
uint32_t           		reponse_time_value = (float)3.3*SLAVE_RESPONSE_TIME;
uint32_t           		timeout_value_port = (float)3.3*TIME_OUT_VALUE;                // 1/303us = 3.3ms;

/* public function body */
uint8_t                 MODBUS_master_read(MODBUS_SLAVES_t *slave);
void                    MASTER_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size); 	
void                    MASTER_TxCpltCallback(UART_HandleTypeDef *huart);		
	
FIFO_error_status       FIFO_put(MODBUS_SLAVES_t *slave);
FIFO_error_status       FIFO_get(FIFO_t *buffer,command_t *command);
void                    TIM_MASTER_PeriodElapsedCallback(TIM_HandleTypeDef *htim);	
uint16_t                master_ModRTU_CRC(uint8_t buf[], int len);

/* functions to read slave data from memory */
void                    master_set_data_single_coil(MODBUS_SLAVES_t *slave,uint16_t input_start_address,uint16_t input_byte,uint8_t *input_data);
void                    master_set_write_single_coil(MODBUS_SLAVES_t *slave,uint16_t input_start_address,uint16_t input_byte,uint8_t *input_data);
void                    master_set_data_discrete_input(MODBUS_SLAVES_t *slave,uint16_t input_start_address,uint16_t input_byte,uint8_t *input_data);
void                    master_set_data_input_register(MODBUS_SLAVES_t *slave,uint16_t input_start_address,uint8_t byte_count,uint8_t *input_data);
void                    master_set_data_holding_register(MODBUS_SLAVES_t *slave,uint16_t input_start_address,uint8_t byte_count,uint8_t *input_data);
uint16_t                master_get_data_discrete_input(MODBUS_SLAVES_t *slave,uint16_t input_start_address);
uint16_t                master_get_data_input_register(MODBUS_SLAVES_t *slave,uint16_t input_start_address);
uint16_t                master_get_data_holding_register(MODBUS_SLAVES_t *slave,uint16_t input_start_address);
void                    master_set_data_multiple_register(MODBUS_SLAVES_t *slave,uint16_t input_start_address,uint8_t byte_count,uint16_t input_data[]);

/******************************************************************************
@brief  This is function is to calculate CRC for Mod bus 
@param  this function get buffer and its length and give CRC value
@retval calculated CRC value 
******************************************************************************/
uint16_t master_ModRTU_CRC(uint8_t buf[], int len)
{
    uint16_t crc = 0xFFFF;

    for (int pos = 0; pos < len; pos++)
    {
      crc ^= (uint16_t)buf[pos];

        for (int i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                    crc >>= 1;
                    crc ^= 0xA001;
            }
            else
            {
                    crc >>= 1;
            }
        }
    }
    return crc;
}

/******************************************************************************
@brief  function to put a command to FIFO 
@param  input slave parameters        
@retval error if FIFO was full  
******************************************************************************/
FIFO_error_status FIFO_put(MODBUS_SLAVES_t *slave)
{
    if(slave->master_FIFO.count==FIFO_BUFFER_SIZE) return FIFO_ERROR;
    slave->master_FIFO.command_data[slave->master_FIFO.in++] = slave->put_command;
    slave->master_FIFO.count++;
    if(slave->master_FIFO.in==FIFO_BUFFER_SIZE) slave->master_FIFO.in=0;    // start from beginning 
    return FIFO_SUCCESS;
}

/******************************************************************************
@brief  function to get a command from FIFO 
@param  ports buffer  
@param  command   
@retval error if FIFO was full  
******************************************************************************/
FIFO_error_status FIFO_get(FIFO_t *buffer,command_t *command)
{
    if(buffer->count==0) return FIFO_ERROR;
    *command = buffer->command_data[buffer->out++];
    buffer->count--;
    if(buffer->out==FIFO_BUFFER_SIZE) buffer->out=0;    // start from beginning 
    return FIFO_SUCCESS;
}

/******************************************************************************
@brief  This is function is for polling between slaves
@param  modbus port
@retval   no
******************************************************************************/
void        master_polling(MODBUS_PORT_t *port)
{ 
    static uint8_t slave_count=0;  
    switch ( port->MODBUS_SLAVES[slave_count]->polling_state)
    {
        case get_FIFO:
        {
            if(FIFO_get(&port->MODBUS_SLAVES[slave_count]->master_FIFO,&port->MODBUS_SLAVES[slave_count]->get_command ) == FIFO_SUCCESS)                                                                                                    // if data exists in FIFO
            {
                port->MODBUS_SLAVES[slave_count]->slave_addr = port->MODBUS_SLAVES[slave_count]->get_command.data[0];
                port->MODBUS_SLAVES[slave_count]->start_address_memory = (port->MODBUS_SLAVES[slave_count]->get_command.data[2] << 8) | port->MODBUS_SLAVES[slave_count]->get_command.data[3]; 
                port->MODBUS_SLAVES[slave_count]->data_quantity =  port->MODBUS_SLAVES[slave_count]->get_command.data[5];  
                port->MODBUS_SLAVES[slave_count]->master_retry_send =0;
                port->MODBUS_SLAVES[slave_count]->timeout_counter=0;
                MODBUS_PORT->setting.UART.rec_data_size=0;
                port->MODBUS_SLAVES[slave_count]->polling_state = send_command;
            }
            else
            {
                slave_count++;
                if( slave_count >= port->slaves_numbers )
                {
                   slave_count=0; 
                }
            }
            break;
        }
        case send_command:
        {
            if( port->MODBUS_SLAVES[slave_count]->master_retry_send < MASTER_MAX_RETRY)
            {
                port->MODBUS_SLAVES[slave_count]->master_retry_send++;
                /* UART RTS */
                if( (MODBUS_PORT->setting.UART.RTS.Port != NULL) && (MODBUS_PORT->setting.UART.RTS.Pin != NULL) )
                {
                    HAL_GPIO_WritePin(MODBUS_PORT->setting.UART.RTS.Port,MODBUS_PORT->setting.UART.RTS.Pin,GPIO_PIN_SET);
                }	
                HAL_UART_Transmit_DMA(MODBUS_PORT->setting.UART.huart,&port->MODBUS_SLAVES[slave_count]->get_command.data[0],port->MODBUS_SLAVES[slave_count]->get_command.command_size);                    
                port->MODBUS_SLAVES[slave_count]->polling_state = wait_response;
                port->MODBUS_SLAVES[slave_count]->response_counter=0;
            }
            else
            {
                /* set  error  */
                port->MODBUS_SLAVES[slave_count]->slave_error = 1;     
                if( (MODBUS_PORT->setting.UART.RTS.Port != NULL ) && (MODBUS_PORT->setting.UART.RTS.Pin != NULL ) )
                {
                    HAL_GPIO_WritePin(MODBUS_PORT->setting.UART.RTS.Port,MODBUS_PORT->setting.UART.RTS.Pin,GPIO_PIN_RESET);
                }
                port->MODBUS_SLAVES[slave_count]->polling_state = get_FIFO; 
                MASTER_RxCpltCallback(MODBUS_PORT->setting.UART.huart,0);         // enable again in disconnection    
            }
             MODBUS_PORT->setting.UART.rec_data_size=0;        
            break;
        }            
        case wait_response:
        {
            if( port->MODBUS_SLAVES[slave_count]->timeout_counter < timeout_value_port)
            {
                port->MODBUS_SLAVES[slave_count]->timeout_counter++;                                                    // count up to time out
                if( port->MODBUS_SLAVES[slave_count]->response_counter < reponse_time_value)    // count up to response
                {
                    port->MODBUS_SLAVES[slave_count]->response_counter++;            
                }
                else
                {
                    if(MODBUS_master_read(port->MODBUS_SLAVES[slave_count]) == RESULT_OK) // check if respone is for correct slave id
                    {
                        port->MODBUS_SLAVES[slave_count]->polling_state = get_FIFO;
                        port->MODBUS_SLAVES[slave_count]->slave_error = 0;                                                                                        // clear bit error : every bit has its slave ID error (max 32 slaves)                    
                        MODBUS_PORT->setting.UART.rec_data_size=0;                     
                    }
                    else
                    {                            
                        port->MODBUS_SLAVES[slave_count]->polling_state = send_command;    
                    }                                
                    port->MODBUS_SLAVES[slave_count]->response_counter=0;
                }
            }
            else
            {
                port->MODBUS_SLAVES[slave_count]->polling_state = get_FIFO;    
            }                
            break;
        }            
    }      
}

/******************************************************************************
@brief  This is function is to read value from slave
@param  slave's parameters
@retval error 
******************************************************************************/
uint8_t     MODBUS_master_read(MODBUS_SLAVES_t *slave)
{
    uint16_t        master_CRC_CALC			=0;
	uint16_t        master_CRC_REC			=0;
    uint8_t         return_value			=0;
    //uint8_t         input_slave_address     =  MODBUS_PORT->setting.UART.rec_data_value[0];
    uint8_t         function_code           =  MODBUS_PORT->setting.UART.rec_data_value[1];
    uint8_t         byte_count              =  MODBUS_PORT->setting.UART.rec_data_value[2];
    master_CRC_CALC                         =  master_ModRTU_CRC( MODBUS_PORT->setting.UART.rec_data_value,(slave->get_command.response_size -2));
    master_CRC_REC                          =  MODBUS_PORT->setting.UART.rec_data_value[(slave->get_command.response_size -2)] + ( MODBUS_PORT->setting.UART.rec_data_value[(slave->get_command.response_size -1)]<<8);    

    if( MODBUS_PORT->setting.UART.rec_data_value[0] == slave->slave_addr)
    {
        if(master_CRC_REC     == master_CRC_CALC)
        {         
                switch (function_code) 
                {
                    //Function 01 (01hex) Read Coils (read/write enable)
                    case MODBUS_READ_SINGLE_COIL:            // read all from 0
                    {
                        master_set_data_single_coil(slave,slave->start_address_memory,byte_count,&MODBUS_PORT->setting.UART.rec_data_value[3]);    
                        break;
                    }
                    //Function 02(02hex) Read Discrete Inputs (read only)    
                    case MODBUS_READ_DISCRETE_INPUT:        // read all from 0
                    {
                        master_set_data_discrete_input(slave,slave->start_address_memory,byte_count,&MODBUS_PORT->setting.UART.rec_data_value[3]);                        
                        break;
                    }
                    //Function 03 (03hex) Read Holding Registers (read/write enable)
                    case MODBUS_READ_HOLDING_REGISTERS:     // read all from 0
                    {        
                        master_set_data_holding_register(slave,slave->start_address_memory,byte_count,&MODBUS_PORT->setting.UART.rec_data_value[3]);                
                        break;
                    }
                    //Function 04 (04hex) Read Input Registers (read only)    
                    case MODBUS_READ_INPUT_REGISTER:        // read all from 0
                    {            
                        master_set_data_input_register(slave,slave->start_address_memory,byte_count,&MODBUS_PORT->setting.UART.rec_data_value[3]);                                    
                        break;
                    }    
                    //Function 05 (04hex) write single Registers    
                    case MODBUS_WRITE_SINGLE_COIL:          //
                    {  
                        master_set_write_single_coil(slave,slave->start_address_memory,1,&MODBUS_PORT->setting.UART.rec_data_value[4]);          
                        break;
                    }    
                    //Function 06 (04hex) write single Registers    
                    case MODBUS_WRITE_SINGLE_REGISTER:      // read all from 0
                    {
                        master_set_data_holding_register(slave,slave->start_address_memory,1,&MODBUS_PORT->setting.UART.rec_data_value[4]);    
                        break;
                    }                            
                    //Function 0F (04hex) Read Input Registers (read only)    
                    case MODBUS_WRITE_MULTIPLE_COIL:        // read all from 0
                    { 
                        byte_count              =  ((slave->put_command.data[4] << 8 | slave->put_command.data[5]) -1)/16 +1;
                        master_set_data_single_coil(slave,slave->start_address_memory,byte_count,&slave->put_command.data[7]);   
                        break;
                    }                            
                    //Function 10 (04hex) Read Input Registers (read only)    
                    case MODBUS_WRITE_MULTIPLE_REGISTER:    // read all from 0
                    {
                        byte_count              =  2*(slave->put_command.data[4] << 8 | slave->put_command.data[5]);
                        master_set_data_holding_register(slave,slave->start_address_memory,byte_count,&slave->put_command.data[7]);    
                        break;
                    }                        
                }
                return_value = RESULT_OK;
        }
        else
        {
            return_value = RESULT_ERROR;
        }
    }
    else
    {
        return_value = RESULT_ERROR;
    }
    /* clear all data */
    for(int i = 0; i< MODBUS_PORT->MODBUS_SLAVES[i]->get_command.response_size; i++)
    {
         MODBUS_PORT->setting.UART.rec_data_value[i] =0;
    } 
    return return_value;
}    

/******************************************************************************
@brief  This is function is to set coil's value from read modbus to internal memory
The requested ON / OFF states are specified by contents of the query data field. A logical 1 in a bit position of the field requests the corresponding coils to be ON. A logical 0 requests it to be OFF. 
Below is an example of a request to write a series of ten coils starting at coil 20 (addressed as 19, or 13 hex) in slave device 17. 
The query data contents are two bytes: CD 01 hex (1100 1101 0000 0001 binary). The binary bits correspond to the coils in the following way: 

Bit:  1  1  0  0  1  1  0  1  0 0 0 0 0 0 0  1 
Coil: 27 26 25 24 23 22 21 20 - - - - - - 29 28 
@param  slaves's parameter
@param  start address
@param  number of bytes
@param  data
@retval no
******************************************************************************/
void     master_set_data_single_coil(MODBUS_SLAVES_t *slave ,uint16_t input_start_address,uint16_t input_byte,uint8_t *input_data)
{
	int byte_address     		= 0;
	int bit_address          	= 0;
    uint16_t input_quantity     = slave->data_quantity;
    uint16_t size_of_coil = slave->memory_size.coil;//*(&slave->memory.coil + 1) - slave->memory.coil;
	if( input_byte <= size_of_coil )
	{
        for (int i=0; i < input_quantity; i++)
        { 
            byte_address     		= ((input_start_address + i )/16);
            bit_address          	= ((input_start_address + i )%16) ;     
            if( ((input_data[i/8]  >> (i & 0x07) ) & 0x01) == 0 )
            {
                slave->memory.coil[byte_address] = slave->memory.coil[byte_address] & ~( 0x0001 << ( bit_address ) );
            }
            else
            {
                slave->memory.coil[byte_address] = slave->memory.coil[byte_address] | ( 0x0001 << ( bit_address ) );
            }    
        } 
	}
}

/******************************************************************************
@brief  This is function is to set coil's value from write modbus to internal memory
@param  slaves's parameter
@param  start address
@param  number of bytes
@param  data
@retval no
******************************************************************************/
void    master_set_write_single_coil(MODBUS_SLAVES_t *slave,uint16_t input_start_address,uint16_t input_byte,uint8_t *input_data)
{
    int byte_address     		= ((input_start_address)/16);
    int bit_address          	= ((input_start_address)%16) ;     
    if( input_data[0] == 0xff )
    {
        slave->memory.coil[byte_address] = slave->memory.coil[byte_address] | ( 0x0001 << ( bit_address ) );
    }
    else if( input_data[0] == 0x00 )
    {
        slave->memory.coil[byte_address] = slave->memory.coil[byte_address] & ~( 0x0001 << ( bit_address ) );
    }    
}
/******************************************************************************
@brief  This is function is to set discrete's value from read modbus to internal memory
@param  slaves's parameter
@param  start address
@param  number of bytes
@param  data
@retval no
******************************************************************************/
void     master_set_data_discrete_input(MODBUS_SLAVES_t *slave,uint16_t input_start_address,uint16_t input_byte,uint8_t *input_data)
{
	int byte_address     		= 0;
	int bit_address          	= 0;
    uint16_t input_quantity     = slave->data_quantity;
    uint16_t size_of_discreteInput = slave->memory_size.discreteInput;//*(&slave->memory.discreteInput + 1) - slave->memory.discreteInput;
	if( input_byte <= size_of_discreteInput )
	{
        for (int i=0; i < input_quantity; i++)
        { 
            byte_address     		= ((input_start_address + i )/16);
            bit_address          	= ((input_start_address + i )%16) ;     
            if( ((input_data[i/8]  >> (i & 0x07) ) & 0x01) == 0 )
            {
                slave->memory.discreteInput[byte_address] = slave->memory.discreteInput[byte_address] & ~( 0x0001 << ( bit_address ) );
            }
            else
            {
                slave->memory.discreteInput[byte_address] = slave->memory.discreteInput[byte_address] | ( 0x0001 << ( bit_address ) );
            }    
        } 
	}
}
/******************************************************************************
@brief  This is function is to set discrete's value from read modbus to internal memory
@param  slaves's parameter
@param  start address
@retval no
******************************************************************************/
uint16_t   master_get_data_discrete_input(MODBUS_SLAVES_t *slave,uint16_t input_start_address)
{
    int byte_address        	= (input_start_address/16);
    int bit_address         	= (input_start_address%16);
    int	start_addr_offset   	= byte_address;
    uint16_t return_value;   
    return_value = (slave->memory.discreteInput[start_addr_offset] >> (bit_address)) & 0x01;
    return return_value;
}
/******************************************************************************
@brief  This is function is to set registers's value from read modbus to internal memory
@param  slaves's parameter
@param  start address
@param  number of bytes
@param  data
@retval no
******************************************************************************/
void        master_set_data_input_register(MODBUS_SLAVES_t *slave,uint16_t input_start_address,uint8_t byte_count,uint8_t *input_data)
{
    uint16_t temp_mem = 0;
    uint16_t size_of_inputRegister = slave->memory_size.inputRegister;
	if( byte_count <= size_of_inputRegister )
	{
        for(int i=0; i<byte_count; i+=2)
        {
            temp_mem = ((input_data[i] << 8) & 0xFF00 ) | ((input_data[i+1]) & 0x00FF);
            slave->memory.inputRegister[input_start_address + i/2] = temp_mem;        
        } 
    }   
}

/******************************************************************************
@brief  This is function is to set multiple registers's value from read modbus to internal memory
@param  slaves's parameter
@param  start address
@param  number of bytes
@param  data
@retval no
******************************************************************************/
void        master_set_data_multiple_register(MODBUS_SLAVES_t *slave,uint16_t input_start_address,uint8_t byte_count,uint16_t input_data[])
{
    uint16_t temp_mem = 0;
    uint16_t size_of_holdingRegister = slave->memory_size.holdingRegister;//*(&slave->memory.holdingRegister + 1) - slave->memory.holdingRegister;
	if( byte_count <= size_of_holdingRegister )
	{
        for(int i=0; i<byte_count; i++)
        {
            temp_mem = ((input_data[i] >> 8) & 0x00FF ) | ((input_data[i] << 8) & 0xFF00);
            slave->memory.inputRegister[input_start_address + i] = temp_mem;        
        }  
    }   
}

/******************************************************************************
@brief  This is function is to get registers's value 
@param  slaves's parameter
@param  start address
@retval no
******************************************************************************/
uint16_t    master_get_data_input_register(MODBUS_SLAVES_t *slave,uint16_t input_start_address)
{
    uint16_t return_value=0;
    return_value = slave->memory.inputRegister[input_start_address];
    return return_value;
}

/******************************************************************************
@brief  This is function is to get registers's value 
@param  slaves's parameter
@param  start address
@retval no
******************************************************************************/
uint16_t    master_get_data_holding_register(MODBUS_SLAVES_t *slave,uint16_t input_start_address)
{
    uint16_t return_value=0;
    return_value = slave->memory.holdingRegister[input_start_address];
    return return_value;
}

/******************************************************************************
@brief  This is function is to set registers's value from read modbus to internal memory
@param  slaves's parameter
@param  start address
@param  number of bytes
@param  data
@retval no
******************************************************************************/
void        master_set_data_holding_register(MODBUS_SLAVES_t *slave,uint16_t input_start_address,uint8_t byte_count,uint8_t *input_data)
{  
    uint16_t temp_mem = 0;
    uint16_t size_of_holdingRegister = slave->memory_size.holdingRegister;//*(&slave->memory.holdingRegister + 1) - slave->memory.holdingRegister;
	if( byte_count <= size_of_holdingRegister )
	{
        for(int i=0; i<byte_count; i+=2)
        {
            temp_mem = ((input_data[i] << 8) & 0xFF00 ) | ((input_data[i+1]) & 0x00FF);
            slave->memory.holdingRegister[input_start_address + i/2] = temp_mem;        
        }   
    }   
}

/******************************************************************************
@brief  This is function is to make read coli command
@param  slaves's parameter
@param  slaves's address
@param  start address
@param  number of bytes
@retval no
******************************************************************************/
FIFO_error_status   master_read_coil(MODBUS_SLAVES_t *slave,uint16_t slave_address,uint16_t input_start_address,uint16_t input_quantity)
{
    uint16_t    CRC_value=0;
    FIFO_error_status    return_value = FIFO_ERROR;    
    
    slave->put_command.data[0] = slave_address;
    slave->put_command.data[1] = MODBUS_READ_SINGLE_COIL;
    slave->put_command.data[2] = (input_start_address >> 8) & 0xFF;
    slave->put_command.data[3] = (input_start_address) & 0xFF;
    slave->put_command.data[4] = (input_quantity >> 8) & 0xFF;
    slave->put_command.data[5] = (input_quantity) & 0xFF;
    CRC_value = master_ModRTU_CRC(slave->put_command.data,6);
    slave->put_command.data[6] = (CRC_value) & 0xFF;
    slave->put_command.data[7] = (CRC_value >> 8) & 0xFF;    
	
    slave->put_command.command_size = 8;
    slave->put_command.response_size=(5+(input_quantity/8 +1 )*2);     
    
    return_value = FIFO_put(slave); 
       
    return     return_value;         
}

/******************************************************************************
@brief  This is function is to make read discrete command
@param  slaves's parameter
@param  slaves's address
@param  start address
@param  number of bytes
@retval no
******************************************************************************/
FIFO_error_status   master_read_discrete_input(MODBUS_SLAVES_t *slave,uint16_t slave_address,uint16_t input_start_address,uint16_t input_quantity)
{
    uint16_t    CRC_value=0;

    FIFO_error_status    return_value = FIFO_ERROR;    
    
    slave->put_command.data[0] = slave_address;
    slave->put_command.data[1] = MODBUS_READ_DISCRETE_INPUT;
    slave->put_command.data[2] = (input_start_address >> 8) & 0xFF;
    slave->put_command.data[3] = (input_start_address) & 0xFF;
    slave->put_command.data[4] = (input_quantity >> 8) & 0xFF;
    slave->put_command.data[5] = (input_quantity) & 0xFF;
    CRC_value = master_ModRTU_CRC(slave->put_command.data,6);
    slave->put_command.data[6] = (CRC_value) & 0xFF;
    slave->put_command.data[7] = (CRC_value >> 8) & 0xFF;    
    slave->put_command.command_size = 8;
    slave->put_command.response_size=(5+(input_quantity/8 +1 )*2);    

    return_value = FIFO_put(slave);      
    return     return_value;        
}

/******************************************************************************
@brief  This is function is to make read regidter command
@param  slaves's parameter
@param  slaves's address
@param  start address
@param  number of bytes
@retval no
******************************************************************************/
FIFO_error_status   master_read_holding_register(MODBUS_SLAVES_t *slave,uint16_t slave_address,uint16_t input_start_address,uint16_t input_quantity)
{
    uint16_t    CRC_value=0;

    FIFO_error_status    return_value = FIFO_ERROR;    
    
    slave->put_command.data[0] = slave_address;
    slave->put_command.data[1] = MODBUS_READ_HOLDING_REGISTERS;
    slave->put_command.data[2] = (input_start_address >> 8) & 0xFF;
    slave->put_command.data[3] = (input_start_address) & 0xFF;
    slave->put_command.data[4] = (input_quantity >> 8) & 0xFF;
    slave->put_command.data[5] = (input_quantity) & 0xFF;
    CRC_value = master_ModRTU_CRC(slave->put_command.data,6);
    slave->put_command.data[6] = (CRC_value) & 0xFF;
    slave->put_command.data[7] = (CRC_value >> 8) & 0xFF;    
    slave->put_command.command_size = 8;
    slave->put_command.response_size=(5+input_quantity*2);        

    return_value = FIFO_put(slave);       
    return     return_value;           
}

/******************************************************************************
@brief  This is function is to make read regidter command
@param  slaves's parameter
@param  slaves's address
@param  start address
@param  number of bytes
@retval no
******************************************************************************/
FIFO_error_status   master_read_input_register(MODBUS_SLAVES_t *slave,uint16_t slave_address,uint16_t input_start_address,uint16_t input_quantity)
{
    uint16_t    CRC_value=0;
    FIFO_error_status    return_value = FIFO_ERROR;
    
    slave->put_command.data[0] = slave_address;
    slave->put_command.data[1] = MODBUS_READ_INPUT_REGISTER;
    slave->put_command.data[2] = (input_start_address >> 8) & 0xFF;
    slave->put_command.data[3] = (input_start_address) & 0xFF;
    slave->put_command.data[4] = (input_quantity >> 8) & 0xFF;
    slave->put_command.data[5] = (input_quantity) & 0xFF;
    CRC_value = master_ModRTU_CRC(  slave->put_command.data,6);
    slave->put_command.data[6] = (CRC_value) & 0xFF;
    slave->put_command.data[7] = (CRC_value >> 8) & 0xFF;    

    slave->put_command.command_size = 8;
    slave->put_command.response_size=(5+input_quantity*2);    

    return_value = FIFO_put(slave);    
	if( return_value == FIFO_ERROR )
    {
        slave->FIFO_Full_error=1;
    }
    else
    {
        slave->FIFO_Full_error=0;
    }
    return     return_value;          
}

/******************************************************************************
@brief  This is function is to make write coil command
@param  slaves's parameter
@param  slaves's address
@param  start address
@param  data
@retval no
******************************************************************************/
FIFO_error_status   master_write_single_coil(MODBUS_SLAVES_t *slave,uint16_t slave_address,uint16_t input_start_address,uint16_t input_data)
{
    uint16_t    CRC_value=0;

    FIFO_error_status    return_value = FIFO_ERROR;    

    slave->put_command.data[0] = slave_address;
    slave->put_command.data[1] = MODBUS_WRITE_SINGLE_COIL;
    slave->put_command.data[2] = (input_start_address >> 8) & 0xFF;
    slave->put_command.data[3] = (input_start_address) & 0xFF;
    if( input_data == 1 )
    {
        slave->put_command.data[4] = 0xFF;
        slave->put_command.data[5] = 0x00;  
    }
    else
    {
        slave->put_command.data[4] = 0x00;
        slave->put_command.data[5] = 0x00;   
    }
    CRC_value= master_ModRTU_CRC( slave->put_command.data,6);
    slave->put_command.data[6] = (CRC_value) & 0xFF;
    slave->put_command.data[7] = (CRC_value >> 8) & 0xFF;    
		 
    slave->put_command.command_size = 8;
    slave->put_command.response_size= 8;    

    return_value = FIFO_put(slave);        
    return     return_value;  
}

/******************************************************************************
@brief  This is function is to make write register command
@param  slaves's parameter
@param  slaves's address
@param  start address
@param  data
@retval no
******************************************************************************/
FIFO_error_status   master_write_single_register(MODBUS_SLAVES_t *slave,uint16_t slave_address,uint16_t input_start_address,uint16_t input_data)
{
    uint16_t    CRC_value=0;
    FIFO_error_status    return_value = FIFO_ERROR;    

     slave->put_command.data[0] = slave_address;
     slave->put_command.data[1] = MODBUS_WRITE_SINGLE_REGISTER;
     slave->put_command.data[2] = (input_start_address >> 8) & 0xFF;
     slave->put_command.data[3] = (input_start_address) & 0xFF;
     slave->put_command.data[4] = (input_data >> 8) & 0xFF;
     slave->put_command.data[5] = (input_data) & 0xFF;
    CRC_value= master_ModRTU_CRC(  slave->put_command.data,6);
     slave->put_command.data[6] = (CRC_value) & 0xFF;
     slave->put_command.data[7] = (CRC_value >> 8) & 0xFF;    
			 
     slave->put_command.command_size = 8;
     slave->put_command.response_size= 8;    

    return_value = FIFO_put(slave);      
    return     return_value;      
}
/******************************************************************************
@brief  The requested ON / OFF states are specified by contents of the query data field. A logical 1 in a bit position of the field requests the corresponding coils to be ON. A logical 0 requests it to be OFF. 
Below is an example of a request to write a series of ten coils starting at coil 20 (addressed as 19, or 13 hex) in slave device 17. 

The query data contents are two bytes: CD 01 hex (1100 1101 0000 0001 binary). The binary bits correspond to the coils in the following way: 
Bit:  1  1  0  0  1  1  0  1  0 0 0 0 0 0 0  1 
Coil: 27 26 25 24 23 22 21 20 - - - - - - 29 28 
@param  slaves's parameter
@param  slaves's address
@param  start address
@param  data
@retval no
******************************************************************************/
FIFO_error_status   master_write_multiple_coil(MODBUS_SLAVES_t *slave,uint16_t slave_address,uint16_t input_start_address,uint16_t coil_amount,uint16_t *input_data)
{
    uint16_t    CRC_value=0;
    uint16_t    bytes_amount = (((coil_amount-1)/8) +1);
    uint8_t     all_bytes = bytes_amount +7;
    FIFO_error_status    return_value = FIFO_ERROR;    

    slave->put_command.data[0] = slave_address;
    slave->put_command.data[1] = MODBUS_WRITE_MULTIPLE_COIL;
    slave->put_command.data[2] = (input_start_address >> 8) & 0xFF;
    slave->put_command.data[3] = (input_start_address) & 0xFF;
    slave->put_command.data[4] = (coil_amount >> 8) & 0xFF;
    slave->put_command.data[5] = (coil_amount) & 0xFF;
    slave->put_command.data[6] = bytes_amount;

    for(int i=0; i< bytes_amount; i++)
    {
        slave->put_command.data[(i*2 + 0) + 7] = ((input_data[i] >> 8 ) & 0xFF);
        slave->put_command.data[(i*2 + 1) + 7] = (input_data[i] & 0xFF);
    }
    CRC_value = master_ModRTU_CRC(  slave->put_command.data, all_bytes);
    slave->put_command.data[all_bytes] = (CRC_value) & 0xFF;
    slave->put_command.data[all_bytes+1] = (CRC_value >> 8) & 0xFF;    
         
    slave->put_command.command_size = all_bytes +2;
    slave->put_command.response_size= 8;    

    return_value = FIFO_put(slave);      
    return     return_value;       
}

/******************************************************************************
@brief  This is function is to make write register command
@param  slaves's parameter
@param  slaves's address
@param  start address
@param  data
@retval no
******************************************************************************/
FIFO_error_status   master_write_multiple_register(MODBUS_SLAVES_t *slave,uint16_t slave_address,uint16_t input_start_address,uint16_t register_amount,uint16_t *input_data)
{
    uint16_t    CRC_value=0;
    uint16_t    byte_amount = (register_amount*2);
    uint8_t     all_bytes = byte_amount +7;

    FIFO_error_status    return_value = FIFO_ERROR;    

    slave->put_command.data[0] = slave_address;
    slave->put_command.data[1] = MODBUS_WRITE_MULTIPLE_REGISTER;
    slave->put_command.data[2] = (input_start_address >> 8) & 0xFF;
    slave->put_command.data[3] = (input_start_address) & 0xFF;

    slave->put_command.data[4] = (register_amount >> 8) & 0xFF;
    slave->put_command.data[5] = (register_amount) & 0xFF;
    slave->put_command.data[6] = byte_amount;

    for(int i=0; i< register_amount; i++)
    {
        slave->put_command.data[(i*2 + 0) + 7] = ((input_data[i] >> 8 ) & 0xFF);
        slave->put_command.data[(i*2 + 1) + 7] = (input_data[i] & 0xFF);
    }
    CRC_value = master_ModRTU_CRC( slave->put_command.data, all_bytes);
    slave->put_command.data[all_bytes+0] = ((CRC_value) & 0xFF);
    slave->put_command.data[all_bytes+1] = ((CRC_value>>8) & 0xFF);    

    slave->put_command.command_size = all_bytes +2;
    slave->put_command.response_size= 8;    

    return_value = FIFO_put(slave);       
    return     return_value;       
}

/******************************************************************************
@brief  This is function timer callback 
@param  htim   
@retval none
******************************************************************************/
void TIM_MASTER_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    master_polling(MODBUS_PORT);
}

/******************************************************************************
@brief  This is function to create a modbus port
@param  modbus port   
@retval none
******************************************************************************/
void create_ModBus(MODBUS_PORT_t *port)
{
	MODBUS_PORT     = port;
	htim_DEV		= port->setting.TIMER.htim;

	/* Timer Config */
    TIM_MasterConfigTypeDef sMasterConfig = {0};
	htim_DEV->Instance = port->setting.TIMER.Instance;
    htim_DEV->Init.Prescaler =  port->setting.TIMER.Prescaler;
    htim_DEV->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_DEV->Init.Period = port->setting.TIMER.Period;
    htim_DEV->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(htim_DEV) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(htim_DEV, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
	htim_DEV->PeriodElapsedCallback     = TIM_MASTER_PeriodElapsedCallback;
	
    if (HAL_UART_Init(MODBUS_PORT->setting.UART.huart) != HAL_OK)
    {
        Error_Handler();
    }    
    /* UART Config */
    MODBUS_PORT->setting.UART.huart->Instance = port->setting.UART.Instance;
    MODBUS_PORT->setting.UART.huart->Init.BaudRate = port->setting.UART.BaudRate;
    MODBUS_PORT->setting.UART.huart->Init.WordLength = UART_WORDLENGTH_8B;
    MODBUS_PORT->setting.UART.huart->Init.StopBits = UART_STOPBITS_1;
    MODBUS_PORT->setting.UART.huart->Init.Parity = UART_PARITY_NONE;
    MODBUS_PORT->setting.UART.huart->Init.Mode = UART_MODE_TX_RX;
    MODBUS_PORT->setting.UART.huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    MODBUS_PORT->setting.UART.huart->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(MODBUS_PORT->setting.UART.huart) != HAL_OK)
    {
        Error_Handler();
    }

    /* UART interrupts */
    MODBUS_PORT->setting.UART.huart->TxCpltCallback    = MASTER_TxCpltCallback;
    MODBUS_PORT->setting.UART.huart->RxEventCallback   = MASTER_RxCpltCallback;
    /* UART RTS */
    HAL_GPIO_WritePin(MODBUS_PORT->setting.UART.RTS.Port,MODBUS_PORT->setting.UART.RTS.Pin,GPIO_PIN_RESET);
    HAL_UARTEx_ReceiveToIdle_DMA(MODBUS_PORT->setting.UART.huart, (uint8_t *) MODBUS_PORT->setting.UART.rec_data_value, 100);
    __HAL_DMA_DISABLE_IT(MODBUS_PORT->setting.UART.RX_hdma, DMA_IT_HT);				

	/* start Timer */
    HAL_TIM_Base_Start_IT(htim_DEV);
};
/******************************************************************************
@brief  This is function for uartRX callback
@param  uart    
@retval none
******************************************************************************/
void	MASTER_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size)
{ 
	for ( int i = 0; i<MODBUS_PORT->slaves_numbers; i++)
	{
		if( MODBUS_PORT->setting.UART.huart ==  huart)
		{
			/* UART Fill buffer */
			__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);    
			HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *)MODBUS_PORT->setting.UART.rec_data_value, Size);
			MODBUS_PORT->setting.UART.rec_data_size = Size;
            if( Size != 0 )
            {
                if( (MODBUS_PORT->setting.LED.Port != NULL ) && (MODBUS_PORT->setting.LED.Pin != NULL ) )
                {
                    HAL_GPIO_WritePin(MODBUS_PORT->setting.LED.Port,MODBUS_PORT->setting.LED.Pin,GPIO_PIN_RESET);
                }  
            }
            else
            { 
            }    
		}
	}	
}

/******************************************************************************
@brief  This is function for uart tx completion interrupt
@param  huart   
@retval 
******************************************************************************/
void	MASTER_TxCpltCallback(UART_HandleTypeDef *huart)
{
	for ( int i = 0; i<MODBUS_PORT->slaves_numbers; i++)
	{
		if( MODBUS_PORT->setting.UART.huart ==  huart)
		{
			/* UART RTS */
			if( (MODBUS_PORT->setting.UART.RTS.Port != NULL) && ( MODBUS_PORT->setting.UART.RTS.Pin != NULL) )
			{
				HAL_GPIO_WritePin(MODBUS_PORT->setting.UART.RTS.Port,MODBUS_PORT->setting.UART.RTS.Pin,GPIO_PIN_RESET);
			}	
            if( (MODBUS_PORT->setting.LED.Port != NULL ) && (MODBUS_PORT->setting.LED.Pin != NULL ) )
            {
                HAL_GPIO_WritePin(MODBUS_PORT->setting.LED.Port,MODBUS_PORT->setting.LED.Pin,GPIO_PIN_SET);
            }             
		}
	}
}

