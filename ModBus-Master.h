/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MODBUS_MASTER_H
#define __MODBUS_MASTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/*******************************************************************************
* @file    ModBus-Master.h
* @author  MR Arash Yousefi
* @brief   ModBus Header file.
  This Header is a header file for using in Modbus protocol. 
********************************************************************************
* @attention
* MAPNA Electric & Control Engineering & Manufacturing Co. 
  MECO. All right reserved.
*
*******************************************************************************/
/*Inclusion of system headers*/
#include "main.h"
#include "stdlib.h"

/******************************************************************************
 ** user manual **
 ------------------------------------------------------------------------------------------------------------------------------------------------------------//
| Address     |            Target      |            Status             |    read Function code     |    single write Function) | multiple write Function     //
| ------------------------------------------------------------------------------------------------------------------------------------------------------------//
| (xx:xx)     |     Coils              |            read/write         |                0x01       |    0x05                   |       0x0F                |
| (xx:xx)     |     Discrete Inputs    |            read only          |                0x02       |     --                    |        --                 |
| (xx:xx)     |     Input Registers    |            read only          |                0x04       |     --                    |        --                 |
| (xx:xx)     |     Holding Registers  |            read/write         |                0x03       |    0x06                   |       0x10                |
 ------------------------------------------------------------------------------------------------------------------------------------------------------------//

-- set uart (transmite DMA and receive interrupt) and enable global interrupt for each slave 
-- add a MODBUS_PORT_t variable for each uart
-- add a MODBUS_SLAVES_t variable for each salves in modbus port
-- add a memory variable for each salves in modbus port

-- fill setting of ports
-- set modbus timer (TIMX) 303us countUp with interrupt
-- at the end create_ModBus(MODBUS_PORT_t *port);
-- you can use SLAVE_RESPONSE_TIME TIME_OUT_VALUE MASTER_MAX_RETRY
-- for using modbus commands use these functions:
master_read_coil
master_read_discrete_input
master_read_holding_register
master_read_input_register
master_write_single_coil
master_write_single_register
master_write_multiple_coil
master_write_multiple_register
******************************************************************************/
#define MODBUS_VERSION                            	    2.1.0              
#define NUMBER_OF_SLAVES                                4

#define FIFO_BUFFER_SIZE                                20 
#define FIFO_DATE_SIZE                                  100           
typedef struct{
    uint8_t command_size;                                               // bytes to send
    uint8_t response_size;                                              // bytes to receive
    uint8_t data[FIFO_DATE_SIZE];                                       // data to send
}command_t;

typedef struct{
    uint16_t in;                                                        // indexpoint to wite
    uint16_t out;                                                       // indexpoint to read
    uint16_t count;                                                     // bytes in FIFO
    command_t command_data[FIFO_BUFFER_SIZE];                           // buffer
}FIFO_t;

typedef enum {FIFO_ERROR =0,FIFO_SUCCESS = !FIFO_ERROR} FIFO_error_status;

typedef enum {
    get_FIFO =0,
    send_command,
    wait_response
} master_polling_t;

typedef struct
{
    uint16_t 		*coil;
    uint16_t 		*discreteInput;
    uint16_t 		*inputRegister;
    uint16_t 		*holdingRegister;
}MEMORY_t;

typedef struct
{
    uint16_t 		coil;
    uint16_t 		discreteInput;
    uint16_t 		inputRegister;
    uint16_t 		holdingRegister;
}MEMORY_SIZE_t;

typedef struct 
{
	master_polling_t                polling_state;
    uint16_t                        start_address_memory;
    uint16_t                        data_quantity;
    uint8_t                         slave_addr;
    uint8_t                         master_retry_send;
    uint32_t                        response_counter;
    command_t                       get_command;
    uint32_t                        polls_counter;
    uint32_t                        timeout_counter;
    uint8_t                         FIFO_Full_error;
    
	FIFO_t                      	master_FIFO;
	command_t                    	put_command;
	uint16_t               			slave_error;
	MEMORY_t						memory;
    MEMORY_SIZE_t                   memory_size;
}MODBUS_SLAVES_t;

typedef struct 
{
    int 				 	slaves_numbers;
    MODBUS_SLAVES_t         *MODBUS_SLAVES[NUMBER_OF_SLAVES];   
	struct
	{
		struct
		{
            void                *Instance;
            TIM_HandleTypeDef	*htim;
            uint32_t            Prescaler;
            uint32_t            Period;
        }TIMER;
        
		struct
		{
			void                *Instance;
			UART_HandleTypeDef	*huart;
			uint32_t        	BaudRate;
			DMA_HandleTypeDef	*RX_hdma;
			void				*MASTER_TxCpltCallback;
			void				*MASTER_RxCpltCallback;
			struct
			{
				GPIO_TypeDef		*Port;
				uint16_t			Pin;
			}RTS;
            uint8_t                         rec_data_value[100];
            uint8_t                         rec_data_size;
		}UART;
		
		struct
		{
			GPIO_TypeDef		*Port;
			uint16_t			Pin;
		}LED;
	}setting;		
}MODBUS_PORT_t;

/* Master general setting */
void create_ModBus(MODBUS_PORT_t *port);

/* functions to send commands to slave */
FIFO_error_status       master_read_coil                (MODBUS_SLAVES_t *slave,uint16_t slave_address,uint16_t input_start_address,uint16_t input_quantity);
FIFO_error_status       master_read_discrete_input      (MODBUS_SLAVES_t *slave,uint16_t slave_address,uint16_t input_start_address,uint16_t input_quantity);
FIFO_error_status       master_read_holding_register    (MODBUS_SLAVES_t *slave,uint16_t slave_address,uint16_t input_start_address,uint16_t input_quantity);
FIFO_error_status       master_read_input_register      (MODBUS_SLAVES_t *slave,uint16_t slave_address,uint16_t input_start_address,uint16_t input_quantity);
FIFO_error_status       master_write_single_coil        (MODBUS_SLAVES_t *slave,uint16_t slave_address,uint16_t input_start_address,uint16_t input_data);
FIFO_error_status       master_write_single_register    (MODBUS_SLAVES_t *slave,uint16_t slave_address,uint16_t input_start_address,uint16_t input_data);
FIFO_error_status       master_write_multiple_coil      (MODBUS_SLAVES_t *slave,uint16_t slave_address,uint16_t input_start_address,uint16_t coil_amount,uint16_t *input_data);
FIFO_error_status       master_write_multiple_register  (MODBUS_SLAVES_t *slave,uint16_t slave_address,uint16_t input_start_address,uint16_t register_amount,uint16_t *input_data);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_MASTER_H */
