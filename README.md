# ModBusMaster for ST MCU 
create a modbus master polling slaves with time out and retrying .. settings for ST MCUs
You can create multiple Modbus ports in MCU with multiple definition of slaves and set time out and retrying value.

* simple .c and .h file
* This code is tested by ST MCU for four meter eastron sdm54 and sdm120.
* It uses standard framing and polling
* you can use setting LED indicator
* It uses buffer data that you can use send coomands easily every where you want and it sends commands periodicaly untile buffer will be empty.
* implements for all (coils and discrete bit mode) and (holding and input register)
* function call supported : 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x0f, 0x10
* use separate memory for each coils discrete-input holding-register and input-register
* using uart DMA
 

1. select a uart port with interrupt and ( rx + tx )dma enabled in stm32cubemx.
2. select a timer with interrupt enabled in stm32cubemx.
3. otional select a GPIO as modbus communication indicator.
4. in stm32cubemx -> Project Manager -> for uart select (Do not Generate function call , unselect visibility )
5. in stm32cubemx -> Project Manager -> for uart register callback ( enabled )
6. in stm32cubemx -> Project Manager -> for timer register callback ( enabled )

