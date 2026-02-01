# testModBusMaster
create a modbus master polling slaves with time out and retrying .. settings for ST MCUs

1. select a uart port with interrupt and ( rx + tx )dma enabled in stm32cubemx.
2. select a timer with interrupt enabled in stm32cubemx.
3. otional select a GPIO as modbus communication indicator.
4.  in stm32cubemx -> Project Manager -> for uart select (Do not Generate function call , unselect visibility )
5.  in stm32cubemx -> Project Manager -> for uart register callback ( enabled )
6.  in stm32cubemx -> Project Manager -> for timer register callback ( enabled )

