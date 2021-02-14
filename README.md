# STM32F4_Measuring_with_I2C_SPI

This is demo project.

The system using BMP180 and MPU9250 sensors for measuring to air temp, air pressure and roll angle

Connections:
BMP180  -> I2C
MPU9250 -> SPI & DMA
PC      -> USART & IT

How to work?

Board gets data from usart with interrupt,
Starts measuring data from sensors
Sends data back to usart
