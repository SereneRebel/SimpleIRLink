# SimpleIRLink
Simple data link for transferring small packets of data using off-the-shelf IR modules like IR-08H

This demonstration sends a packet from the master side of 16 bytes with a packed id byte, command byte, data length byte and 2 crc bytes.
The packet is reeived and decoded by the slave side, after which the command byte is inverted and the packet id is marked as ACK(0x80) and sent back.
The master awaits the reply, as well as discrading echoed sent packet.

## Setup
Master side uses a Raspberry pi CM4 with UART4 enabled in config.txt. Other pi/linux sbc can be used with adapting the uart define.
Slave side uses an STM32 Nucleo F303 board

### Connections - Master side
![20241213_152533](https://github.com/user-attachments/assets/d4bcde2c-48a2-4396-b394-913dea238c31)

### Connections - Slave side
![20241213_152539](https://github.com/user-attachments/assets/d4af9b9d-8cdf-4a77-907a-73272faf867c)

