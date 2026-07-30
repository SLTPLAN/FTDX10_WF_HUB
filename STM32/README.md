目前支持了STM32F103C8T6（Blue Pill） 使用PlatformIO构建。可能是因为速度慢，我没有添加串口功能，按照我写的参考文档，将ACC的CNT TX与CNT RX连在一起就可以了。

At present, the STM32F103C8T6 (Blue Pill) is supported, and it is built using PlatformIO. Because of the limited speed, I did not include the serial port function. As per the reference document I provided, you just need to short the ACC's CNT TX and CNT RX pins together.

STM32 Blue Pill	→	DX10/101 ACC 

PA4 (CS)	→	Pin6

PA5 (SCK)	→	Pin5

PA7 (MOSI)	→	Pin4

GND	—	GND

USB	→	PC（PWR+DATA）

更新日志：
7/30：优化帧读取方式，防止因帧不完整导致的卡顿
