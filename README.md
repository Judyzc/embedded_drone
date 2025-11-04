Repository for Embedded Systems design project: quadcopter drone.

`BreadDrone.ino`: initialize and read data from IMU, time of flight, and pressure sensors. Test that sensors are responsive, data makes sense, and ESP can sample fast enough.


Main System-on-Chip: STM32F405RG
- CPU: 168 MHz ARM Cortex M4 with single-precision FPU
- RAM: 192 KB SRAM

Our MCU: ESP32-MINI-1-N4
-  CPU: 240 MHz
-  Radio: 2.412 - 2.484 GHz
