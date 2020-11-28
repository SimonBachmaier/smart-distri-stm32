# Smart Distri Software

Software for STM32 MCU to monitor and restart 9 electrical fuses.  
The software was developed as a student in the Formula Student Team "Strohm und Söhne e.V." in Nuernberg Germany for use in our next car "Nora 7".  
Through the custom board various low voltage components will be powered with 24V.  
One fuse is connected to the so called "shutdown circuit". The fuse connected to the shutdown circuit is monitored to detect which part of the car tripped the shutdown circuit.

The project was created with STM32CubeIDE.  
  
The software was developed for a custom board (board developed by other members of the team) containing the following components:
- MCU: STM32F407VGT6U
- Eletrical fuses: TI TPS26636
- Power monitor: INA226 
    
8 fuses are monitored through the ADC of the MCU. The ninth fuse is monitored through the power monitor to get a more accurate reading.  
  
Communication with the power monitor is handled over I2C.  
CAN messages containing the measured data are being send continuessly at a certain rate.  
When receiving a certain CAN message the software tries to restart the fuses.  
If the "shutdown circuit" is triggered the software sends out CAN messages containing information which components caused the error.  