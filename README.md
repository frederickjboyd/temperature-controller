# Temperature Controller #
High-performance temperature controller to regulate the temperature of laser diodes using a thermoelectric cooler.  
The primary goal of this code is to implement a PID controller that can be easily adapted to various scenarios depending on a users' needs.  
*Code adapted from "dual_unipolar_temp_controller" by Shreyas Potnis*  

## Hardware: ##
* chipKIT uC32
* Analog Shield
* Custom PCB

Two main components of the temperature controller circuit (custom PCB):
1. Temperature sensing
  * A TH10K thermistor is used as part of a voltage divider to create a voltage divider that is dependent on temperature.
  * INA188 takes the difference between the voltage divider and the set point and amplifies it by a factor of 100.
  * Error signal is sent into the Analog Shield via ADC0.
2. Temperature controlling
  * A fundamental property of op amps is exploited to control the current going through the Thermoelectric Cooler (TEC).
  * The inverting input is connected to one end of a sense resistor (the other side of the sense resistor is grounded).
  * Since both inputs of the op amp must be at the same voltage, a specified voltage can be sent into the noninverting input via DAC0.
  * Since the voltage at both op amp inputs must be the same, the voltage sent through DAC0 will be the same voltage across the sense resistor.
  * The voltage of DAC0 and the resistance of the sense resistor is known, so the current can be calculated through I = V/R.
  * The sense resistor is connected to the TEC in series, so their currents are the same.
  * The PID controller code calculates a voltage to send to DAC0 to appropriately adjust the current going through the TEC.
