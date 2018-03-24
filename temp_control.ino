/* PID controller to regulate the temperature of laser diodes
 * Frederick Boyd, 2018
 * Vutha Labs
 * Code adapted from "dual_unipolar_temp_controller" by Shreyas Potnis
 * 
 * Hardware:
 * - chipKIT uC32
 * - Analog Shield
 * - Custom PCB
 * 
 * Two main components of the temperature controller circuit (custom PCB):
 * 1. Temperature sensing
 * A TH10K thermistor is used as part of a voltage divider to create a voltage divider that is dependent on temperature.
 * INA188 takes the difference between the voltage divider and the set point and amplifies it by a factor of 100.
 * Error signal is sent into the Analog Shield via ADC0.
 * 
 * 2. Temperature controlling
 * A fundamental property of op amps is exploited to control the current going through the Thermoelectric Cooler (TEC).
 * The inverting input is connected to one end of a sense resistor (the other side of the sense resistor is grounded).
 * Since both inputs of the op amp must be at the same voltage, a specified voltage can be sent into the noninverting input via DAC0.
 * Since the voltage at both op amp inputs must be the same, the voltage sent through DAC0 will be the same voltage across the sense resistor.
 * The voltage of DAC0 and the resistance of the sense resistor is known, so the current can be calculated through I = V/R.
 * The sense resistor is connected to the TEC in series, so their currents are the same.
 * The PID controller code calculates a voltage to send to DAC0 to appropriately adjust the current going through the TEC.
 */

#include <analogShield.h>

/* Minimum/Maximum volatges that will be outputted on DAC0 */
#define GATE_VOLTAGE_MIN 0
#define GATE_VOLTAGE_MAX 2.0
#define N_ACCUM 100

#define ZEROV 32768 // Zero volts

struct Params {
  /* Enable/Disable PID controller loop */
  unsigned int enable0;
  /* Arbitrary voltage to send into voltage divider (V-) */
  /* Writes to DAC1 on Analog Shield */
  unsigned int arb_voltage0;
  /* Set point for control system - voltage sent directly into INA188 as a reference (V+) */
  /* NOTE: set_temp0 is dependent on arb_voltage0 */
  /* Writes to DAC2 on Analog Shield */
  unsigned int set_temp0;
  /* PID Controller Parameters */
  float prop_gain0; // Proportional gain
  float pi_pole0;   // Integral gain
  float pd_pole0;   // Derivative gain
};

struct Logger {
  /* Voltage to be sent into OP97 (V+) */
  unsigned int gate_voltage0;
  float error_signal0;
  float accumulator0;
  float dt_seconds;
};

Params params;
Logger logger;

unsigned long current_time;

float accumulator_small0;
float prop_term0;
float derivative_term0;
float error_signal_instant0;

unsigned int n_accum0;

float alpha_avg;
float gv0;


/* Converts a bit value (0 < bits < 65536) to a voltage (-5 < volts < +5) */
float toVolts(unsigned int bits) {
  return (bits - 32768) * (5.0 / 32768.0);
}


/* Converts a voltage (-5 < volts < +5) to a bit value (0 < bits < 65536) */
unsigned int toBits(float volts) {
  return ((unsigned int)(volts * 32768.0 / 5.0) + 32768);
}

void setup() {
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  /* Open serial communications, initialize output ports: */
  Serial.begin(115200);

  /* Setting parameters */
  params.enable0 = 1;
  params.arb_voltage0 = toBits(3.0);
  params.set_temp0 = toBits(1.7);
  params.prop_gain0 = 1;
  params.pi_pole0 = 1;
  params.pd_pole0 = 0;

  /* Initializing logger values */
  logger.gate_voltage0 = ZEROV;
  logger.accumulator0 = 0.0;

  /* Initializing other variables */
  accumulator_small0 = 0.0;
  prop_term0 = 0.0;
  error_signal_instant0 = 0.0;
  n_accum0 = 0;
  derivative_term0 = 0.0;

  alpha_avg = 1. / N_ACCUM;

  analog.write(ZEROV, params.arb_voltage0, 
               params.set_temp0, ZEROV, 
               true);

  /* Time used to calculate integrals and derivatives for PID controller */
  current_time = micros();
  Serial.println("Setup Complete.");
}


void loop() {
  unsigned long previous_time = current_time;
  current_time = micros();
  logger.dt_seconds = ((float)(current_time - previous_time)) * 1e-6;
  
  float error_signal_prev0 = error_signal_instant0;
  error_signal_instant0 = toVolts(analog.read(0));
  Serial.println(error_signal_instant0);
  
  /* Logging error signal average */
  logger.error_signal0 = error_signal_instant0 * alpha_avg + logger.error_signal0 * (1. - alpha_avg);
  if (params.enable0) {
    /* BEGIN: PID Controller Loop */
    accumulator_small0 += error_signal_instant0 * logger.dt_seconds;
    n_accum0 += 1;
    if (n_accum0 > N_ACCUM) {
      n_accum0 = 0;
      logger.accumulator0 += accumulator_small0 * params.prop_gain0 * params.pi_pole0;
      accumulator_small0 = 0;
    }
    
    prop_term0 = 0.99 * prop_term0 + 0.01 * (error_signal_instant0 * params.prop_gain0);
    float der_term = (error_signal_instant0 - error_signal_prev0) / logger.dt_seconds / params.pd_pole0;
    derivative_term0 = 0.995 * derivative_term0 + 0.005 * (der_term * params.prop_gain0);
    
    float gv0 = logger.accumulator0 + prop_term0;
    
    if (gv0 > GATE_VOLTAGE_MAX) {
      gv0 = GATE_VOLTAGE_MAX;
      logger.accumulator0 = gv0;
    }
    else if (gv0 < GATE_VOLTAGE_MIN) {
      gv0 = GATE_VOLTAGE_MIN;
      logger.accumulator0 = gv0;
    }
    logger.gate_voltage0 = toBits(gv0);
    /* END: PID Controller Loop */
  }
  else {
    /* Option to skip PID controller loop and apply minimum specified voltage to DAC0 */
    logger.gate_voltage0 = toBits(GATE_VOLTAGE_MIN);
    logger.accumulator0 = 0.0;
  }
  /* Write the calculated gate_voltage0 to DAC0 */
  analog.write(0, logger.gate_voltage0);
}

