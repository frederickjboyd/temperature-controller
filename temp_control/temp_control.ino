#include <analogShield.h>
#include <math.h>

#define UNO_ID "TEMP_CO_0\r\n"
#define GATE_VOLTAGE_MIN 0
#define GATE_VOLTAGE_MAX 2.0
#define N_ACCUM 100

#define ZEROV 32768 // Zero volts

float toVolts(float bits);
unsigned int toBits(float volts);
float resistance(double T);
float voltageDivCalc(float R, float Vi);

struct Params {
  unsigned int enable0;      // Enable PID loop
  unsigned int enable_heat;  // 0 if cooling object, 1 if heating object
  unsigned int set_temp0;    // Voltage to send as reference into INA188 (DAC2)
  unsigned int arb_voltage0; // Arbitrary voltage to send into voltage divider (DAC1)
  float prop_gain0;          // Proportional gain
  float pi_pole0;            // Integral gain
  float pd_pole0;            // Derivative gain
};

struct Logger {
  unsigned int gate_voltage0; // Voltage applied to noninverting input on OP97 in bits
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

/* ----- SET TEMPERATURE ----- */
double T = 30;
/* --------------------------- */

float R = resistance(T);

float toVolts(float bits) {
  float volts = ((float)(bits - 32768)) * (5.0 / 32768.0);
  return volts + (0.0183 * volts); // ADC error correction factor
}

unsigned int toBits(float volts) {
  volts -= (-0.0132 * volts) + 0.0173; // DAC error correction factor
  return (volts * 32768.0 / 5.0) + 32768;
}

/* Given a specified temperature T(C), calculates the resistance of a thermistor (TH10K from Thorlabs) */
float resistance(double T) {
  T += 273.15; // Converting C -> K
  /* Coefficients to calculate resistance as a function of thermistor temperature */
  /* Values from TH10K datasheet - Temp Range: (0, 49)C */
  double A = -1.5470381e1;
  double B = 5.6022839e3;
  double C = -3.7886070e5;
  double D = 2.4971623e7;
  return 10000 * exp(A + B/T + C/pow(T, 2) + D/pow(T, 3));
}

/* Calculates v_o of a voltage divider with thermistor and a 10k resistor */
float voltageDivCalc(float R, float Vi) {
  return Vi * R / (R + 10000);
}

void setup() {
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  /* Open serial communications, initialize output ports: */
  Serial.begin(115200);

  /* Setting parameters */
  params.enable0 = 1;
  params.enable_heat = 0;
  float Vi = 3.0;
  float Vo = voltageDivCalc(R, Vi);
  Serial.println(Vi);
  Serial.println(R);
  Serial.println(Vo);
  params.arb_voltage0 = toBits(Vi);
  params.set_temp0 = toBits(Vo);
  Serial.println("STARTING NEW RUN...");
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

  // EEPROM_readAnything(0, params);
  // EEPROM_readAnything(sizeof(params), logger);
  current_time = micros();
}

void loop() {
  unsigned long previous_time = current_time;
  current_time = micros();
  logger.dt_seconds = ((float)(current_time - previous_time)) * 1e-6;

  /*
  if (Serial.available())
    parseSerial();
  */

  float error_signal_prev0 = error_signal_instant0;
  error_signal_instant0 = toVolts(analog.read(0));

  /* Accounting for fluctuations in DAC and ADC */
  float Vi = toVolts(analog.read(1));
  float Vo = voltageDivCalc(R, Vi);
  params.set_temp0 = toBits(Vo);
  analog.write(2, params.set_temp0);
  //Serial.println("Vi:");
  //Serial.println(Vi);
  //Serial.println("Vo:");
  //Serial.println(Vo);

  /* Logging error signal average */
  logger.error_signal0 = error_signal_instant0 * alpha_avg + logger.error_signal0 * (1. - alpha_avg);
  if (params.enable0) {
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

    if (params.enable_heat) {
      gv0 *= -1;
    }

    if (gv0 > GATE_VOLTAGE_MAX) {
      gv0 = GATE_VOLTAGE_MAX;
      logger.accumulator0 = gv0;
    }

    if (gv0 < GATE_VOLTAGE_MIN) {
      gv0 = GATE_VOLTAGE_MIN;
      logger.accumulator0 = gv0;
    }

    logger.gate_voltage0 = toBits(gv0);
  }

  else {
    logger.gate_voltage0 = toBits(GATE_VOLTAGE_MIN);
    logger.accumulator0 = 0.0;
  }

  analog.write(0, logger.gate_voltage0);

}


/*
template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
  const byte* p = (const byte*)(const void*)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
    EEPROM.write(ee++, *p++);
  return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
  byte* p = (byte*)(void*)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(ee++);
  return i;
}

void parseSerial() {
  char byte_read = Serial.read();
  if (byte_read == 'g') {
    // get params, send the entire struct in one go
    Serial.write((const uint8_t*)&params, sizeof(Params));

  }
  if (byte_read == 'l') {
    // send the logger data
    Serial.write((const uint8_t*)&logger, sizeof(Logger));

  }
  if (byte_read == 'w') {
    // write to EEPROM
    EEPROM_writeAnything(0, params);
    EEPROM_writeAnything(sizeof(params), logger);
  }
  if (byte_read == 'r') {
    EEPROM_readAnything(0, params);
    EEPROM_readAnything(sizeof(params), logger);
  }
  if (byte_read == 'i') {
    // return ID
    Serial.write(UNO_ID);

  }
  if (byte_read == 's') {
    // read in size(Params) bytes
    Params params_temp;
    int bytes_read = Serial.readBytes((char *) &params_temp, sizeof(Params));
    // check for validity of parameters
    params = params_temp;
  }
}
*/
