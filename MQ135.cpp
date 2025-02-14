/**************************************************************************/
/*!
@file     MQ135.cpp
*/
/**************************************************************************/

#include "MQ135.h"

/**************************************************************************/
/*!
@brief  Default constructor

@param[in] pin  The analog input pin for the readout of the sensor
*/
/**************************************************************************/
MQ135::MQ135(PinName pin) : _pin(pin) {}

MQ135::~MQ135() {}

/**************************************************************************/
/*!
@brief  Get the correction factor to correct for temperature and humidity

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The calculated correction factor
*/
/**************************************************************************/
float MQ135::getCorrectionFactor(float t, float h) {
 if(t < 20){
        return CORA * t * t - CORB * t + CORC - (h-33.)*CORD;
    } else {
        return CORE * t + CORF * h + CORG;
    }
}

/**************************************************************************/
/*!
@brief  Get the resistance of the sensor, ie. the measurement value

@return The sensor resistance in kOhm
*/
/**************************************************************************/
float MQ135::getResistance() {
  AnalogIn ain(_pin);
  float analogValue = ain.read();
  
  // Debug print for analog value
  
  // Check if analog value is within expected range
  if (analogValue <= 0 || analogValue >= 1) {
    printf("Error: Invalid analog value %f\r\n", analogValue);
    return -1;
  }
  
  float intVal = analogValue * 1023;
  
  // Check if integer value is within expected range
  if (intVal <= 0 || intVal >= 1023) {
    printf("Error: Invalid integer value %d\r\n", intVal);
    return -1;
  }
  
  float resistance = ((1023.0 / intVal) - 1.0) * RLOAD;
  
  return resistance;
}

/**************************************************************************/
/*!
@brief  Get the resistance of the sensor, ie. the measurement value corrected
        for temp/hum

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The corrected sensor resistance kOhm
*/
/**************************************************************************/
float MQ135::getCorrectedResistance(float t, float h) {
  return getResistance() / getCorrectionFactor(t, h);
}

/**************************************************************************/
/*!
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air)

@return The ppm of CO2 in the air
*/
/**************************************************************************/
float MQ135::getPPM() {
  float resistance = getResistance();
  
  if (resistance <= 0) {
    printf("Error: Invalid resistance %f\r\n", resistance);
    return -1;
  }
  
  float ppm = PARA * pow((resistance / RZERO), -PARB);
  
  // Debug print for PPM calculation

  return ppm;
}

/**************************************************************************/
/*!
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air), corrected
        for temp/hum

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The ppm of CO2 in the air
*/
/**************************************************************************/
float MQ135::getCorrectedPPM(float t, float h) {
  return PARA * pow((getCorrectedResistance(t, h) / RZERO), -PARB);
}

/**************************************************************************/
/*!
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air)

@return The ppm of CO2 in the air
*/
/**************************************************************************/
float MQ135::getPPM(float rzero) {
  return PARA * pow((getResistance() / rzero), -PARB);
}

/**************************************************************************/
/*!
@brief  Get the ppm of CO2 sensed (assuming only CO2 in the air), corrected
        for temp/hum

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The ppm of CO2 in the air
*/
/**************************************************************************/
float MQ135::getCorrectedPPM(float t, float h, float rzero) {
  return PARA * pow((getCorrectedResistance(t, h) / rzero), -PARB);
}

/**************************************************************************/
/*!
@brief  Get the resistance RZero of the sensor for calibration purposes

@return The sensor resistance RZero in kOhm
*/
/**************************************************************************/
float MQ135::getRZero() {
  return getResistance() * pow((ATMOCO2 / PARA), (1. / PARB));
}

/**************************************************************************/
/*!
@brief  Get the corrected resistance RZero of the sensor for calibration
        purposes

@param[in] t  The ambient air temperature
@param[in] h  The relative humidity

@return The corrected sensor resistance RZero in kOhm
*/
/**************************************************************************/
float MQ135::getCorrectedRZero(float t, float h) {
  return getCorrectedResistance(t, h) * pow((ATMOCO2 / PARA), (1. / PARB));
}
