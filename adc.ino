/* ========================================================================== */
/*   adc.ino                                                                  */
/*                                                                            */
/*   Code for reading/averaging ADC channels                                  */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/* ========================================================================== */

// Variables

unsigned long CheckADCChannels=0;
#ifdef A0_MULTIPLIER
  unsigned int Channel0Readings[5];
  unsigned int Channel0Average;
#endif

void SetupADC(void)
{
  #ifdef A0_MULTIPLIER
    analogReference(DEFAULT);
    // Serial.println("Setup A0");
    pinMode(A0, INPUT);
  #endif
  pinMode(BATT_ADC,INPUT);
  pinMode(PYRO_ADC,INPUT);
}

void CheckADC(void)
{
  if (millis() >= CheckADCChannels)
  {
    #ifdef A0_MULTIPLIER
      Channel0Average = ReadADC(A0, A0_MULTIPLIER, Channel0Readings);
      // Serial.print("Average=");Serial.println(Channel0Average);
    #endif
  
    CheckADCChannels = millis() + 1000L;
  }
}

unsigned int ReadADC(int Pin, float Multiplier, unsigned int *Readings)
{
  int i;
  unsigned int Result;
  
  for (i=0; i<4; i++)
  {
    Readings[i] = Readings[i+1];
  }

  Readings[4] = analogRead(Pin);
  //Serial.print("A0=");Serial.println(Readings[4]);
  
  Result = 0;
  for (i=0; i<5; i++)
  {
    Result += Readings[i];
  }
  
  return (float)Result * Multiplier / 5.0;
}

// Connected directly from the battery to the ADC.
// We scale this value into one byte using the fact that the voltage will always be
// between 0.5 and 2V if we are turned on.
// Convert back to actual voltage using:
// actual = 0.5 + 1.5*raw/255.0
uint8_t GetBattVoltage(){
  float batt_reading = analogRead(BATT_ADC)*3.3/1024.0;
  return (uint8_t)((batt_reading-0.5) / (1.5/255.0));
}

// Connected via a 15K/4.7K Voltage Divider. Therefore, we multiply value by 4.9 to get actual voltage.
// We scale to one byte by assume the voltage will be between 0 and 5V. (Valid, I guess...)
// Scale back up using:
// actual = raw * 5.0/255.0
uint8_t GetPyroVoltage(){
  float batt_reading = 4.19*analogRead(PYRO_ADC)*3.3/1024.0;
  return (uint8_t)((batt_reading) / (5.0/255.0));
}
