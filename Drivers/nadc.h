#ifndef __NADC_H
#define __NADC_H

void ADC_Init(void);
void adc_sample_trigger(void);

float get_battery_voltage(void);
void battery_voltage_detection(void);

extern low_voltage vbat;
#endif




