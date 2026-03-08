#include "power_telemetry.h"

//SHUNT RESISTOR VALUES
const double R_sense = 1000;

double currentSense(double V_ADC, double gain){
    return V_ADC / ((R_sense) * gain);
}

