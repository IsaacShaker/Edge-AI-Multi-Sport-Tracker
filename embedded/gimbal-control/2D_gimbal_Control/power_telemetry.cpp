#include "power_telemetry.h"

//SHUNT RESISTOR VALUES
const double gain = 200; // V per V
const double R71 = 82500; //ohms
const double R67 = 10000; //ohms
const double R_IMON = 1200; // ohms
const double G_IMON = 243; // uA per A

//This function finds the current passing through a sense resistor using the constant gain of 200 of the INA4180A4IPWR
double currentSense(double V_ADC, double R_SENSE){
    return V_ADC / ((R_SENSE) * gain);
}

//This function calculates the voltage of the main supply rail using the known resistor divider values
double systemVoltage(double V_SYS){
    return V_SYS * ((R71 + R67) / R67);
}

//This function takes the buffered voltage from the TPS259830LNRGER IMON output to find the current supplied to the entire system
double systemCurrent(double V_IMON){
    return V_IMON / ((R_IMON) * G_IMON);
}