//RESISTOR VALUES FOR TELEMETRY
extern const double R_sense;

//Finds the current passing through a sense resistor using the constant gain of 200 of the INA4180A4IPWR
double currentSense(double V_ADC, double R_SENSE);

//Calculates the voltage of the main supply rail using the known resistor divider values
double systemVoltage(double V_SYS);

//Takes the buffered voltage from the TPS259830LNRGER IMON output to find the current supplied to the entire system
double systemCurrent(double V_IMON);