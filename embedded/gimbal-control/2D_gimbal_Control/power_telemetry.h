//RESISTOR VALUES FOR TELEMETRY
extern const double R_sense;

//Low Side current function
double lsCurrentSense(double V_ADC, double R_f, double R_g, double R_sense);

//high side current function
double hsCurrentSense(double V_ADC, double R_f, double R_g, double R_sense);



