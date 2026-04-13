#undef HSE_VALUE
#define HSE_VALUE 25000000U

#include <SimpleFOC.h>
#include <EEPROM.h>
#include <Wire.h>
#include "definitions.h"
#include "gimbal_settings.h"
#include "power_telemetry.h"

//---------------------------------------
//    CLOCK CONFIGURATION (only done on STM32F412RET6TR)
//---------------------------------------

extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
#if defined(RCC_PLLR_SUPPORT)
  RCC_OscInitStruct.PLL.PLLR = 2;
#endif

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    while (1) {}
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    while (1) {}
  }

  // Make sure the global clock variable is updated
  SystemCoreClockUpdate();
}


//---------------------------------------
//    SERIAL DEBUG
//---------------------------------------
HardwareSerial SerialCM(USART2);
HardwareSerial SerialDebug(USART3);
#define Serial SerialDebug


//---------------------------------------
//    SENSOR CONFIGURATION
//---------------------------------------

MagneticSensorI2C sensorBottom = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensorTop = MagneticSensorI2C(AS5600_I2C);

//---------------------------------------
//    MOTOR & DRIVER CONFIGURATION
//---------------------------------------

// --- Bottom Motor (Pan/Yaw) ---
BLDCMotor motorBottom = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, KV_RATING, Q_INDUCTANCE);
BLDCDriver3PWM driverBottom = BLDCDriver3PWM(B_PH1, B_PH2, B_PH3, B_EN);  // PWM pins + enable

// --- Top Motor (Tilt/Pitch) ---
BLDCMotor motorTop = BLDCMotor(POLE_PAIRS, PHASE_RESISTANCE, KV_RATING, Q_INDUCTANCE);
BLDCDriver3PWM driverTop = BLDCDriver3PWM(T_PH1, T_PH2 , T_PH3, T_EN);  // PWM pins + enable

//---------------------------------------
//    CONTROL VARIABLES
//---------------------------------------


//Motor Movement Limits
float target_angle_bottom = 0;  // pan/yaw target (radians)
float target_angle_top = 0;     // tilt/pitch target (radians)

float target_angle_bottom_min = 1.5; // pan/yaw target angle minimum
float target_angle_bottom_max = 3.5; // pan/yaw target angle maximum
float target_angle_top_min = -5.0; // tilt/pitch target angle minimum
float target_angle_top_max = -3.0; // tilt/pitch target angle maximum

float home_angle_bottom = 0;  // pan/yaw target (radians) defualt
float home_angle_top = 0;     // tilt/pitch target (radians) default

//Positional Telemetry
bool returnPosition = false; //Position Logging (Serial)
unsigned long pos_last_time = 0;
uint32_t pos_counter = 0;

/*
Power Telemetry
*/
bool checkPower = false; //Power Safety
unsigned long pow_last_time = 0;
uint32_t pow_counter = 0;

bool returnPower = false; //Power Return

float current_sys = 0;
float power_sys = 0;
float voltage_sys = 0;
float current_12V = 0;
float current_5V = 0;
float current_3V3 = 0;
float current_BP1 = 0;
float current_TP1 = 0;
float current_BP2 = 0;
float current_TP2 = 0;
float current_BP3 = 0;
float current_TP3 = 0;

//---------------------------------------
//    MEMORY SETTINGS
//---------------------------------------
Settings settings;  //Instatiate the gimbal settings.

//---------------------------------------
//    COMMANDER (Serial Interface)
//---------------------------------------
Commander command = Commander(SerialDebug);

/**
 * MOTOR MOVEMENT COMMANDS
 */

float checkBounds(int motor, float angle_in){
  if(motor == 0){
    if(angle_in < target_angle_bottom_min) return target_angle_bottom_min;
    if(angle_in > target_angle_bottom_max) return target_angle_bottom_max;
  }
  if(motor == 1){
    if(angle_in < target_angle_top_min) return target_angle_top_min;
    if(angle_in > target_angle_top_max) return target_angle_top_max;
  }
  return angle_in;
}


void doTargetBottom(char* cmd) {
  command.scalar(&target_angle_bottom, cmd);
  float target, relative;
  //If relative (second variable), is enabled (1), then offset the current location.
  //else, send non-relative location offset.
  if (sscanf(cmd, "%f %f", &target, &relative) == 2) {
    if(relative == 1){
      target_angle_bottom = checkBounds(0, (fmod(motorBottom.shaftAngle() + target, 6.28f)));
    }
    else{
      target_angle_bottom = checkBounds(0, (fmod(target, 6.28f)));
    }
  }
  else{
    SerialDebug.println("Usage: B<angle> <relative?>  (e.g., B1.0 1)");
  }
}

void doTargetTop(char* cmd) {
  command.scalar(&target_angle_top, cmd);
  float target, relative;
  //If relative (second variable), is enabled (1), then offset the current location.
  //else, send non-relative location offset.
  if (sscanf(cmd, "%f %f", &target, &relative) == 2) {
    if(relative == 1){
      target_angle_top = checkBounds(1, (fmod(motorTop.shaftAngle() + target, 6.28f)));
    }
    else{
      target_angle_top = checkBounds(1, (fmod(target, 6.28f)));
    }
    
  }
  else{
    SerialDebug.println("Usage: T<angle> <relative?>  (e.g., B1.0 1)");
  }
}


// Move both motors at once: expects "pan_angle tilt_angle"
void doTargetBoth(char* cmd) {
  // Parse two floats from the command string
  float pan, tilt;
  if (sscanf(cmd, "%f %f", &pan, &tilt) == 2) {
    target_angle_bottom = checkBounds(0, pan);
    target_angle_top = checkBounds(1, tilt);
  } else {
    SerialDebug.println("Usage: M<pan> <tilt>  (e.g., M1.57 0.5)");
  }
}

void homeMotors(char* cmd) {
  target_angle_top = settings.top_home;
  target_angle_bottom = settings.bottom_home;
}

void lock_motors(char* cmd){
  float enableMotor;
  if (sscanf(cmd, "%f", &enableMotor) == 1){
    if(enableMotor){
      motorBottom.enable();
      motorTop.enable();
      SerialDebug.println("Motors Enabled");
    }
    else{
      motorTop.disable();
      motorBottom.disable();
      SerialDebug.println("Motors Disabled");
    }
  }
  else{
    SerialDebug.println("Usage: K <(0 for disable, 1 for enable)>");
  }
}



/**
 *  Settings Adjustment Commands
 */

//Set velocity of either motors in single command.
void setVelocity(char* cmd) {
  //Get the position in radians of the bottom and top motor encoders.
  float velocity, motor;

  //Motor = Input1, Velocity = Input2
  if (sscanf(cmd, "%f %f", &motor, &velocity) == 2) {
    if(motor == 1){
      motorTop.velocity_limit = velocity;
      SerialDebug.print("Top Motor ");
    }
    else{
      motorBottom.velocity_limit = velocity;
      SerialDebug.print("Bottom Motor ");
    }
    SerialDebug.print("Velocity (Rad/s): "); SerialDebug.println(velocity, 3);
  }

  else{
    SerialDebug.println("Usage: V <motor (0 for bottom, 1 for top)> <velocity (rad/s)>");
  }
}

void bSetPID(char* cmd) {
  //parse three floats from cmd, map to P, I, D.
  float bP, bI, bD;

  //If exists, set bottom P, I, D.
  if (sscanf(cmd, "%f %f %f", &bP, &bI, &bD) == 3) {
    motorBottom.PID_velocity.P = bP;
    motorBottom.PID_velocity.I= bI;
    motorBottom.PID_velocity.D = bD;
    motorBottom.PID_velocity.reset();
    SerialDebug.print("bottom P: "); SerialDebug.print(bP, 3);
    SerialDebug.print(" | bottom I: "); SerialDebug.print(bI, 3);
    SerialDebug.print(" | bottom D: "); SerialDebug.println(bD, 3);
  } 
  else {
    SerialDebug.println("Usage: Bconfig <P> <I> <D>");
  }
}


void tSetPID(char* cmd) {
  //parse three floats from cmd, map to P, I, D.
  float tP, tI, tD;

  //If exists, set top P, I, D.
  if (sscanf(cmd, "%f %f %f", &tP, &tI, &tD) == 3) {
    motorTop.PID_velocity.P = tP;
    motorTop.PID_velocity.I= tI;
    motorTop.PID_velocity.D = tD;
    motorTop.PID_velocity.reset();
    SerialDebug.print("top P: "); SerialDebug.print(tP, 3);
    SerialDebug.print(" | top I: "); SerialDebug.print(tI, 3);
    SerialDebug.print(" | top D: "); SerialDebug.println(tD, 3);
  } 
  else {
    SerialDebug.println("Usage: Tconfig <P> <I> <D>");
  }
}

//Set LPF of either motors in single command.
void setLPF(char* cmd) {
  //Get the position in radians of the bottom and top motor encoders.
  float seconds, motor;

  //Motor = Input1, Seconds = Input2
  if (sscanf(cmd, "%f %f", &motor, &seconds) == 2) {
    if(motor == 1){
      motorTop.LPF_velocity.Tf = seconds;
      SerialDebug.print("Top Motor ");
    }
    else{
      motorBottom.LPF_velocity.Tf = seconds;
      SerialDebug.print("Bottom Motor ");
    }
    SerialDebug.print("Seconds (s): "); SerialDebug.println(seconds, 3);
  }

  else{
    SerialDebug.println("Usage: L <motor (0 for bottom, 1 for top)> <seconds (s)>");
  }
}

void getInfo(char* cmd) {
  SerialDebug.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
  //Get the position in radians of the bottom and top motor encoders.
  float bottomPos = motorBottom.shaftAngle();
  float topPos = motorTop.shaftAngle();
  SerialDebug.print("Top Position (Rads): "); SerialDebug.print(topPos, 3);
  SerialDebug.print(" | Bottom Position (Rads): "); SerialDebug.println(bottomPos, 3);

  //Return the saved home position.
  SerialDebug.print("Top Home Pos (Rads): "); SerialDebug.print(home_angle_top, 3);
  SerialDebug.print(" | Bottom Home Pos (Rads): "); SerialDebug.println(home_angle_bottom, 3);


  //Return the current PID values.
  float tP, tI, tD;
  tP = motorTop.PID_velocity.P;
  tI = motorTop.PID_velocity.I;
  tD = motorTop.PID_velocity.D;
  SerialDebug.print("top P: "); SerialDebug.print(tP, 3);
  SerialDebug.print(" | top I: "); SerialDebug.print(tI, 3);
  SerialDebug.print(" | top D: "); SerialDebug.println(tD, 3);

  float bP, bI, bD;
  bP = motorBottom.PID_velocity.P;
  bI = motorBottom.PID_velocity.I;
  bD = motorBottom.PID_velocity.D;
  SerialDebug.print("bottom P: "); SerialDebug.print(bP, 3);
  SerialDebug.print(" | bottom I: "); SerialDebug.print(bI, 3);
  SerialDebug.print(" | bottom D: "); SerialDebug.println(bD, 3);

  //return the current velocity limits.
  float bV, tV, tLPF, bLPF;
  tV = motorTop.velocity_limit;
  bV = motorBottom.velocity_limit;
  SerialDebug.print("bottom vlimit: "); SerialDebug.print(bV, 3);
  SerialDebug.print(" | top vlimit: "); SerialDebug.println(tV, 3);
  tLPF = motorTop.LPF_velocity.Tf;
  bLPF = motorBottom.LPF_velocity.Tf;
  SerialDebug.print("bottom LPF: "); SerialDebug.print(bLPF, 3);
  SerialDebug.print(" | top LPF: "); SerialDebug.println(tLPF, 3);


  SerialDebug.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");

}

void saveSettings(char* cmd){
  float pidSet, vSet, homeSet;

  //input 1 = PID, 2 = velocity, 3 = home
  if (sscanf(cmd, "%f %f %f", &pidSet, &vSet, &homeSet) == 3) {
    bool savedPID = false;
    bool savedV = false;
    bool savedHome = false;

    if(pidSet == 1){
      settings.bottom_p = motorBottom.PID_velocity.P;
      settings.bottom_i = motorBottom.PID_velocity.I;
      settings.bottom_d = motorBottom.PID_velocity.D;
      settings.top_p = motorTop.PID_velocity.P;
      settings.top_i = motorTop.PID_velocity.I;
      settings.top_d = motorTop.PID_velocity.D;

      savedPID = true;
    }
    if(vSet == 1){
      settings.top_vlimit = motorTop.velocity_limit;
      settings.bottom_vlimit = motorBottom.velocity_limit;
      settings.top_lpf = motorTop.LPF_velocity.Tf;
      settings.bottom_lpf = motorBottom.LPF_velocity.Tf;
      savedV = true;

    }
    if(homeSet == 1){
      settings.bottom_home = motorBottom.shaftAngle();
      settings.top_home = motorTop.shaftAngle();
      savedHome = true;
    }
    settings_save(settings);

    // ---- Return Message ----
    SerialDebug.print("SAVE OK | ");

    if (savedPID)  SerialDebug.print("PID ");
    if (savedV)  SerialDebug.print("VEL ");
    if (savedHome) SerialDebug.print("HOME ");

    SerialDebug.println();
  }
  else {
    SerialDebug.println("SAVE ERROR | Usage: S<pid> <vel> <home>  (ex: S1 0 1)");
  }
}




/**
 * Telemetry
 */

void recordData(char* cmd){
  float recordData;
  if (sscanf(cmd, "%f", &recordData) == 1){
    if(recordData){ 
      returnPosition = true;
      pos_last_time = 0;
      pos_counter = 0;
      SerialDebug.println("time_ms,top_angle,bottom_angle");

    }
    else{
      returnPosition = false;
    }
  }
  else{
    SerialDebug.println("Usage: I <(0 for disable, 1 for enable)>");
  }
}

void powerCheck(){
  //System Total Voltage Rule
  if(voltage_sys > VOLT_SYS_MAX){
    //Add code to un-safe shut-off system (I.e. ALL lines cutoff)
    SerialDebug.println("\n\n!!! VOLT_SYS_MAX EXCEEDED! !!!\n\n");
  }
  
  //System Total Current Rule
  if(current_sys > CURR_SYS_MAX){
    //Add code to un-safe shut-off system (I.e. ALL lines cutoff)
    SerialDebug.println("\n\n!!! CURR_SYS_MAX EXCEEDED! !!!\n\n");
  }
  
  //12V Rail Current Rule
  if(current_12V > CURR_12V_MAX){
    ///Add code to disable 12V Rail (Motors mainly)
    motorBottom.disable();
    motorTop.disable();
    SerialDebug.println("\n\n!!! CURR_12V_MAX EXCEEDED! !!!\n\n");
  }

  //5V Rail Current Rule
  if(current_5V > CURR_5V_MAX){
    //Add code to disable 5V Rail (CM5, etc.)
    SerialDebug.println("\n\n!!! CURR_5V_MAX EXCEEDED! !!!\n\n");
  }

  //3.3V Rail Current Rule
  if(current_3V3 > CURR_3V3_MAX){
    //Map to actual enable pin later.
    //Add code to disable 3.3V Rail (SSD, etc.)
    SerialDebug.println("\n\n!!! CURR_3V3_MAX EXCEEDED! !!!\n\n");
  }
  
  //Top and Bottom Motor Current Rule
  if(((current_BP1 + current_BP2 + current_BP3) > CURR_PHASE_MAX) || ((current_TP1 + current_TP2 + current_TP3) > CURR_PHASE_MAX)){
    motorBottom.disable();
    motorTop.disable();
    //Map to actual enable pin(s) later.
    SerialDebug.println("\n\n!!! MOTOR CURRENT MAX(S) EXCEEDED! !!!\n\n");
  }
}

//Pin Configurations (STM32 ONLY)
void configureDigitalInputs() {
  pinMode(EXT_INT, INPUT);
  pinMode(YAW_nFAULT, INPUT);
  pinMode(PITCH_nFAULT, INPUT);
  pinMode(CM_TO_STM, INPUT);
  pinMode(PG_3_3V, INPUT);
  pinMode(PG_12V, INPUT);
  pinMode(PG_5V, INPUT);
}

void configureAnalogInputs() {
  // Optional in STM32duino, but OK for readability
  pinMode(ADC_eFUSE_V, INPUT_ANALOG);
  pinMode(ADC_eFUSE_I, INPUT_ANALOG);
  pinMode(ADC_5V, INPUT_ANALOG);
  pinMode(ADC_12V, INPUT_ANALOG);
  pinMode(ADC_3_3V, INPUT_ANALOG);
  pinMode(ADC_PITCH_RS3, INPUT_ANALOG);
  pinMode(ADC_PITCH_RS2, INPUT_ANALOG);
  pinMode(ADC_PITCH_RS1, INPUT_ANALOG);
  pinMode(ADC_YAW_RS1, INPUT_ANALOG);
  pinMode(ADC_YAW_RS2, INPUT_ANALOG);
  pinMode(ADC_YAW_RS3, INPUT_ANALOG);
}

void configureOutputsSafe() {
  pinMode(YAW_EN, OUTPUT);
  pinMode(YAW_nSLEEP, OUTPUT);
  pinMode(YAW_nRESET, OUTPUT);
  pinMode(PITCH_EN, OUTPUT);
  pinMode(PITCH_nSLEEP, OUTPUT);
  pinMode(STM_TO_CM, OUTPUT);
  pinMode(STM_STAT, OUTPUT);
  pinMode(EN_12V, OUTPUT);
  pinMode(PITCH_nRESET, OUTPUT);
  pinMode(EN_5V, OUTPUT);

  // Safe startup states:
  // Keep power rails and motor drivers disabled until firmware is ready.
  digitalWrite(YAW_EN, LOW);
  digitalWrite(PITCH_EN, LOW);
  digitalWrite(EN_12V, LOW);
  digitalWrite(EN_5V, LOW);

  // Active-low reset/sleep lines held LOW for safety
  digitalWrite(YAW_nSLEEP, LOW);
  digitalWrite(YAW_nRESET, LOW);
  digitalWrite(PITCH_nSLEEP, LOW);
  digitalWrite(PITCH_nRESET, LOW);

  // Communication/status outputs default low
  digitalWrite(STM_TO_CM, LOW);
  digitalWrite(STM_STAT, LOW);
}

void configurePWMOutputs() {
  pinMode(YAW_IN1, OUTPUT);
  pinMode(YAW_IN2, OUTPUT);
  pinMode(YAW_IN3, OUTPUT);
  pinMode(PITCH_IN1, OUTPUT);
  pinMode(PITCH_IN2, OUTPUT);
  pinMode(PITCH_IN3, OUTPUT);

  // Start with PWM off
  analogWrite(YAW_IN1, 0);
  analogWrite(YAW_IN2, 0);
  analogWrite(YAW_IN3, 0);
  analogWrite(PITCH_IN1, 0);
  analogWrite(PITCH_IN2, 0);
  analogWrite(PITCH_IN3, 0);
}

void releaseDrivers() {
  // Call this only after system checks pass
  digitalWrite(YAW_nRESET, HIGH);
  digitalWrite(YAW_nSLEEP, HIGH);
  digitalWrite(PITCH_nRESET, HIGH);
  digitalWrite(PITCH_nSLEEP, HIGH);
}

void enablePowerRails() {
  digitalWrite(EN_5V, HIGH);
  delay(POWER_STAGE_TIME);
  digitalWrite(EN_12V, HIGH);
}

void enableMotorDrivers() {
  digitalWrite(YAW_EN, HIGH);
  digitalWrite(PITCH_EN, HIGH);
}


void initUARTs() {
  SerialCM.setRx(PA3);
  SerialCM.setTx(PA2);
  SerialCM.begin(115200);

  SerialDebug.setTx(PC10);
  SerialDebug.setRx(PC11);
  SerialDebug.begin(115200);

  delay(50);

  SerialDebug.println("Debug UART ready");
  SerialCM.println("CM UART ready");
}


//---------------------------------------
//    SETUP
//---------------------------------------

void setup() {
  
  //SystemClock_Config();
  #if defined(ARDUINO_ARCH_STM32)
  initUARTs();
  TwoWire BottomWire(YAW_SDA_Pin, YAW_SCL_Pin);
  TwoWire TopWire(PITCH_SDA_Pin, PITCH_SCL_Pin);
  //SystemClock_Config();
  configureDigitalInputs(); 
  configureAnalogInputs();
  configureOutputsSafe();
  configurePWMOutputs();
  releaseDrivers();
  enablePowerRails();
  #endif

  SimpleFOCDebug::enable(&Serial);
  SerialDebug.println(F("=== 2D Gimbal Initializing ==="));

  // -------------------------------------------------
  // Initialize sensors - COMMENT OUT CODE BLOCKS
  // -------------------------------------------------

  //
  #if defined(ARDUINO_ARCH_STM32)
      SerialDebug.println(F("Init bottom sensor (BottomWire)..."));
      BottomWire.begin();
      sensorBottom.init(&BottomWire);

      SerialDebug.println(F("Init top sensor (TopWire)..."));
      TopWire.begin();
      sensorTop.init(&TopWire);
  #else
      SerialDebug.println(F("Init bottom sensor (Wire)..."));
      Wire.begin();
      sensorBottom.init(&Wire);

      SerialDebug.println(F("Init top sensor (Wire1)..."));
      Wire1.begin();
      sensorTop.init(&Wire1);
  #endif




  // -------------------------------------------------
  // Motor Setup (Pan/Yaw)
  // -------------------------------------------------

  //Check if valid settings save in memory. If not, load defaults.
  if(!settings_load(settings)){
    SerialDebug.println("No valid settings... Loading defaults.");

     // Default PID values
    settings.bottom_p = 0.01;
    settings.bottom_i = 0.0;
    settings.bottom_d = 0.0;

    settings.top_p = 0.01;
    settings.top_i = 0.0;
    settings.top_d = 0.0;

    settings.bottom_home = 0.0;
    settings.top_home = 0.0;

    settings.top_vlimit = 7.0;
    settings.bottom_vlimit = 7.0;

    settings.top_lpf = 0.01;
    settings.bottom_lpf = 0.01;

    settings_save(settings);
  }


  // -------------------------------------------------
  // Bottom Motor Setup (Pan/Yaw)
  // -------------------------------------------------
  SerialDebug.println(F("--- Bottom Motor (Pan) ---"));
  motorBottom.linkSensor(&sensorBottom);

  driverBottom.voltage_power_supply = 12;
  driverBottom.init();
  motorBottom.linkDriver(&driverBottom);

  motorBottom.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motorBottom.controller = MotionControlType::angle;

  // Velocity PID
  motorBottom.PID_velocity.P = settings.bottom_p;
  motorBottom.PID_velocity.I = settings.bottom_i;
  motorBottom.PID_velocity.D = settings.bottom_d;
  motorBottom.PID_velocity.output_ramp = 100;

  motorBottom.voltage_limit = 2.3;
  motorBottom.LPF_velocity.Tf = settings.bottom_lpf;

  // Angle P controller
  motorBottom.P_angle.P = 10;
  motorBottom.velocity_limit = settings.bottom_vlimit;

  motorBottom.useMonitoring(Serial);
  motorBottom.init();
  motorBottom.initFOC();

  // -------------------------------------------------
  // Top Motor Setup (Tilt/Pitch)
  // -------------------------------------------------
  SerialDebug.println(F("--- Top Motor (Tilt) ---"));
  motorTop.linkSensor(&sensorTop);

  driverTop.voltage_power_supply = 12;
  driverTop.init();
  motorTop.linkDriver(&driverTop);

  motorTop.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motorTop.controller = MotionControlType::angle;

  // Velocity PID - may need different tuning for tilt axis
  motorTop.PID_velocity.P = settings.top_p;
  motorTop.PID_velocity.I = settings.top_i;
  motorTop.PID_velocity.D = settings.top_d;
  motorTop.PID_velocity.output_ramp = 100;

  motorTop.voltage_limit = 2.3;
  motorTop.LPF_velocity.Tf = settings.top_lpf;

  // Angle P controller
  motorTop.P_angle.P = 10;
  motorTop.velocity_limit = settings.top_vlimit;

  motorTop.useMonitoring(Serial);
  motorTop.init();
  motorTop.initFOC();

  // -------------------------------------------------
  // Motor Inital State Setup
  // -------------------------------------------------
  target_angle_bottom = motorBottom.shaftAngle();
  target_angle_top = motorTop.shaftAngle();
  home_angle_bottom = settings.bottom_home;
  home_angle_top = settings.top_home;
  motorBottom.disable();
  motorTop.disable();

  // -------------------------------------------------
  // Commander setup
  // -------------------------------------------------
  command.add('B', doTargetBottom, "bottom/pan angle (rad), relative?");
  command.add('T', doTargetTop, "top/tilt angle (rad), relative?");
  command.add('M', doTargetBoth, "both: M<pan> <tilt>");
  command.add('H', homeMotors, "Home: H");
  command.add('Y', bSetPID, "Bottom: P<P> <I> <D>");
  command.add('P', tSetPID, "Top: Y<P> <I> <D>");
  command.add('X', getInfo, "Get Info: X");
  command.add('I', recordData, "Record Positional Data: I");
  command.add('K', lock_motors, "Enable Motors: K<0 for Disable, 1 for Enable>");
  command.add('V', setVelocity, "Set Velocity: V<motor (0 for bottom, 1 for top)> <velocity (rad/s)>");
  command.add('L', setLPF, "Set LPF: V<motor (0 for bottom, 1 for top)> <seconds>");
  command.add('S', saveSettings, "Save Settings: S <PID(0|1)> <Velocity(0|1)> <Home(0|1)>");

  SerialDebug.println(F("=== 2D Gimbal Ready ==="));
  SerialDebug.println(F("ATTENTION: Motors initialized as disabled!"));
  _delay(1000);
}
  


//---------------------------------------
// MAIN LOOP
//---------------------------------------

void loop() {

  SerialDebug.println("Hello! This is working.");
  delay(500);
  // Run FOC for both motors
  motorBottom.loopFOC();
  motorTop.loopFOC();

  // Position control updates
  motorBottom.move(target_angle_bottom);
  motorTop.move(target_angle_top);

  // Return positional telemetry every 10ms (if enabled) in CSV format.
  if(returnPosition){
    if (millis() - pos_last_time >= 10) {
      pos_last_time += 10;
      pos_counter += 10;

      SerialDebug.print(",");
      SerialDebug.print(pos_counter);                   // time
      SerialDebug.print(",");
      SerialDebug.print(motorTop.shaftAngle(), 3);  // top motor
      SerialDebug.print(",");
      SerialDebug.println(motorBottom.shaftAngle(), 3); // bottom motor
    }
  }


  //ADC
  current_sys = systemCurrent(digitalRead(PA1));
  voltage_sys = systemVoltage(digitalRead(PA0));
  power_sys = current_sys * voltage_sys;
  current_3V3 = currentSense(digitalRead(PC2), 0.0015);
  current_5V = currentSense(digitalRead(PC0), 0.0015);
  current_12V = currentSense(digitalRead(PC1), 0.0015);
  current_BP1 = currentSense(digitalRead(PA4), 0.0100);
  current_TP1 = currentSense(digitalRead(PC5), 0.0100);
  current_BP2 = currentSense(digitalRead(PA5), 0.0100);
  current_TP2 = currentSense(digitalRead(PC4), 0.0100);
  current_BP3 = currentSense(digitalRead(PA6), 0.0100);
  current_TP3 = currentSense(digitalRead(PC3), 0.0100);

  //Check the power values of the current and voltage lines at a rate of ~75hz.
  if(checkPower){
    if (millis() - pow_last_time >= 13) {
      pow_last_time += 13;
      pow_counter += 13;
      powerCheck();
    }
  }


  // Serial command processing
  command.run();
}