#include <SimpleFOC.h>

// =====================================================
// SENSOR CONFIGURATION
// =====================================================
// Each AS5600 on its own I2C bus (same address 0x36, separate buses)
// Bottom motor sensor on Wire (SDA0/SCL0)
// Top motor sensor on Wire1 (SDA1/SCL1)

MagneticSensorI2C sensorBottom = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensorTop = MagneticSensorI2C(AS5600_I2C);

// =====================================================
// MOTOR & DRIVER CONFIGURATION
// =====================================================

// --- Bottom Motor (Pan/Yaw) ---
// pole pairs=7, phase resistance=2.3Ω, KV=220, inductance=0.00086H
BLDCMotor motorBottom = BLDCMotor(7, 2.3, 220, 0.00086);
BLDCDriver3PWM driverBottom = BLDCDriver3PWM(1, 2, 3, 0);  // PWM pins + enable

// --- Top Motor (Tilt/Pitch) ---
// ADJUST these values to match your top motor specs!
// Using same motor type as bottom as placeholder
BLDCMotor motorTop = BLDCMotor(7, 2.3, 220, 0.00086);
BLDCDriver3PWM driverTop = BLDCDriver3PWM(6,7,8,5);  // PWM pins + enable
// ^^^ CHANGE THESE PINS to match your wiring!

// =====================================================
// CONTROL VARIABLES
// =====================================================
float target_angle_bottom = 0;  // pan/yaw target (radians)
float target_angle_top = 0;     // tilt/pitch target (radians)

// =====================================================
// COMMANDER (Serial Interface)
// =====================================================
Commander command = Commander(Serial);

void doTargetBottom(char* cmd) {
  command.scalar(&target_angle_bottom, cmd);
  float target, relative;
  //If relative (second variable), is enabled (1), then offset the current location.
  //else, send non-relative location offset.
  if (sscanf(cmd, "%f %f", &target, &relative) == 2) {
    if(relative == 1){
      target_angle_bottom = fmod(motorBottom.shaftAngle() + target, 6.28f);
    }
    else{
      target_angle_bottom = fmod(target, 6.28f);
    }
  }
  else{
    Serial.println("Usage: B<angle> <relative?>  (e.g., B1.0 1)");
  }
}


void doTargetTop(char* cmd) {
  command.scalar(&target_angle_top, cmd);
  float target, relative;
  //If relative (second variable), is enabled (1), then offset the current location.
  //else, send non-relative location offset.
  if (sscanf(cmd, "%f %f", &target, &relative) == 2) {
    if(relative == 1){
      target_angle_top = fmod(motorTop.shaftAngle() + target, 6.28f);
    }
    else{
      target_angle_top = fmod(target, 6.28f);
    }
    
  }
  else{
    Serial.println("Usage: T<angle> <relative?>  (e.g., B1.0 1)");
  }
}

// Move both motors at once: expects "pan_angle tilt_angle"
void doTargetBoth(char* cmd) {
  // Parse two floats from the command string
  float pan, tilt;
  if (sscanf(cmd, "%f %f", &pan, &tilt) == 2) {
    target_angle_bottom = pan;
    target_angle_top = tilt;
    Serial.print("Pan: "); Serial.print(pan, 3);
    Serial.print(" | Tilt: "); Serial.println(tilt, 3);
  } else {
    Serial.println("Usage: M<pan> <tilt>  (e.g., M1.57 0.5)");
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
    Serial.print("bottom P: "); Serial.print(bP, 3);
    Serial.print(" | bottom I: "); Serial.print(bI, 3);
    Serial.print(" | bottom D: "); Serial.println(bD, 3);
  } 
  else {
    Serial.println("Usage: Bconfig <P> <I> <D>");
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
    Serial.print("top P: "); Serial.print(tP, 3);
    Serial.print(" | top I: "); Serial.print(tI, 3);
    Serial.print(" | top D: "); Serial.println(tD, 3);
  } 
  else {
    Serial.println("Usage: Tconfig <P> <I> <D>");
  }
}

//Set velocity of either motors in single command.
void setVelocity(char* cmd) {
  //Get the position in radians of the bottom and top motor encoders.
  float velocity, motor;

  //Motor = Input1, Velocity = Input2
  if (sscanf(cmd, "%f %f", &motor, &velocity) == 2) {
    if(motor == 1){
      motorTop.velocity_limit = velocity;
      Serial.print("Top Motor ");
    }
    else{
      motorBottom.velocity_limit = velocity;
      Serial.print("Bottom Motor ");
    }
    Serial.print("Velocity (Rad/s): "); Serial.println(velocity, 3);
  }

  else{
    Serial.println("Usage: V <motor (0 for bottom, 1 for top)> <velocity (rad/s)>");
  }
}

void getPos(char* cmd) {
  //Get the position in radians of the bottom and top motor encoders.
  float bottomPos = motorBottom.shaftAngle();
  float topPos = motorTop.shaftAngle();
  Serial.print("Top Position (Rads): "); Serial.print(topPos, 3);
  Serial.print(" | Bottom Position (Rads): "); Serial.print(bottomPos, 3);
}

// K1 = enable (hold current position)
// K0 = disable (motors go limp)
void doEnableDisable(char* cmd) {
  if (cmd[0] == '1') {
    motorBottom.enable();
    motorTop.enable();
    // Hold whatever position the axes are currently at
    target_angle_bottom = motorBottom.shaftAngle();
    target_angle_top    = motorTop.shaftAngle();
    Serial.println(F("Motors enabled"));
  } else {
    motorBottom.disable();
    motorTop.disable();
    Serial.println(F("Motors disabled"));
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);
  Serial.println(F("=== 2D Gimbal Initializing ==="));

  // -------------------------------------------------
  // Initialize sensors
  // -------------------------------------------------
  Serial.println(F("Init bottom sensor (Wire / SDA0)..."));
  sensorBottom.init(&Wire);   // I2C bus 0 (SDA0/SCL0)

  Serial.println(F("Init top sensor (Wire1 / SDA1)..."));
  sensorTop.init(&Wire1);     // I2C bus 1 (SDA1/SCL1)

  // -------------------------------------------------
  // Bottom Motor Setup (Pan/Yaw)
  // -------------------------------------------------
  Serial.println(F("--- Bottom Motor (Pan) ---"));
  motorBottom.linkSensor(&sensorBottom);

  driverBottom.voltage_power_supply = 12;
  driverBottom.init();
  motorBottom.linkDriver(&driverBottom);

  motorBottom.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motorBottom.controller = MotionControlType::angle;

  // Velocity PID
  motorBottom.PID_velocity.P = 0.5f;
  motorBottom.PID_velocity.I = 0;
  motorBottom.PID_velocity.D = 0;
  motorBottom.PID_velocity.output_ramp = 100;

  motorBottom.voltage_limit = 2.3;
  motorBottom.LPF_velocity.Tf = 0.01f;

  // Angle P controller
  motorBottom.P_angle.P = 10;
  motorBottom.velocity_limit = 25;

  motorBottom.useMonitoring(Serial);
  motorBottom.init();
  motorBottom.initFOC();

  // -------------------------------------------------
  // Top Motor Setup (Tilt/Pitch)
  // -------------------------------------------------
  Serial.println(F("--- Top Motor (Tilt) ---"));
  motorTop.linkSensor(&sensorTop);

  driverTop.voltage_power_supply = 12;
  driverTop.init();
  motorTop.linkDriver(&driverTop);

  motorTop.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motorTop.controller = MotionControlType::angle;

  // Velocity PID - may need different tuning for tilt axis
  // (tilt axis often has different load/inertia than pan)
  motorTop.PID_velocity.P = 0.05f;
  motorTop.PID_velocity.I = 0;
  motorTop.PID_velocity.D = 0;
  motorTop.PID_velocity.output_ramp = 100;

  motorTop.voltage_limit = 2.3;
  motorTop.LPF_velocity.Tf = 0.01f;

  // Angle P controller
  motorTop.P_angle.P = 10;
  motorTop.velocity_limit = 25;

  // Optional: limit tilt range to prevent gimbal hitting frame
  // motorTop.P_angle.limit = 1.57;  // ~90 degrees max travel

  motorTop.useMonitoring(Serial);
  motorTop.init();
  motorTop.initFOC();

  // -------------------------------------------------
  // Commander setup
  // -------------------------------------------------
  command.add('B', doTargetBottom, "bottom/pan angle (rad), relative?");
  command.add('T', doTargetTop, "top/tilt angle (rad), relative?");
  command.add('M', doTargetBoth, "both: M<pan> <tilt>");
  command.add('Y', bSetPID, "Bottom: P<P> <I> <D>");
  command.add('P', tSetPID, "Top: Y<P> <I> <D>");
  command.add('X', getPos, "X: <Top (rad/s)> <Bottom (rad/s)>");
  command.add('V', setVelocity, "Usage: V <motor (0 for bottom, 1 for top)> <velocity (rad/s)>");
  command.add('K', doEnableDisable, "K1=enable (hold pos), K0=disable (limp)");

  Serial.println(F("=== 2D Gimbal Ready ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("  B<angle>        - set pan/bottom angle (rad)"));
  Serial.println(F("  T<angle>        - set tilt/top angle (rad)"));
  Serial.println(F("  M<pan> <tilt>   - set both angles"));
  Serial.println(F("  K1 / K0         - enable / disable motors"));
  Serial.println(F("  Example: B1.57  T0.5  M1.57 0.5"));
  // Sentinel that the Pi waits for before sending any commands
  Serial.println(F("READY"));
  _delay(1000);
  target_angle_bottom = motorBottom.shaftAngle();
  target_angle_top = motorTop.shaftAngle();
}


// =====================================================
// MAIN LOOP
// =====================================================
void loop() {
  // Run FOC for both motors - keep this as fast as possible
  motorBottom.loopFOC();
  motorTop.loopFOC();

  // Position control updates
  motorBottom.move(target_angle_bottom);
  motorTop.move(target_angle_top);

  // Serial command processing
  command.run();
}