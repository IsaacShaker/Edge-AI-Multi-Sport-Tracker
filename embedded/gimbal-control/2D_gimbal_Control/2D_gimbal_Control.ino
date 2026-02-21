@ -38,11 +38,39 @@ float target_angle_top = 0;     // tilt/pitch target (radians)
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
@ -59,6 +87,78 @@ void doTargetBoth(char* cmd) {
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

void getPos(char* cmd) {
  //Get the position in radians of the bottom and top motor encoders.
  float bottomPos = motorBottom.shaftAngle();
  float topPos = motorTop.shaftAngle();

  Serial.print("Top Position (Rads): "); Serial.print(topPos, 3);
  Serial.print(" | Bottom Position (Rads): "); Serial.print(bottomPos, 3);
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



// =====================================================
// SETUP
// =====================================================
@ -90,7 +190,7 @@ void setup() {
  motorBottom.controller = MotionControlType::angle;

  // Velocity PID
  motorBottom.PID_velocity.P = 0.5f;
  motorBottom.PID_velocity.P = 0.01f;
  motorBottom.PID_velocity.I = 0;
  motorBottom.PID_velocity.D = 0;
  motorBottom.PID_velocity.output_ramp = 100;
@ -121,7 +221,7 @@ void setup() {

  // Velocity PID - may need different tuning for tilt axis
  // (tilt axis often has different load/inertia than pan)
  motorTop.PID_velocity.P = 0.05f;
  motorTop.PID_velocity.P = 0.01f;
  motorTop.PID_velocity.I = 0;
  motorTop.PID_velocity.D = 0;
  motorTop.PID_velocity.output_ramp = 100;
@ -143,13 +243,13 @@ void setup() {
  // -------------------------------------------------
  // Commander setup
  // -------------------------------------------------
  command.add('B', doTargetBottom, "bottom/pan angle (rad)");
  command.add('T', doTargetTop, "top/tilt angle (rad)");
  command.add('B', doTargetBottom, "bottom/pan angle (rad), relative?");
  command.add('T', doTargetTop, "top/tilt angle (rad), relative?");
  command.add('M', doTargetBoth, "both: M<pan> <tilt>");

  // Move to home position
  motorBottom.move(0);
  motorTop.move(0);
  command.add('Y', bSetPID, "Bottom: P<P> <I> <D>");
  command.add('P', tSetPID, "Top: Y<P> <I> <D>");
  command.add('X', getPos, "X: <Top (rad/s)> <Bottom (rad/s)>");
  command.add('V', setVelocity, "Usage: V <motor (0 for bottom, 1 for top)> <velocity (rad/s)>");

  Serial.println(F("=== 2D Gimbal Ready ==="));
  Serial.println(F("Commands:"));
@ -158,6 +258,9 @@ void setup() {
  Serial.println(F("  M<pan> <tilt>   - set both angles"));
  Serial.println(F("  Example: B1.57  T0.5  M1.57 0.5"));
  _delay(1000);

  target_angle_bottom = motorBottom.shaftAngle();
  target_angle_top = motorTop.shaftAngle();
}

// =====================================================
