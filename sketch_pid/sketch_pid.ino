/*
 * Sample showing PID control of a motor.
 * A switch switches the motor between two positions.
 */


#include "hallmotor.cpp"

// We'll use a switch to change between two positions
const int SWITCH = 13;

// Out 2 motors
HallMotor motorA;
HallMotor motorB;

void setup() {
  Serial.begin(9600); 

  // Motor A setup
  motorA = HallMotor(7, 6, 10, 3, 2, hallInterruptA); 
  motorA.setup();
  motorA.ticksPer360 = 75; // change according to your motor
  motorA.maxSpeed = 80;
  motorA.minSpeed = 30;
  motorA.kp = 1.35; 
  motorA.ki = 0.00005; 
  motorA.kd = 0.01; 

  // Motor B setup
  motorB = HallMotor(5, 4, 9, 1, 0, hallInterruptB); 
  motorB.setup();
  motorB.ticksPer360 = 75; // change according to your motor
  motorB.maxSpeed = 80;
  motorB.minSpeed = 30;
  motorB.kp = 1.35; 
  motorB.ki = 0.00005; 
  motorB.kd = 0.01; 

  // Set up switch pin
  pinMode(SWITCH, INPUT);
}

void loop() {
   // Output for debugging
    //if (fabs(errorA)>0) {
    char buffer[100];
    sprintf(buffer, "tarpos:%d, dir:%d, speed:%d, err:%d, pos:%d", motorA.targetPosition, motorA.direction, motorA.speed, motorA.error, motorA.currentPosition);
    strcat(buffer, ", pid:");    
    dtostrf(motorA.pidOutput,10, 1, buffer+strlen(buffer));
    Serial.println(buffer);
    //}
  
  // Read desired position
  if (digitalRead(SWITCH)) {
    motorA.targetPosition = 0;
    motorB.targetPosition = 0;
  }
  else {
    motorA.targetPosition = 75*10; // 10 turns of a 75 ticksPer360 motor
    motorB.targetPosition = 75*10; // 10 turns of a 75 ticksPer360 motor
  }

  // Move to the desired position
  motorA.calculatePID();
  motorA.driveMotor();  
  motorB.calculatePID();
  motorB.driveMotor();  
  
}

// Interrupt handler for motor A
void hallInterruptA() {
    // Now read hall encoder 2, so we can work out the direction of spin
    int hall2Value = digitalRead(motorA.hallPin2);
  
    // Adjust the current position based on the direction
    if (hall2Value == 1) // clockwise
      motorA.currentPosition++;
    else // anti-clockwise
      motorA.currentPosition--;
}

// Interrupt handler for motor B
void hallInterruptB() {
    // Now read hall encoder 2, so we can work out the direction of spin
    int hall2Value = digitalRead(motorB.hallPin2);
  
    // Adjust the current position based on the direction
    if (hall2Value == 1) // clockwise
      motorB.currentPosition++;
    else // anti-clockwise
      motorB.currentPosition--;
}
