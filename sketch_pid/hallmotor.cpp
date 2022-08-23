/*
 * Control a DC motor with a hall sensor.
 */
 
#include <arduino.h>

class HallMotor {
  public:
    // Pins
    int fwdPin;
    int bwdPin;
    int speedPin;
    int hallPin1;
    int hallPin2;

    // Interrupt service routine
    void (*isr)();

  public:
    // Motor characteristics
    int ticksPer360;              // number of hall sensor triggers for a 360 degree turn of the motor shaft
    int maxSpeed = 80;            // max speed we will drive the motor
    int minSpeed = 30;            // min speed we will drive the motor (if too low the motor won't move)

    // Motor state
    int direction = 0;            // 0, 1 for clockwise and counter-clockwise (depends on wiring order!)
    int speed = 0;                // 0-100

    // Error tracking
    int error = 0;                // current positional error
    int previousError = 0;        // previous positional error

    // PID calculation
    float pidOutput = 0;          // this will determine the motor speed and direction
    
    // PID constants 
    float kp = 1.35;              // proportional constant
    float ki = 0.00005;           // integral constant
    float kd = 0.01;              // derivative constant

   public:
    // Motor position (the units are hall sensor ticks)
    int targetPosition = 0;             // where we want the motor to go
    volatile int currentPosition = 0;   // where the motor is 

  public:
    // Constructor
    HallMotor() {
    }

    // Constructor
    HallMotor(int fwdPin, int bwdPin, int speedPin, int hallPin1, int hallPin2, void (*isr)(void) ) {
      this->fwdPin = fwdPin;
      this->bwdPin = bwdPin;
      this->speedPin = speedPin;
      this->hallPin1 = hallPin1;
      this->hallPin2 = hallPin2;
      this->isr = isr;
    }

    // Set up pins and interrupts
    void setup() {
        // Set up motor pins
        pinMode(this->fwdPin, OUTPUT);
        pinMode(this->bwdPin, OUTPUT);
        pinMode(this->speedPin, OUTPUT);
      
        // Set up interrupts for the hall sensor
        pinMode(this->hallPin1, INPUT);
        digitalWrite(this->hallPin1, HIGH);
        attachInterrupt(digitalPinToInterrupt(this->hallPin1), this->isr, RISING);
    }


    // Compute the PID output
    void calculatePID()
    {
      float pidP, pidD, pidI;
    
      float currentTime = 0;      // time in the moment of calculation
      float previousTime = 0;     //for calculating delta t
      float deltaTime = 0;        //time difference
      
      //Determining the elapsed time
      currentTime = micros(); //current time
      deltaTime = (currentTime - previousTime) / 1000000.0; //time difference in seconds
      previousTime = currentTime; //save the current time for the next iteration to get the time difference
      
      // Calculate current error
      this->error = this->currentPosition - this->targetPosition; //Current position - target position (or setpoint)
    
      // Calcuate the PID output
      pidP = this->kp * this->error;
      pidD = this->kd * ((this->error - this->previousError) / deltaTime);
      pidI = this->ki * (pidI + this->error); // ki * (pidI + this->error * deltaTime);
      this->pidOutput = pidP + pidD + pidI;
    
      // Save the error so we can compute the derivative next time round
      this->previousError = this->error; 
    }

    // Drive the motors according to the PID output
    void driveMotor()
    {
      // Adjust direction based on PID output
      if (this->pidOutput < 0) 
        this->direction = -1; // anti-clockwise
      else if (this->pidOutput > 0) 
        this->direction = 1;  // clockwise
      else 
        this->direction = 0;  // stop
    
      // Adjust speed based on PID output
      speed = (int)fabs(this->pidOutput); 
      if (speed > this->maxSpeed) 
        speed = this->maxSpeed; 
      if (speed < minSpeed && this->error != 0)
        speed = this->minSpeed;
    
      // Now turn the motor
      if (this->direction == -1) 
        this->turnMotorBackward(speed);
      else if (this->direction == 1) 
        this->turnMotorForward(speed);
      else 
      {
        this->stopMotor();
        speed = 0;
      }
    }    

    // Turn motor forwards at speed s (from 0 to 100)
    void turnMotorForward(int s){              
      digitalWrite(this->fwdPin, HIGH);
      digitalWrite(this->bwdPin, LOW); 
      analogWrite(this->speedPin, s*255/100);
    }
    
    // Turn motor backwards at speed s (from 0 to 100)
    void turnMotorBackward(int s){
      digitalWrite(this->fwdPin,LOW);
      digitalWrite(this->bwdPin,HIGH);
      analogWrite(this->speedPin, s*255/100);
    }
    
    // Stop motor 
    void stopMotor(){
      digitalWrite(this->fwdPin, LOW);
      digitalWrite(this->bwdPin, LOW);
    }    
};
