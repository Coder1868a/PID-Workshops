// Robot configuration code.
motor LeftMotor = motor(PORT1, ratio18_1, false);

motor RightMotor = motor(PORT2, ratio18_1, true);

// ----------------------------------------------------------------------------
//                                                                            
//    Project:                                               
//    Author:
//    Created:
//    Configuration:        
//                                                                            
// ----------------------------------------------------------------------------

// Include the V5 Library
#include "vex.h"

// Allows for easier use of the VEX Library
using namespace vex;

// Begin project code

// Autonomous variables


void preAutonomous(void) {
  // actions to do when the program starts
  
}

// Used to tune the PID

// Lateral Movement PID
double kp = 0.00000; // tuning value for Potential (error)
double ki = 0.00000; // tuning value for Derivative (minor changes)
double kd = 0.00000; // tuning value for Integral (speed)


////////////////////////////////////////////
// Turning PID /////////////////////////////
////////////////////////////////////////////

double tp = 0.00000; // tuning value for Potential (error)
double ti = 0.00000; // tuning value for Derivative (minor changes)
double td = 0.00000; // tuning value for Integral (speed)

// ------------------------------------------------------


// Variables created for PID loop
int desiredValue = 200; // the number of degrees the robot turns.
int desiredTurnValue = 0; // the degrees value the robot turns

// Lateral Movement

int error; // SensorValue - DesiredValue or the distance between how far you travel or rotate and how far you want to travel or rotate.
int prevError = 0; // Position 20 milliseconds ago
int derivative;
int totalError = 0; // totalError = totalError + error

// Turning Movement

int turningerror;
int prevTurningError = 0;
int turningderivative;
int totalTurnError = 0;

// Variables that are changed for PID
bool enableDrivePID = true; // create a true/false variable that is only true during auton
bool resetDriveSensors = true;

// Variables created for motor spin in PID
float wheel_diameter = 4.0
int inchesToDegrees(float inches){
  double deg = inches * ((wheel_diameter*2*3.1415)/360); // also multiplies by gear ratio, but gear ratio is not measurable at this time.

  return deg;
}

int drivePID() {
  while(enableDrivePID /* This is equivalent to while(true) but only for when autonomous runs. */) { // Allows PID to run when autonomous starts.
    if(resetDriveSensors){
      resetDriveSensors = false;
      LeftMotor.setPosition(0, degrees);
      RightMotor.setPosition(0, degrees);
    }
    //////////////////////Lateral Movement PID//////////////////////////////////////////////////////////
    int leftmotorposition = LeftMotor.position(degrees); // how far the left side of the robot traveled
    int rightmotorposition = RightMotor.position(degrees); // how far the right side of the robot traveled
    
    int averageposition = (leftmotorposition + rightmotorposition)/2; // The average of the positions of both the motors


    // Potential

    error = averageposition - desiredValue; // The distance between how far you traveled and how far you want to travel.


    // Derivative

    derivative = error - prevError; // The speed (or instantaneous rate of change) 


    // Integral

    totalError += error; // Adds error to totalError if target is not reached yet
    double lateralmotorpower = (error*kp + derivative*kd + totalError*ki)/12; // Sets the motorPower needed to have the PID program work

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////// Turning Movement PID ///////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    
    int turnDifference = leftmotorposition - rightmotorposition; // The average of the positions of both the motors


    // Potential

    turningerror = turnDifference - desiredTurnValue; // The distance between how far you turned and how far you want to turn.


    // Derivative

    turningderivative = turningerror - prevTurningError; // The speed (or instantaneous rate of change) for turning 


    // Integral

    totalTurnError += turningerror; // Adds error to totalError if target is not reached yet
    double turningmotorpower = (turningerror*tp + turningderivative*td + totalTurnError*ti)/12; // Sets the motorPower needed to have the PID program work


    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // Spins the motors forward in # of degrees

    LeftMotor.spin(forward, lateralmotorpower + turningmotorpower, voltageUnits::volt);
    RightMotor.spin(forward, lateralmotorpower - turningmotorpower, voltageUnits::volt);

    prevTurningError = turningerror; // Error last loop becomes error this loop: only put right before loop reset so the Preverror always = error of previous loop.
    vex::task::sleep(20); // prevents an instance of an infinite loop by waiting for 20 milliseconds.
  }
  return 1; // returns the value of 1 so it could be referenced as a task.
}
// void move(desiredValue, desiredTurnValue) {
//   resetDriveSensors = true;
// }

void autonomous(void) {
  
  enableDrivePID = true;
  
  vex::task taskName(drivePID); // references drivePID as a task.


  // Example of PID
  // This example below moves the robot forward 123 inches and then 
  // turns the robot 90 degrees to the left
  resetDriveSensors = true; // Resets the drive sensors

  desiredValue = inchesToDegrees(123); // moves 123 inches forward
  vex::task::sleep(4000); // Waits for 4 seconds to prevent the brain for going too fast

  resetDriveSensors = true; // Must reset drive sensors for each movement in PID
  desiredTurnValue = 90; // turns 90 degrees to the left
  vex::task::sleep(40); // Waits for 40 milliseconds 

  
}

void userControl(void) {
  enableDrivePID = false;

  
}

int main() {
  // create competition instance
  competition Competition;

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  // Run the pre-autonomous function.
  preAutonomous();

  
}
