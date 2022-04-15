/*
 * Example on how to move the osoyoo motors using SparkFun_TB6612 library
 */

/******** INCLUDE *********/

#include <SparkFun_TB6612.h> // TB6612FNG (motor driver / H-bridge chip) library to control the motors

/******** DEFINE *********/

/*
 *   forward
 * MFL -- MFR
 *  |      |
 *  |      |
 * MRL -- MRR
 *    back
 */

#define IN1_MRR 8   // MRR: Motor rear right
#define IN2_MRR 10
#define PWM_MRR 9   // ENA: enable/disable H bridge (used as PWM to control motor speed)

#define IN1_MRL 12  // MRL: Motor rear left
#define IN2_MRL 13
#define PWM_MRL 11

#define IN1_MFR 7   // MFR: Motor front right
#define IN2_MFR 5
#define PWM_MFR 6

#define IN1_MFL 4   // MFL: Motor front left
#define IN2_MFL 2
#define PWM_MFL 3

/******** GLOBALS *********/

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int motor_offset_rr = 1;
const int motor_offset_rl = 1;
const int motor_offset_fr = 1;
const int motor_offset_fl = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor_rr = Motor(IN1_MRR, IN2_MRR, PWM_MRR, motor_offset_rr);
Motor motor_rl = Motor(IN1_MRL, IN2_MRL, PWM_MRL, motor_offset_rl);
Motor motor_fr = Motor(IN1_MFR, IN2_MFR, PWM_MFR, motor_offset_fr);
Motor motor_fl = Motor(IN1_MFL, IN2_MFL, PWM_MFL, motor_offset_fl);

bool one_time_only = true;

/******** SETUP *********/

void setup()
{
    // Nothing here
    // PIN setup is done in Motor constructor
}

/******** LOOP *********/

void loop()
{
  if (one_time_only)
  {
    motor_rr.drive(128,1000);
    motor_rr.drive(-128,1000);
    motor_rr.brake();
    delay(1000);

    motor_rl.drive(128,1000);
    motor_rl.drive(-128,1000);
    motor_rl.brake();
    delay(1000);

    motor_fr.drive(128,1000);
    motor_fr.drive(-128,1000);
    motor_fr.brake();
    delay(1000);

    motor_fl.drive(128,1000);
    motor_fl.drive(-128,1000);
    motor_fl.brake();

    one_time_only = false;
  }
  else
  {
    delay(1000);
  }
}
