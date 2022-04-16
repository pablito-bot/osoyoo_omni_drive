/*
 * A program to help determine how many pulses per revolution our motor has
 * open the serial port terminal, run the program, adjust the time the motor runs
 * the encoder value is printed once the motor stops. Is probably better to really brake
 * the motor (by sending 1 and 1 to both the control signals and setting PWM value to 0).
 * 
 */

/******** INCLUDE *********/

// TB6612FNG (motor driver / H-bridge chip) library to control the motors
#include <SparkFun_TB6612.h>

// PinChangeInt library can handle the Arduino's pin change interrupts as quickly, easily, and reasonably as possible.
#include <PinChangeInt.h>

/******** DEFINE *********/

/* motors
 *
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

// encoders

#define hall_sensor_RR1 A14 // RR
#define hall_sensor_RR2 A15 // RR

#define hall_sensor_RL1 A13 // RL
#define hall_sensor_RL2 A12 // RL

#define hall_sensor_FR1 A9  // FR
#define hall_sensor_FR2 A11 // FR

#define hall_sensor_FL1 A8  // FL
#define hall_sensor_FL2 A10 // FL

/******** GLOBALS *********/

// motors

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

// encoder tick counters
volatile unsigned int count_rr = 0;
volatile unsigned int count_rl = 0;
volatile unsigned int count_fr = 0;
volatile unsigned int count_fl = 0;

/******** SETUP *********/

void setup()
{
    // encoder pin configuration
    pinMode(hall_sensor_RR1, INPUT);
    pinMode(hall_sensor_RL1, INPUT);
    digitalWrite(hall_sensor_RR1, HIGH); // use the internal pullup resistor
    digitalWrite(hall_sensor_RL1, HIGH); // use the internal pullup resistor
    PCintPort::attachInterrupt(hall_sensor_RR1, interruptCountRR, CHANGE);
    PCintPort::attachInterrupt(hall_sensor_RL1, interruptCountRL, CHANGE);

    // initialize serial port at 9600 baud rate
    Serial.begin(9600);
    delay(300);
}

void interruptCountRR()
{
    count_rr++;
}

void interruptCountRL()
{
    count_rl++;
}

//void interruptCountFR()
//{
//    count_fr++;
//}
//
//void interruptCountFL()
//{
//    count_fl++;
//}

/******** LOOP *********/

void loop()
{
  if(count_rl > 397)
  {
    motor_rl.brake();
    delay(1000);
    Serial.println(count_rl); // 450!
    count_rl = 0;
    delay(100);
  }
  else
  {
    motor_rl.drive(80);
  }
}
