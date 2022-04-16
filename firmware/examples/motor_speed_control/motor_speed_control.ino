/*
 * A motor speed controller example
 */

/******** INCLUDE *********/

#include <SparkFun_TB6612.h> // TB6612FNG (motor driver / H-bridge chip) library to control the motors

/* There are only two external interrupt pins, INT0 and INT1, and they are mapped to Arduino pins 2 and 3,
 * however on the Osoyoo shield the encoders are wired to pins 2 and 4. Therefore the use of a special library
 * to handle interruptions on any desired pin is needed.
 * According to their documentation the PinChangeInt library can handle the Arduino's pin change interrupts
 * as quickly, easily, and reasonably as possible.
 */

#include <PinChangeInt.h>
#include <PID_v1.h>                  // PID based control library to achieve motor speed setpoint as fast and smooth as possible

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

#define pulses_per_revolution 450

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

// measure elapsed time
unsigned long last_time;

// PID

// PID documentation available under:
//    http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

double setpoint = 2.0;
double speed_sensor = 0.0;
double double_pid_output = 0.0;

// Specify the links and initial tuning parameters
double kp = 0.045;
double ki = 150.0;
double kd = 0.1;
int ctrl_delay = 49;

PID myPID(&speed_sensor, &double_pid_output, &setpoint, kp, ki, kd, DIRECT);

/******** SETUP *********/

void setup()
{
    // encoder pin configuration
    pinMode(hall_sensor_RR1, INPUT);
    pinMode(hall_sensor_RL1, INPUT);
    digitalWrite(hall_sensor_RR1, HIGH); //use the internal pullup resistor
    digitalWrite(hall_sensor_RL1, HIGH); //use the internal pullup resistor
    PCintPort::attachInterrupt(hall_sensor_RR1, interruptCountRR, CHANGE);
    PCintPort::attachInterrupt(hall_sensor_RL1, interruptCountRL, CHANGE);

    // initialize serial port at 9600 baud rate
    Serial.begin(9600);

    // initialize variable to measure speed
    last_time = millis();

    // PID controller

    // limit the output of the PID controller between 0 and 255
    myPID.SetOutputLimits(0, 255);

    // turn the PID on and rise flag for "auto" mode
    myPID.SetMode(AUTOMATIC);

    // sets the period, in Milliseconds, at which the calculation is performed
    myPID.SetSampleTime(ctrl_delay);

    // not sure if this dealy is really needed
    delay(300);
    Serial.println("setup complete");
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

float measureSpeed()
{
    float delta_pulses_right = float(count_rr);

    unsigned long current_time = millis();

    float delta_time = float(current_time - last_time);

    // get time for next cycle
    last_time = current_time;

    // reset count
    count_rr = 0;

    /*  speed conversion from : [pulses] / [ms]    to    [rad] / [sec]
     *  
     *  delta_pulses_right [pulses]    1000 [ms]               2 Pi [rad]
     *  -------------------------- * ----------- * ---------------------------------
     *       delta_time [ms]           1 [sec]       pulses_per_revolution [pulses]
     */

    // return value is in : rad/sec
    return (delta_pulses_right * 1000.0 * 2 * 3.14159265359) / ( delta_time * pulses_per_revolution);
}

/******** LOOP *********/

void loop()
{
    // update PID input
    speed_sensor = double(measureSpeed());

    // calculate required PWM to match the desired speed
    myPID.Compute();

    // cast PID double precision floating point output to int
    int int_pid_output = int(double_pid_output);

    // send PID output to motor
    motor_rr.drive(int_pid_output);

    // debug info (uncomment at will)
    // Serial.println("setpoint :");
    // Serial.println(setpoint);
    // Serial.println("speed_sensor :");
    Serial.println(speed_sensor);
    // Serial.println("PID output :");
    // Serial.println(int_pid_output);

    delay(ctrl_delay);
}
