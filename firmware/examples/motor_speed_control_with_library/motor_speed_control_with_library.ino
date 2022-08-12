/*
 * A motor speed controller example for 4 motors at the same time using custom motor speed ctrl library
 * 
 */

/******** INCLUDE *********/

#include <motor_speed_control.h> // speed control library
#include <PinChangeInt.h> // library to handle arduino pin interrupts (used for motor encoder)

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

#define hall_sensor_RR1 A14
#define hall_sensor_RR2 A15

#define hall_sensor_RL1 A13
#define hall_sensor_RL2 A12

#define hall_sensor_FR1 A9
#define hall_sensor_FR2 A11

#define hall_sensor_FL1 A8
#define hall_sensor_FL2 A10

#define pulses_per_revolution 450

/******** GLOBALS *********/

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int motor_offset_rr = -1;
const int motor_offset_rl = -1;
const int motor_offset_fr = -1;
const int motor_offset_fl = -1;

// PID control parameters
double kp = 50.0;
double ki = 300.0;
double kd = 1.25;

int ctrl_delay = 49;

double pid_output_lower_bound = 0.0;
double pid_output_upper_bound = 255.0;

MotorSpeedCtrl motor_rr = MotorSpeedCtrl(IN1_MRR, IN2_MRR, PWM_MRR, motor_offset_rr,
                                         hall_sensor_RR1, hall_sensor_RR2, pulses_per_revolution,
                                         kp, ki, kd, ctrl_delay, pid_output_lower_bound, pid_output_upper_bound);

MotorSpeedCtrl motor_rl = MotorSpeedCtrl(IN1_MRL, IN2_MRL, PWM_MRL, motor_offset_rl,
                                         hall_sensor_RL1, hall_sensor_RL2, pulses_per_revolution,
                                         kp, ki, kd, ctrl_delay, pid_output_lower_bound, pid_output_upper_bound);

MotorSpeedCtrl motor_fr = MotorSpeedCtrl(IN1_MFR, IN2_MFR, PWM_MFR, motor_offset_fr,
                                         hall_sensor_FR1, hall_sensor_FR2, pulses_per_revolution,
                                         kp, ki, kd, ctrl_delay, pid_output_lower_bound, pid_output_upper_bound);

MotorSpeedCtrl motor_fl = MotorSpeedCtrl(IN1_MFL, IN2_MFL, PWM_MFL, motor_offset_fl,
                                         hall_sensor_FL1, hall_sensor_FL2, pulses_per_revolution,
                                         kp, ki, kd, ctrl_delay, pid_output_lower_bound, pid_output_upper_bound);

/******** SETUP *********/

void setup()
{
  // encoder pin configuration
  // setup interrupt callback function, gets executed upon pin state change
  auto motor_rr_encoder_state_change = [] () { motor_rr.encoder_state_change(); }; // C++ 11 lambda function
  PCintPort::attachInterrupt(hall_sensor_RR1,  motor_rr_encoder_state_change, CHANGE);
  auto motor_rl_encoder_state_change = [] () { motor_rl.encoder_state_change(); };
  PCintPort::attachInterrupt(hall_sensor_RL1,  motor_rl_encoder_state_change, CHANGE);
  auto motor_fr_encoder_state_change = [] () { motor_fr.encoder_state_change(); };
  PCintPort::attachInterrupt(hall_sensor_FR1,  motor_fr_encoder_state_change, CHANGE);
  auto motor_fl_encoder_state_change = [] () { motor_fl.encoder_state_change(); };
  PCintPort::attachInterrupt(hall_sensor_FL1,  motor_fl_encoder_state_change, CHANGE);

  // test an arbitrary motor speed setpoint
  motor_rr.set_motor_speed(0.0);
  motor_rl.set_motor_speed(0.5);
  motor_fr.set_motor_speed(0.0);
  motor_fl.set_motor_speed(0.0);

  // initialize serial port at 9600 baud rate
  Serial.begin(9600);

  // allow some small time for setup function to stabilize
  delay(100);
}

/******** LOOP *********/

void loop()
{
  motor_rr.run_ctrl_loop();
  motor_rl.run_ctrl_loop();
  motor_fr.run_ctrl_loop();
  motor_fl.run_ctrl_loop();

  // debug info (uncomment at will)
  // Serial.println("setpoint :");
  // Serial.println(setpoint);
  // Serial.println("rr_speed_sensor :");
  Serial.println(motor_rl.get_motor_speed());
  // Serial.println("rr PID output :");
  // Serial.println(int_rr_pid_output);

  delay(ctrl_delay);
}
