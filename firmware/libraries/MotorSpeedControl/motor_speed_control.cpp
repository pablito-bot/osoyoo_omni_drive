/*
 * A motor speed controller library
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 */

#include "motor_speed_control.h"

MotorSpeedCtrl::MotorSpeedCtrl(uint8_t motor_dir_1, uint8_t motor_dir_2, uint8_t motor_pwm, const int motor_offset,
                               uint8_t hall_sensor_1, uint8_t hall_sensor_2, int pulses_per_revolution,
                               double kp, double ki, double kd, int ctrl_delay, double pid_output_lower_bound, double pid_output_upper_bound)
{
    motor_ = std::make_shared<Motor>(motor_dir_1, motor_dir_2, motor_pwm, motor_offset);
    encoder_ = std::make_shared<EncoderHandler>(hall_sensor_1, hall_sensor_2);
    pulses_per_revolution_ = pulses_per_revolution;

    // PID, documentation available under:
    //      http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
    sensed_motor_speed_ = 0.0;
    double_pid_output_ = 0.0;
    motor_speed_setpoint_ = 0.0;
    ctrl_delay_ = ctrl_delay;
    int_pid_output_ = 0;

    pid_ = std::make_shared<PID>(&sensed_motor_speed_, &double_pid_output_, &motor_speed_setpoint_, kp, ki, kd, DIRECT);
    // limit the output of the PID controller between desired bounds
    pid_->SetOutputLimits(pid_output_lower_bound, pid_output_upper_bound);
    // turn the PID on and rise flag for "auto" mode
    pid_->SetMode(AUTOMATIC);
    // sets the period, in Milliseconds, at which the calculation is performed
    pid_->SetSampleTime(ctrl_delay_);

    // initialize variable to measure speed
    last_time_ = millis();
}

void MotorSpeedCtrl::encoder_state_change()
{
    encoder_->encoder_state_change();
}

double MotorSpeedCtrl::set_motor_speed(double desired_motor_speed)
{
    this->motor_speed_setpoint_ = desired_motor_speed;
}

double MotorSpeedCtrl::get_motor_speed()
{
    return this->sensed_motor_speed_;
}

int MotorSpeedCtrl::get_pid_output()
{
    return this->int_pid_output_;
}

void MotorSpeedCtrl::brake()
{
    motor_->brake();
}

double MotorSpeedCtrl::measure_speed()
{
    float delta_pulses = float(encoder_->get_encoder_count());
    unsigned long current_time = millis();
    float delta_time = float(current_time - last_time_);

    // save time data for next cycle
    last_time_ = current_time;

    // reset count
    encoder_->reset_encoder_count();

    /*  speed conversion from : [pulses] / [ms]    to    [rad] / [sec]
    *  
    *  delta_pulses_right [pulses]    1000 [ms]               2 Pi [rad]
    *  --------------------------- * ----------- * ---------------------------------
    *       delta_time [ms]             1 [sec]     pulses_per_revolution [pulses]
    */

    // return value is in : rad/sec
    // 1000 * 2 * pi = 6.2831853071
    sensed_motor_speed_ = double((delta_pulses * 6.2831853071) / ( delta_time * pulses_per_revolution_));
    Serial.println(sensed_motor_speed_);
}

void MotorSpeedCtrl::run_ctrl_loop()
{
    // update PID input
    this->measure_speed();
    // calculate required PWM to match the desired speed
    pid_->Compute();
    // cast PID double precision floating point output to int
    int_pid_output_ = int(double_pid_output_);
    // send PID output to motor
    motor_->drive(int_pid_output_);
}
