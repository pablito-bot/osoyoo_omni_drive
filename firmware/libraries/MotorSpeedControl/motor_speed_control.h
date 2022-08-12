/*
 * A motor speed controller library
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 */

#ifndef MOTOR_SPEED_CONTROL_h
#define MOTOR_SPEED_CONTROL_h

#include <SparkFun_TB6612.h> // TB6612FNG (motor driver / H-bridge chip) library to control the motors, has been adapted to our h bridge chip (L298N)
#include <PID_v1.h> // PID based control library to achieve motor speed setpoint as fast and smooth as possible
#include <encoder_handler.h> // helper encoder library
#include <ArxSmartPtr.h> // smart pointer library for arduino

class MotorSpeedCtrl
{
  public:
    MotorSpeedCtrl(uint8_t motor_dir_1, uint8_t motor_dir_2, uint8_t motor_pwm, const int motor_offset,
                   uint8_t hall_sensor_1, uint8_t hall_sensor_2, int pulses_per_revolution,
                   double kp, double ki, double kd, int ctrl_delay, double pid_output_lower_bound, double pid_output_upper_bound);

    void encoder_state_change();
    double set_motor_speed(double desired_motor_speed);
    double get_motor_speed();
    int get_pid_output();
    void brake();
    double measure_speed();
    void run_ctrl_loop();

  private:
    std::shared_ptr<Motor> motor_;
    std::shared_ptr<EncoderHandler> encoder_;
    std::shared_ptr<PID> pid_;

    int pulses_per_revolution_;
    int int_pid_output_;
    double motor_speed_setpoint_;
    double double_pid_output_;
    double sensed_motor_speed_;

    unsigned long last_time_;
    int ctrl_delay_;
};

#endif
