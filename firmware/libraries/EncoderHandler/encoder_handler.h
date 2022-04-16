/*
 * Helper library to setup and read a quadrature encoder. Also determines the direction of rotation.
 * Author: Oscar Lima (olima_84@yahoo.com)
 *
 * See : https://www.youtube.com/watch?v=v4BbSzJ-hz4
 */

#ifndef encoder_handler_h
#define encoder_handler_h

#include <Arduino.h>

class EncoderHandler
{
  public:
    EncoderHandler(uint8_t hall_sensor_1, uint8_t hall_sensor_2);
    void encoder_state_change();
    long int get_encoder_count();
    void reset_encoder_count();

  private:
    uint8_t hall_sensor_1_;
    uint8_t hall_sensor_2_;
    long int encoder_count_;
};

#endif
