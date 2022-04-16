/*
 * Helper library to setup and read a quadrature encoder. Also determines the direction of rotation.
 * Author: Oscar Lima (olima_84@yahoo.com)
 *
 * See : https://www.youtube.com/watch?v=v4BbSzJ-hz4
 */

#include "encoder_handler.h"

EncoderHandler::EncoderHandler(uint8_t hall_sensor_1, uint8_t hall_sensor_2)
{
    encoder_count_ = 0;
    hall_sensor_1_ = hall_sensor_1;
    hall_sensor_2_ = hall_sensor_2;

    // encoder pin setup
    pinMode(hall_sensor_1_, INPUT); // set pin to be used as input
    pinMode(hall_sensor_2_, INPUT); // set pin to be used as input
    digitalWrite(hall_sensor_1_, HIGH); // use the internal pullup resistor
    digitalWrite(hall_sensor_2_, HIGH); // use the internal pullup resistor
}

void EncoderHandler::encoder_state_change()
{
  /*   clockwise motor rotation
   *
   *        0   1
   *    ___     ___
   *  _|   |___|   |___  Hall sensor 1
   *
   *        1   0
   *      ___     ___
   *  ___|   |___|   |__ Hall sensor 2
   *
   *  Hall sensors are 90° out of phase
   *
   *  NOTE: Observe Hall sensor 1 change interrupt, either RISING (0 -> 1) or FALLING (1 -> 0)
   *  shortly after the event: both sensors have *different* value.
   *
   *  ---------------------------------
   *
   *   counter-clockwise motor rotation
   *
   *        1   0
   *        ___     ___
   *    ___|   |___|   |___  Hall sensor 1
   *
   *        1   0
   *      ___     ___
   *  ___|   |___|   |___    Hall sensor 2
   *
   *  Hall sensors are 90° out of phase
   *
   *  NOTE: Observe Hall sensor 1 change interrupt, either RISING (0 -> 1) or FALLING (1 -> 0)
   *  shortly after the event: both sensors have the *same* value.
   *
   */

  if(digitalRead(hall_sensor_1_) != digitalRead(hall_sensor_2_))
      encoder_count_++;
  else
      encoder_count_--;
}

long int EncoderHandler::get_encoder_count()
{
    return encoder_count_;
}
