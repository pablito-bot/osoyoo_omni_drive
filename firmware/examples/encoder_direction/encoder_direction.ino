/*
 * Example on how to read an encoder but taking into account the direction of rotation
 * Clock Wise (CW) or Counter Clock Wise (CCW). This is done based on phase shift.
 * See : https://www.youtube.com/watch?v=v4BbSzJ-hz4
*/

#include <PinChangeInt.h>

// each motor has 2 hall sensors to detect rotation direction (CW or CCW)

#define hall_sensor_1 A14 // RR
#define hall_sensor_2 A15 // RR

/*

#define hall_sensor_RR1 A14 // RR
#define hall_sensor_RR2 A15 // RR

#define hall_sensor_RL1 A13 // RL
#define hall_sensor_RL2 A12 // RL

#define hall_sensor_FR1 A9  // FR
#define hall_sensor_FR2 A11 // FR

#define hall_sensor_FL1 A8  // FL
#define hall_sensor_FL2 A10 // FL

*/

// to keep count of the encoder pulses
volatile long int count = 0;

void setup()
{
  pinMode(hall_sensor_1, INPUT);      // set the pin to input
  pinMode(hall_sensor_2, INPUT);      // set the pin to input

  digitalWrite(hall_sensor_1, HIGH);  // use the internal pullup resistor
  digitalWrite(hall_sensor_2, HIGH);  // use the internal pullup resistor

  PCintPort::attachInterrupt(hall_sensor_1, hall_sensor_1_state_change, CHANGE);

  Serial.begin(9600);
  delay(300);
}

void hall_sensor_1_state_change()
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
   *  shortly after the event both sensors have *different* value.
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
   *  shortly after the event both sensors have the *same* value.
   *
   */

  if(digitalRead(hall_sensor_1) != digitalRead(hall_sensor_2))
  {
    count++;
  }
  else
  {
    count--;
  }
}

void loop()
{
  Serial.println(count);
  delay(200);
}
