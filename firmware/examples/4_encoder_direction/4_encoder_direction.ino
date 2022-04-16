/*
 * Example on how to use the library encoder_handler.h to setup and read 4 encoders
 * 
 */

#include <encoder_handler.h>
#include <PinChangeInt.h> // library to handle encoder interruptions

/* encoder pin definition, each motor has 2 hall sensors to detect rotation direction (CW or CCW)
 *  
 *   forward
 *  FL -- FR  (front left, front right)
 *  |      |
 *  |      |
 *  RL -- RR  (rear left, rear right)
 *    back
 */

#define hall_sensor_RR1 A14
#define hall_sensor_RR2 A15

#define hall_sensor_RL1 A13
#define hall_sensor_RL2 A12

#define hall_sensor_FR1 A9
#define hall_sensor_FR2 A11

#define hall_sensor_FL1 A8
#define hall_sensor_FL2 A10

// NOTE: most encoder pin setup is done in constructor
EncoderHandler encoder_rr = EncoderHandler(hall_sensor_RR1, hall_sensor_RR2);
EncoderHandler encoder_rl = EncoderHandler(hall_sensor_RL1, hall_sensor_RL2);
EncoderHandler encoder_fr = EncoderHandler(hall_sensor_FR1, hall_sensor_FR2);
EncoderHandler encoder_fl = EncoderHandler(hall_sensor_FL1, hall_sensor_FL2);

void setup()
{
  // setup interrupt callback function, gets executed upon pin state change
  auto rr1_pin_change = [] () { encoder_rr.encoder_state_change(); }; // C++ 11 lambda functions
  auto rl1_pin_change = [] () { encoder_rl.encoder_state_change(); };
  auto fr1_pin_change = [] () { encoder_fr.encoder_state_change(); };
  auto fl1_pin_change = [] () { encoder_fl.encoder_state_change(); };
  PCintPort::attachInterrupt(hall_sensor_RR1, rr1_pin_change, CHANGE);
  PCintPort::attachInterrupt(hall_sensor_RL1, rl1_pin_change, CHANGE);
  PCintPort::attachInterrupt(hall_sensor_FR1, fr1_pin_change, CHANGE);
  PCintPort::attachInterrupt(hall_sensor_FL1, fl1_pin_change, CHANGE);

  Serial.begin(9600);
  delay(300);
}

void loop()
{
  Serial.println(encoder_rr.get_encoder_count());
  Serial.println(encoder_rl.get_encoder_count());
  Serial.println(encoder_fr.get_encoder_count());
  Serial.println(encoder_fl.get_encoder_count());
  Serial.println("-----");
  delay(200);
}
