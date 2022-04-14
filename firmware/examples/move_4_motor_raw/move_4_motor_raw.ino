/*
 * Example on how to move a motor with L298N osoyoo motor driver and custom arduino interface hat
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

void setup()
{
    pinMode(IN1_MRR, OUTPUT);
    pinMode(IN2_MRR, OUTPUT);
    pinMode(PWM_MRR, OUTPUT);

    pinMode(IN1_MRL, OUTPUT);
    pinMode(IN2_MRL, OUTPUT);
    pinMode(PWM_MRL, OUTPUT);

    // Initialize the motor driver
    digitalWrite(IN1_MRR, 0);
    digitalWrite(IN2_MRR, 1);
    analogWrite(PWM_MRR, 0);

    digitalWrite(IN1_MRL, 0);
    digitalWrite(IN2_MRL, 1);
    analogWrite(PWM_MRL, 0);

    digitalWrite(IN1_MFR, 0);
    digitalWrite(IN2_MFR, 1);
    analogWrite(PWM_MFR, 0);

    digitalWrite(IN1_MFL, 0);
    digitalWrite(IN2_MFL, 1);
    analogWrite(PWM_MFL, 0);

    delay(300);
}

void move_forward(int pwm_value)
{
  digitalWrite(IN1_MRR, 0);
  digitalWrite(IN2_MRR, 1);
  analogWrite(PWM_MRR, pwm_value);

  digitalWrite(IN1_MRL, 0);
  digitalWrite(IN2_MRL, 1);
  analogWrite(PWM_MRL, pwm_value);

  digitalWrite(IN1_MFR, 0);
  digitalWrite(IN2_MFR, 1);
  analogWrite(PWM_MFR, pwm_value);

  digitalWrite(IN1_MFL, 0);
  digitalWrite(IN2_MFL, 1);
  analogWrite(PWM_MFL, pwm_value);
}

void move_backwards(int pwm_value)
{
  digitalWrite(IN1_MRR, 1);
  digitalWrite(IN2_MRR, 0);
  analogWrite(PWM_MRR, pwm_value);

  digitalWrite(IN1_MRL, 1);
  digitalWrite(IN2_MRL, 0);
  analogWrite(PWM_MRL, pwm_value);

  digitalWrite(IN1_MFR, 1);
  digitalWrite(IN2_MFR, 0);
  analogWrite(PWM_MFR, pwm_value);

  digitalWrite(IN1_MFL, 1);
  digitalWrite(IN2_MFL, 0);
  analogWrite(PWM_MFL, pwm_value);
}

void move_right(int pwm_value)
{
  // see: https://osoyoo.com/manual/V1.1metal-mecanum.pdf

  digitalWrite(IN1_MRR, 0); // forward
  digitalWrite(IN2_MRR, 1);
  analogWrite(PWM_MRR, pwm_value);

  digitalWrite(IN1_MRL, 1); // backwards
  digitalWrite(IN2_MRL, 0);
  analogWrite(PWM_MRL, pwm_value);

  digitalWrite(IN1_MFR, 1); // backwards
  digitalWrite(IN2_MFR, 0);
  analogWrite(PWM_MFR, pwm_value);

  digitalWrite(IN1_MFL, 0); // forward
  digitalWrite(IN2_MFL, 1);
  analogWrite(PWM_MFL, pwm_value);
}

void move_left(int pwm_value)
{
  // see: https://osoyoo.com/manual/V1.1metal-mecanum.pdf

  digitalWrite(IN1_MRR, 1); // backwards
  digitalWrite(IN2_MRR, 0);
  analogWrite(PWM_MRR, pwm_value);

  digitalWrite(IN1_MRL, 0); // forward
  digitalWrite(IN2_MRL, 1);
  analogWrite(PWM_MRL, pwm_value);

  digitalWrite(IN1_MFR, 0); // forward
  digitalWrite(IN2_MFR, 1);
  analogWrite(PWM_MFR, pwm_value);

  digitalWrite(IN1_MFL, 1); // backwards
  digitalWrite(IN2_MFL, 0);
  analogWrite(PWM_MFL, pwm_value);
}

void loop()
{
  // make a square motion
  int arbitrary_delay = 1500;
  int weak_pwm_value = 100;
  int strong_pwm_value = 150;

  move_forward(weak_pwm_value);
  delay(arbitrary_delay);

  move_right(strong_pwm_value);
  delay(arbitrary_delay);

  move_backwards(weak_pwm_value);
  delay(arbitrary_delay);

  move_left(strong_pwm_value);
  delay(arbitrary_delay);
}
