/*
 * Small program to test the encoders hardware connection
*/

// motor encoders

#define hall_sensor_RR1 53 // RR
#define hall_sensor_RR2 51 // RR

#define hall_sensor_RL1 49 // RL
#define hall_sensor_RL2 47 // RL

#define hall_sensor_FR1 45 // FR
#define hall_sensor_FR2 41 // FR

#define hall_sensor_FL1 43 // FL
#define hall_sensor_FL2 39 // FL

void setup()
{
  pinMode(hall_sensor_FL2, INPUT);      // set the pin to input
  digitalWrite(hall_sensor_FL2, HIGH);  // use the internal pullup resistor

  Serial.begin(9600);
  delay(300);
}

void loop()
{
  Serial.println(digitalRead(hall_sensor_FL2));
  delay(200);
}
