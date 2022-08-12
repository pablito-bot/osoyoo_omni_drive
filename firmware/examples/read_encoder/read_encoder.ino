/*
 * Example on how to read an encoder, using PinChangeInt library
*/
#include <PinChangeInt.h>

// Encoder count signal
#define hall_sensor_RR1 53 // RR
#define hall_sensor_RR2 51 // RR

#define hall_sensor_RL1 49 // RL
#define hall_sensor_RL2 47 // RL

#define hall_sensor_FR1 45 // FR
#define hall_sensor_FR2 41 // FR

#define hall_sensor_FL1 43 // FL
#define hall_sensor_FL2 39 // FL

volatile long int burp_left = 0;     // a counter to see how many times the pin has changed
volatile long int burp_right = 0;    // a counter to see how many times the pin has changed

void setup()
{
    pinMode(hall_sensor_RR1, INPUT);      // set the pin to input
    pinMode(hall_sensor_RR2, INPUT);     // set the pin to input

    digitalWrite(hall_sensor_RR1, HIGH);  // use the internal pullup resistor
    digitalWrite(hall_sensor_RR2, HIGH); // use the internal pullup resistor

    PCintPort::attachInterrupt(hall_sensor_RR1, burpcountleft, CHANGE);
    PCintPort::attachInterrupt(hall_sensor_RR2, burpcountright, CHANGE);

    Serial.begin(9600);
    delay(300);
}

void loop()
{
    Serial.println(burp_left);
    Serial.println(burp_right);
    delay(200);
}

void burpcountleft()
{
    burp_left++;
}

void burpcountright()
{
    burp_right++;
}
