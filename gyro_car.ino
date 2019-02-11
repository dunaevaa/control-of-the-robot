#include <Wire.h>
#include <IRremote.h>
#include "motor.h"
#include "gyro_acsel.h"

#define ANGLE10 0x20DF8877
#define ANGLE20 0x20DF48B7
#define ANGLE30 0x20DFC837
#define ANGLE40 0x20DF28D7
#define ANGLE50 0x20DFA857
#define ANGLE60 0x20DF6897
//#define ANGLE70 0x20DFE817
//#define ANGLE80 0x20DF18E7
//#define ANGLE90 0x20DF9867
#define DOWN 0x20DFC03F
#define UP 0x20DF40BF
#define FORWARD 0x20DF02FD
#define LEFT 0x20DFE01F
#define RIGHT 0x20DF609F
#define STOP 0x20DF22DD
#define BACKWARD 0x20DF827D
#define LINE 0x20DF08F7

int RECV_PIN = 9;
IRrecv irrecv(RECV_PIN);
decode_results results;
int diod_pin = 6;

void setup()
{
  giroscop_setup();
  setup_motor_system(2, 3, 4, 5);
  _stop();
  Calc_CompensatorZ(3000);
  pinMode(diod_pin, OUTPUT);
  irrecv.enableIRIn();
  Serial.begin(9600);
}

void loop()
{
  if (irrecv.decode(&results))
  {
    Serial.println(results.value, HEX);
    switch (results.value)
    {
      case ANGLE10:
        digitalWrite(diod_pin, 1);
        Angle(10);
        digitalWrite(diod_pin, 0);
        break;
      case ANGLE20:
        digitalWrite(diod_pin, 1);
        Angle(20);
        digitalWrite(diod_pin, 0);
        break;
      case ANGLE30:
        digitalWrite(diod_pin, 1);
        Angle(30);
        digitalWrite(diod_pin, 0);
        break;
      case ANGLE40:
        digitalWrite(diod_pin, 1);
        Angle(40);
        digitalWrite(diod_pin, 0);
        break;
      case ANGLE50:
        digitalWrite(diod_pin, 1);
        Angle(50);
        digitalWrite(diod_pin, 0);
        break;
      case ANGLE60:
        digitalWrite(diod_pin, 1);
        Angle(60);
        digitalWrite(diod_pin, 0);
        break;
      case UP:
        digitalWrite(diod_pin, 1);
        forward_t(10000000);
        digitalWrite(diod_pin, 0);
        break;
      case DOWN:
        digitalWrite(diod_pin, 1);
        backward_t(10000000);
        digitalWrite(diod_pin, 0);
        break;
      case RIGHT:
        digitalWrite(diod_pin, 1);
        Angle_t(100000000);
        digitalWrite(diod_pin, 0);
        break;
    }
    irrecv.resume();
  }
  time_gyro(0.5);
}
