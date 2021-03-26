#include <Encoder.h>

Encoder knobLeft(2, 3);
Encoder knobRight(4, 5);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
}

long positionLeft  = -999;
long positionRight = -999;
long newLeft, newRight;
void loop() {
  update_EN();
//  long newLeft, newRight;
//  newLeft = knobLeft.read();
//  newRight = knobRight.read();
//  if (newLeft != positionLeft || newRight != positionRight) {
//    positionLeft = newLeft;
//    positionRight = newRight;
//  }
//  String content = String(newLeft) + "|" +String(newRight);
//  Serial.println(content);

}

void update_EN()
{
  //long newLeft, newRight;
  newLeft = knobLeft.read();
  newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) {
  positionLeft = newLeft;
  positionRight = newRight;
  }
  String content = String(newLeft) + "|" +String(newRight);
  Serial.println(content);
}
