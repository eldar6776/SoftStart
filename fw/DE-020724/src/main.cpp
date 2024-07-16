// Libraries
#include <RBDdimmer.h> //https://github.com/RobotDynOfficial/RBDDimmer
// Parameters
const int zeroCrossPin = 14;
const int acdPin = 13;
const int relayPin = 12;
const int onoffPin = 32;
int MIN_POWER = 0;
int MAX_POWER = 90;
int POWER_STEP = 1;
// Variables
int power = 0;
bool on = false;
// Objects
dimmerLamp acd(acdPin, zeroCrossPin);
void testDimmer();
void setup() {
  // Init Serial USB
  Serial.begin(115200);
  Serial.println(F("ESP32 System"));
  acd.begin(NORMAL_MODE, ON);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  pinMode(onoffPin, INPUT_PULLUP);
  acd.setPower(50); // setPower(0-100%);
}
void printSpace(int val) {
  if ((val / 100) == 0)
    Serial.print(" ");
  if ((val / 10) == 0)
    Serial.print(" ");
}
void loop() {
  // testDimmer();
  int preVal = power;

  if (Serial.available()) {
    int buf = Serial.parseInt();
    if (buf != 0)
      power = buf;
    delay(200);
  }
  acd.setPower(power); // setPower(0-100%);

  if (preVal != power) {
    Serial.print("lampValue -> ");
    printSpace(acd.getPower());
    Serial.print(acd.getPower());
    Serial.println("%");
  }
  delay(50);
}
void testDimmer() { /* function testDimmer */
  ////Sweep light power to test dimmer
  if ((on == false) && (digitalRead(onoffPin) == LOW)) {
    acd.setState(ON);
    for (power = MIN_POWER; power <= MAX_POWER; power += POWER_STEP) {
      acd.setPower(power); // setPower(0-100%);
      Serial.print("lampValue -> ");
      Serial.print(acd.getPower());
      Serial.println("%");
      delay(50);
    }
    digitalWrite(relayPin, HIGH);
    on = true;
  }

  if ((on == true) && (digitalRead(onoffPin) == LOW)) {
    digitalWrite(relayPin, LOW);
    for (power = MAX_POWER; power >= MIN_POWER; power -= POWER_STEP) {
      acd.setPower(power); // setPower(0-100%);
      Serial.print("lampValue -> ");
      Serial.print(acd.getPower());
      Serial.println("%");
      delay(50);
    }
    acd.setState(OFF);
    on = false;
  }
}