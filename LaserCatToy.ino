/** source:
    https://github.com/TrashRobotics/LaserCatToy/blob/main/laser_toy/laser_toy_en.ino
*/

#include <Servo.h>
#include <FastLED.h>

#define FIRST_LINK_SERVO_PIN  9   // pin of the first link servo
#define SECOND_LINK_SERVO_PIN 10  // pin of the second link servo
#define LASER_PIN             11  // laser pin
#define RESET_BUTTON 5
#define GREEN_LED 2


#define BASE_FIRST_LINK_SERVO_POS   90  // base position of servos (in degrees)
#define BASE_SECOND_LINK_SERVO_POS  10

#define MIN_FIRST_LINK_SERVO_POS    60  // minimum angle that the first servo can take
#define MAX_FIRST_LINK_SERVO_POS    120 // maximum angle that the first servo can take

#define MIN_SECOND_LINK_SERVO_POS    2  // minimum angle that the second servo can take
#define MAX_SECOND_LINK_SERVO_POS    45  // maximum angle that the second servo can take

#define LASER_BRIGHTNESS  200 // яlaser brightness
#define SERVO_DELAY       6
#define seconds() (millis()/1000)
#define THREE_MIN 3*60
bool laser_on;
Servo firstLinkServo;   // first link servo
Servo secondLinkServo;  // second link servo
void sanity_check(int sec_hold = 3);
void idle_pos();
int angle_diff = 10;
int start_tic;

uint8_t fsp_v=BASE_FIRST_LINK_SERVO_POS;
uint8_t ssp=BASE_SECOND_LINK_SERVO_POS;
#define ANGLE_BIAS 15

void setup() {
  Serial.begin(9600);
  pinMode(LASER_PIN, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  pinMode(RESET_BUTTON, INPUT);

  analogWrite(LASER_PIN, LOW);
  laser_on = false;
  digitalWrite(GREEN_LED, LOW);

  firstLinkServo.attach(FIRST_LINK_SERVO_PIN);      // bind drives to ports
  secondLinkServo.attach(SECOND_LINK_SERVO_PIN);    //

  firstLinkServo.write(BASE_FIRST_LINK_SERVO_POS);  // set to base position
  delay(200);
  secondLinkServo.write(BASE_SECOND_LINK_SERVO_POS); // set to base position
  delay(200);
  Serial.println("ZERO");
  digitalWrite(GREEN_LED, HIGH);

  secondLinkServo.write(BASE_SECOND_LINK_SERVO_POS);
  delay(5000);
  sanity_check(2);

  Serial.println("LET's GO");
  analogWrite(LASER_PIN, LASER_BRIGHTNESS);  // start the laser
  laser_on = true;
  digitalWrite(GREEN_LED, LOW);

  start_tic = seconds();
}


void loop() {
  EVERY_N_SECONDS(10) {
    int tic_tac = THREE_MIN - (seconds() - start_tic);
    if (tic_tac >= 0) {
      Serial.print(tic_tac);
      Serial.println("  left");
    }
  }
  if ((seconds() - start_tic) < THREE_MIN) {


    //uint8_t fsp = random(MIN_FIRST_LINK_SERVO_POS, MAX_FIRST_LINK_SERVO_POS);     // set the first servo to a random position
    //uint8_t ssp = random(MIN_SECOND_LINK_SERVO_POS, MAX_SECOND_LINK_SERVO_POS);   // set the second servo to a random position


    // relative movement for close changes
    uint8_t fsp = random(fsp-ANGLE_BIAS, fsp+ANGLE_BIAS);     // set the first servo to a random position
    uint8_t ssp = random(ssp-ANGLE_BIAS, ssp+ANGLE_BIAS);   // set the second servo to a random positio

    // top and bottom limits
    if (fsp<MIN_FIRST_LINK_SERVO_POS) fsp=MIN_FIRST_LINK_SERVO_POS;
    else if (fsp>MAX_FIRST_LINK_SERVO_POS) fsp=MAX_FIRST_LINK_SERVO_POS;
    
    if (ssp<MIN_SECOND_LINK_SERVO_POS) ssp=MIN_SECOND_LINK_SERVO_POS;
    else if (ssp>MAX_SECOND_LINK_SERVO_POS) ssp=MAX_SECOND_LINK_SERVO_POS;
    
    slowSetServosPos(fsp, ssp, 100, SERVO_DELAY);   // slow change of servo positions (blocking)
    delay(rand_delay());

  }
  if (((seconds() - start_tic) > THREE_MIN) &&  laser_on) {
    analogWrite(LASER_PIN, 0);  // shut the laser
    laser_on = false;
    idle_pos();
    Serial.println("TIME IS UP");

  }

  if (digitalRead(RESET_BUTTON) == HIGH) {
    start_tic = seconds();
    digitalWrite(GREEN_LED, HIGH);
    delay(1000);
    digitalWrite(GREEN_LED, LOW);
    analogWrite(LASER_PIN, LASER_BRIGHTNESS);  // start the laser
    laser_on = true;


  }
}

#define MILLIS_TO_SEC 1000
int rand_delay() {

  //  Serial.print("Delay:  "); Serial.println((millis() % 4 + 2));
  //  return (millis() % 4 + 2) * MILLIS_TO_SEC;
  int hold = abs((angle_diff / 10));
  Serial.print("Delay:  "); Serial.println(hold);
  return hold * MILLIS_TO_SEC;
}
void slowSetServosPos(uint8_t flsPos, uint8_t slsPos, uint8_t steps, uint8_t dt)
{
  static uint8_t flsLastPos = BASE_FIRST_LINK_SERVO_POS;    // previous servo positions (start value centered)
  static uint8_t slsLastPos = BASE_SECOND_LINK_SERVO_POS;   //
  Serial.print("first servo:  "); Serial.print(flsLastPos +  (flsPos - flsLastPos));
  Serial.print("    second servo:  "); Serial.println(slsLastPos + (slsPos - slsLastPos));
  angle_diff = sqrt(pow((flsPos - flsLastPos), 2) + pow((slsPos - slsLastPos), 2));
  for (uint8_t i = 0; i < steps; i++) {     //  add each step a little to the current positions
    firstLinkServo.write(flsLastPos + i * (flsPos - flsLastPos) / steps);

    secondLinkServo.write(slsLastPos + i * (slsPos - slsLastPos) / steps);

    delay(dt);
  }
  flsLastPos = flsPos;  // update past positions
  slsLastPos = slsPos;
}

void idle_pos() {

  secondLinkServo.write(90);
  firstLinkServo.write(90);

}

void sanity_check(int sec_hold = 3) {
  secondLinkServo.write(0);
  firstLinkServo.write(0);
  Serial.println("0");
  int hold = sec_hold * 1000;
  delay(hold);
  secondLinkServo.write(45);
  firstLinkServo.write(45);

  Serial.println("45");

  delay(hold);
  secondLinkServo.write(90);
  firstLinkServo.write(90);

  Serial.println("90");

  delay(hold);
  secondLinkServo.write(135);
  firstLinkServo.write(135);

  Serial.println("135");

  delay(hold);
  secondLinkServo.write(175);
  firstLinkServo.write(175);

  Serial.println("175");

  delay(hold);
}
