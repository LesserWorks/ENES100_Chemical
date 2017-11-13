#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/* 
 * For atan2(), if y is 0, it gives PI, if x is 0, it gives PI / 2, if both are 0, it gives 0.
 * atan2 returns a value (-PI, PI) in radians
 * For third quadrant, it gives negative angle referenced from 0, so atan2(-10,-10) gives -135.00000 (but in radians).
 * 
 * 
 * Navigation:
 * Sonar sensor returns an analog value regarding distance to object, no direction info given.
 * If sensor returns value above threshhold, turn until it drops below threshold. Then keep going forward,
 * compute new heading.
 * 
 * With two sonar sensors in front, go forward till we detect an obstacle in one or both of them. Turn until obstacle disappears
 * from view.
 */
#include "Enes100.h"
#include "NewPing.h"
#define MARKER_ID 12
#define RX_PIN 8
#define TX_PIN 9
#define LEFT_DIR 4
#define LEFT_PWM 5
#define RIGHT_PWM 6
#define RIGHT_DIR 7
#define LEFT_TRIGGER 0
#define LEFT_ECHO 0
#define RIGHT_TRIGGER 0
#define RIGHT_ECHO 0
#define PH_PIN A0
static const int16_t defaultSpeed = 128; // A positive number passed to the motors() function 
                                         // makes the motor turn forward
/*
//Definitions for measuring pH
unsigned long int avgValue;  //Store the average value of the sensor feedback
float b;
int buffer[10] //array to hold pH readings
*/
// These macros give values in milliradians
#define FULL_CIRCLE ((uint16_t)(M_PI * 2000.0)) // Two pi
#define THREE_QUARTER ((uint16_t)(M_PI * 1500.0)) // Three pi over two
#define HALF_CIRCLE ((uint16_t)(M_PI * 1000.0)) // Pi
#define ONE_QUARTER ((uint16_t)(M_PI * 500.0)) // Pi over two

#define tol 50 // Tolerance in millimeters (or milliradians)
#define degToRad(degree) ((degree * M_PI) / 180.0)
#define radToDeg(radian) ((radian * 180.0) / M_PI)
#define closeEnough(a,b) (a <= (b + tol) && a >= (b - tol)) // True if a is within (b - tol, b + tol)
Enes100 rf("The Swiss Army Bot", CHEMICAL, MARKER_ID, RX_PIN, TX_PIN);
NewPing rsense(RIGHT_TRIGGER, RIGHT_ECHO, 255);
NewPing lsense(LEFT_TRIGGER, LEFT_ECHO, 255);

struct coord // Use this instead of provided coordinate class because theirs uses floats
{
  uint16_t x;
  uint16_t y;
  uint16_t theta;
};

struct coord robot; // Updated with coordinates whenevey they are received (the current location of robot).
struct coord pool;
uint8_t clearance = 20; // Clearance in cm between robot and obstacle

void getLocation(void);
uint16_t headingToDestination(uint16_t destx, uint16_t desty);
void moveTo(uint16_t destx, uint16_t desty);
uint8_t moveToUntilObstacle(uint16_t destx, uint16_t desty);
void turnTo(uint16_t heading);
void motors(int16_t leftSpeed, int16_t rightSpeed);
void getPH(void);
void moveToWall(void);
void turnTest(void);
void radioTest(void);
void baseObjectiveTest(void);

void setup() 
{
  Serial.begin(9600);
  delay(5000);
  uint8_t success = rf.retrieveDestination();
  if(!success)
  {
    Serial.println("Failed to get destination.");
    rf.println("Failed to get destination.");
  }
  pool.x = (uint16_t)(1000.0 * rf.destination.x); // Convert from meters to millimeters
  pool.y = (uint16_t)(1000.0 * rf.destination.y);
  getLocation();
  moveToWall();
  /*motors(128, 128);
  delay(1000);
  motors(0, 0);
  delay(1000);
  motors(128, -128);
  delay(1000);
  motors(-128, 128);
  delay(1000);
  motors(0, 0);*/
}

void loop() 
{
  getLocation();
}

uint16_t headingToDestination(uint16_t destx, uint16_t desty)
{
  int16_t dy = (int16_t)desty - (int16_t)robot.y;
  int16_t dx = (int16_t)destx - (int16_t)robot.x;
  if(dx == 0) // Is destination straight up or down?
  {
    return dy > 0 ? ONE_QUARTER : THREE_QUARTER;
  }
  
  if(dy == 0) // Is destination to right or left?
  {
    return dx > 0 ? 0 : HALF_CIRCLE;
  }
  int16_t heading = 1000.0 * atan2(dy, dx); 
  if(heading < 0)
  {
    heading = FULL_CIRCLE + heading; // (0, 360) instead of (-180, 180)
  }
  return (uint16_t)heading;
}
void getLocation(void) // This function updates the robots's coordinates
{
  uint8_t success = rf.updateLocation();
  if(!success)
  {
    Serial.println("Failed to update location.");
    rf.println("Failed to update location.");
    return;
  }
  robot.x = (uint16_t)(1000.0 * rf.location.x); // Meters as a float to millimeters as a uint16_t
  robot.y = (uint16_t)(1000.0 * rf.location.y);
  robot.theta = (uint16_t)(1000.0 * rf.location.theta); // Radians to milliradians
  // The following line references theta from east instead of north, if necessary
  //robot.theta = robot.theta > THREE_QUARTER ? robot.theta - THREE_QUARTER : robot.theta + ONE_QUARTER;
  return;
}
void moveTo(uint16_t destx, uint16_t desty) // Moves to destination location without accounting for obstacles
{
  uint16_t heading = headingToDestination(destx, desty);
  uint8_t corrections = 0;
  while(!closeEnough(robot.x, destx) || !closeEnough(robot.y, desty))
  { // Not sure if this is the best way, but it should work.
    getLocation(); // Update our location and heading
    if(!closeEnough(robot.theta, heading)) // Have we drifted off course?
    {
      motors(0, 0); // Stop motors
      turnTo(heading); // Correct our heading if we have drifted off course
      corrections++;
    }
    if(corrections > 10) // If we have had to correct this many times, we will probably not hit the destination
    {
      heading = headingToDestination(destx, desty); // Recompute heading
      corrections = 0; // Reset the number of corrections
    }
    motors(defaultSpeed, defaultSpeed);
  }
  
  motors(0, 0); // Stop moving
  return;
}
// Moves to destination until an obstacle is encountered. 
// If an obstacle is encountered, the robot stops and the function returns 0. 
// If it reaches the destination, the robot stops and the function returns 1.
uint8_t moveToUntilObstacle(uint16_t destx, uint16_t desty) 
{
  uint16_t heading = headingToDestination(destx, desty);
  uint8_t corrections = 0;
  while(!closeEnough(robot.x, destx) || !closeEnough(robot.y, desty))
  { // Not sure if this is the best way, but it should work.
    getLocation(); // Update our location and heading
    if(!closeEnough(robot.theta, heading)) // Have we drifted off course?
    {
      motors(0, 0); // Stop motors
      turnTo(heading); // Correct our heading if we have drifted off course
      corrections++;
    }
    if(corrections > 10) // If we have had to correct this many times, we will probably not hit the destination
    {
      heading = headingToDestination(destx, desty); // Recompute heading
      corrections = 0; // Reset the number of corrections
    }
    uint8_t rdist = rsense.ping_cm();
    uint8_t ldist = lsense.ping_cm();
    if((rdist || ldist) && (rdist < clearance || ldist < clearance)) // Are either nonzero and if so, within allowable clearance?
    {
      motors(0, 0);
      return 0; // We did not reach destination
    }
    motors(defaultSpeed, defaultSpeed);
  }
  
  motors(0, 0); // Stop moving
  return 1; // We did reach destination
}
void turnTo(uint16_t heading) // This function turns the robot until it is pointing in the given direction
{
  getLocation();
  if(robot.theta < heading) // Must turn counterclockwise (to left)
  { // To turn left, right motors go forward and left motors go backward
    motors(defaultSpeed*(-1), defaultSpeed);
  }
  else // Must turn clockwise (to right)
  { // To turn right, left motors for forward and right motors go backward
    motors(defaultSpeed, defaultSpeed*(-1));
  }
  while(!closeEnough(robot.theta, heading))
  {
    getLocation(); //updates angle so that we'll know to stop turning
  }
  motors(0,0);
  return;
}
void motors(int16_t leftSpeed, int16_t rightSpeed) // This function turns the motors on at the given speeds (from -255-255)
{ // A positive number causes that motor to turn forward
  if(leftSpeed == 0)
  {
    digitalWrite(LEFT_DIR, LOW);
    digitalWrite(LEFT_PWM, 0);
  }
  if(rightSpeed == 0)
  {
    digitalWrite(RIGHT_DIR, LOW);
    digitalWrite(RIGHT_PWM, 0);
  }
  if(leftSpeed < 0)
  {
    digitalWrite(LEFT_DIR, LOW);
    analogWrite(LEFT_PWM, (uint8_t)(leftSpeed * (-1)));
  }
  else
  {
    digitalWrite(LEFT_DIR, HIGH);
    analogWrite(LEFT_PWM, leftSpeed);
  }
  if(rightSpeed < 0)
    {
    digitalWrite(RIGHT_DIR, LOW);
    analogWrite(RIGHT_PWM, (uint8_t)(rightSpeed *(-1)));
    }
  else
  {
    digitalWrite(RIGHT_DIR, HIGH);
    analogWrite(RIGHT_PWM, rightSpeed);
  }   
}

void getPH(void)
{
  uint16_t total = 0;
  for(uint8_t i = 0; i < 10; i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    /*buffer[i]=analogRead(PH_PIN);
    total += buffer[i];*/
    total += analogRead(PH_PIN);
    delay(10);
  }
  float phValue = ((float)total) / 10.0;
  phValue = map(phValue, 0, 1023, 0, 14);
  rf.baseObjective(phValue);
}
// Below are MS5 functions
void moveToWall(void) // Forward locomotion test
{
  getLocation();
  moveTo(3900, robot.y);
  rf.navigated();
  rf.println("Reached wall");
  rf.endMission();
}
void turnTest(void) // Turning test
{
  getLocation(); // Robot should start facing to right
  moveTo(1000, robot.y); // Move one meter to right
  turnTo(THREE_QUARTER); // Turn to point down
  moveTo(robot.x, robot.y + 250); // Move 25 cm downward
  turnTo(0); // Turn to point to right
  moveTo(robot.x + 250, robot.y); // Move 25 cm to right
  turnTo(ONE_QUARTER); // Turn to point up
  rf.endMission();
  return;
}
void radioTest(void) // RF communications test
{
  Serial.begin(9600);
  getLocation();
  Serial.print("X: ");
  Serial.println(robot.x);
  Serial.print("Y: ");
  Serial.println(robot.y);
  Serial.print("Theta: ");
  Serial.println(robot.theta);
  rf.print("X: ");
  rf.println((int)robot.x);
  rf.print("Y: ");
  rf.println((int)robot.y);
  rf.print("Theta: ");
  rf.println((int)robot.theta);
  moveTo(robot.x + 700, robot.y);
  Serial.print("X: ");
  Serial.println(robot.x);
  Serial.print("Y: ");
  Serial.println(robot.y);
  Serial.print("Theta: ");
  Serial.println(robot.theta);
  rf.print("X: ");
  rf.println((int)robot.x);
  rf.print("Y: ");
  rf.println((int)robot.y);
  rf.print("Theta: ");
  rf.println((int)robot.theta);
}
void baseObjectiveTest(void) // Navigate to pool and measure pH
{
  moveTo(pool.x - 100, pool.y - 100); // Drives till we are about 10 cm away from pool
}
