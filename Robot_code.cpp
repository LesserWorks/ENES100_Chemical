#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#define STOP 0U
#define FORWARD 1U
#define BACKWARD 2U
#define LEFT 3U
#define RIGHT 4U
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
#define MARKER_ID 7
// Pins for APC
#define RX_PIN 8
#define TX_PIN 9
// Pins and colors for status LED
#define RED 2
#define BLUE 3
#define GREEN 10
#define MAGENTA 4
#define CYAN 5
#define YELLOW 6
#define WHITE 7
#define OFF 0
// Pins for drive motors
// Motor terminal 1
#define LEFT_DIR 4
#define LEFT_PWM 5
// Motor terminal 2
#define RIGHT_PWM 6
#define RIGHT_DIR 7
// Pins for sonor sensors
#define LEFT_TRIGGER 0
#define LEFT_ECHO 0
#define RIGHT_TRIGGER 0
#define RIGHT_ECHO 0
// Pin for pH meter
#define PH_PIN A0
static const int16_t defaultSpeed = 250; // A positive number passed to the motors() function 
                                         // makes the motor turn forward
static const int16_t halfDefault = 255;  
static const int16_t turnSpeed = 190;                                       
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
//uint8_t clearance = 20; // Clearance in cm between robot and obstacle

void getLocation(void);
/* Old functions */
uint16_t headingToDestination(uint16_t destx, uint16_t desty);
void moveTo(uint16_t destx, uint16_t desty);
uint8_t moveToUntilObstacle(uint16_t destx, uint16_t desty, uint8_t clearance_cm);
void turnTo(uint16_t heading);
void motors(int16_t leftSpeed, int16_t rightSpeed);
void getPH(void);
void moveToWall(void);
void turnTest(void);
void radioTest(void);
void baseObjectiveTest(void);

/* New functions */
void mot(const uint8_t speed, const uint8_t mode);
void turn2(const uint16_t heading, const uint8_t speed);

void setup() 
{
  pinMode(RED, 1);
  pinMode(GREEN, 1);
  pinMode(BLUE, 1);
  status(OFF);
  Serial.begin(9600);
  status(RED);
  delay(750);
  status(WHITE);
  delay(750);
  status(GREEN);
  delay(750);
  bool success = rf.retrieveDestination();
  if(!success)
  {
    Serial.println("Failed to get destination.");
    rf.println("Failed to get destination.");
  }
  pool.x = (uint16_t)(1000.0 * rf.destination.x); // Convert from meters to millimeters
  pool.y = (uint16_t)(1000.0 * rf.destination.y);
}

void loop() 
{
  getLocation();
  delay(1000);
  /*getLocation();
  rf.print("X: ");
  rf.println((int)robot.x);
  rf.print("Y: ");
  rf.println((int)robot.y);
  rf.print("Theta: ");
  rf.println((int)robot.theta);
  delay(1000);*/
}

uint16_t headingToDestination(uint16_t destx, uint16_t desty) // This works, has been tested
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
  status(BLUE);
  bool success = rf.updateLocation();
  if(!success)
  {
    Serial.println("Failed to update location.");
    rf.println("Failed to update location.");
    status(RED);
  }
  else
  {
    status(GREEN);
  }
  robot.x = (uint16_t)(1000.0 * rf.location.x); // Meters as a float to millimeters as a uint16_t
  robot.y = (uint16_t)(1000.0 * rf.location.y);
  //robot.theta = (uint16_t)(1000.0 * rf.location.theta); // Radians to milliradians
  int16_t temp = 1000.0 * rf.location.theta; // If update is unsuccessful, it just writes same value back into robot.theta
  if(temp < 0)
  {
    robot.theta = (uint16_t)(temp + FULL_CIRCLE);
  }
  else
  {
    robot.theta = (uint16_t)temp;
  }
  rf.print("Theta: ");
  rf.println((int)robot.theta);
  Serial.print("Theta: ");
  Serial.println(robot.theta);
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
    int16_t highb = 0;
    int16_t lowb = 0;
    if((int16_t)heading - tol < 0) // This bunch of if statements just calculates a margin of error for the heading
    {
      lowb = 0;
      highb = (int16_t)heading + tol;
    }
    else if((int16_t)heading + tol > ((int16_t)FULL_CIRCLE))
    {
      highb = (int16_t)FULL_CIRCLE;
      lowb = (int16_t)heading - tol;
    }
    else
    {
      highb = (int16_t)heading + tol;
      lowb = (int16_t)heading - tol;
    }
    if(!(((int16_t)robot.theta > lowb) && ((int16_t)robot.theta < highb))) // Have we drifted off course?
    {
      mot(0, STOP); // Stop motors
      turn2(heading, turnSpeed); // Correct our heading if we have drifted off course
      corrections++;
    }
    if(corrections > 10) // If we have had to correct this many times, we will probably not hit the destination
    {
      heading = headingToDestination(destx, desty); // Recompute heading
      corrections = 0; // Reset the number of corrections
    }
    mot(defaultSpeed, FORWARD);
  }
  
  mot(0, STOP); // Stop moving
  return;
}
// Moves to destination until an obstacle is encountered. 
// If an obstacle is encountered, the robot stops and the function returns 0. 
// If it reaches the destination, the robot stops and the function returns 1.
uint8_t moveToUntilObstacle(uint16_t destx, uint16_t desty, uint8_t clearance_cm) 
{
  uint16_t heading = headingToDestination(destx, desty);
  uint8_t corrections = 0;
  while(!closeEnough(robot.x, destx) || !closeEnough(robot.y, desty))
  { // Not sure if this is the best way, but it should work.
    getLocation(); // Update our location and heading
    int16_t highb = 0;
    int16_t lowb = 0;
    if((int16_t)heading - tol < 0) // This bunch of if statements just calculates a margin of error for the heading
    {
      lowb = 0;
      highb = (int16_t)heading + tol;
    }
    else if((int16_t)heading + tol > ((int16_t)FULL_CIRCLE))
    {
      highb = (int16_t)FULL_CIRCLE;
      lowb = (int16_t)heading - tol;
    }
    else
    {
      highb = (int16_t)heading + tol;
      lowb = (int16_t)heading - tol;
    }
    if(!(((int16_t)robot.theta > lowb) && ((int16_t)robot.theta < highb))) // Have we drifted off course?
    {
      mot(0, STOP); // Stop motors
      turn2(heading, turnSpeed); // Correct our heading if we have drifted off course
      corrections++;
    }
    if(corrections > 10) // If we have had to correct this many times, we will probably not hit the destination
    {
      heading = headingToDestination(destx, desty); // Recompute heading
      corrections = 0; // Reset the number of corrections
    }
    uint8_t rdist = rsense.ping_cm();
    uint8_t ldist = lsense.ping_cm();
    if((rdist || ldist) && (rdist < clearance_cm || ldist < clearance_cm)) // Are either nonzero and if so, within allowable clearance?
    {
      mot(0, STOP);
      return 0; // We did not reach destination
    }
    mot(defaultSpeed, defaultSpeed);
  }
  
  mot(0, STOP); // Stop moving
  return 1; // We did reach destination
}
void turnTo(uint16_t heading) // Use turn2 instead
{
  getLocation();
  if(robot.theta < heading) // Must turn counterclockwise (to left)
  { // To turn left, right motors go forward and left motors go backward
    motors(defaultSpeed*-1, defaultSpeed);
  }
  else // Must turn clockwise (to right)
  { // To turn right, left motors for forward and right motors go backward
    motors(halfDefault, halfDefault*(-1));
  }
  while(!closeEnough(robot.theta, heading))
  {
    getLocation(); //updates angle so that we'll know to stop turning
  }
  motors(0,0);
  return;
}
void turn2(const uint16_t heading, const uint8_t speed)
{
  getLocation();
  int16_t diff = (int16_t)heading - (int16_t)robot.theta;
  Serial.println(diff);
  if(diff > 0) 
  {
    if(diff > (int16_t)HALF_CIRCLE) // diff is within (180, 360)
    {
       mot(speed, RIGHT);
       Serial.println("Turning right");
    }
    else // diff is within (0, 180)
    {
      mot(speed, LEFT);
      Serial.println("Turning left");
    }
  }
  else
  {
    if(diff > (-1)*(int16_t)HALF_CIRCLE) // diff is within (-180, 0)
    {
      mot(speed, RIGHT);
      Serial.println("Turning right-else");
    }
    else // diff is within (-360, -180)
    {
      mot(speed, LEFT);
      Serial.println("Turning left-else");
    }
  }
  int16_t highb = 0;
  int16_t lowb = 0;
  if((int16_t)heading - tol < 0)
  {
    lowb = 0;
    highb = (int16_t)heading + tol;
  }
  else if((int16_t)heading + tol > ((int16_t)FULL_CIRCLE))
  {
    highb = (int16_t)FULL_CIRCLE;
    lowb = (int16_t)heading - tol;
  }
  else
  {
    highb = (int16_t)heading + tol;
    lowb = (int16_t)heading - tol;
  }
  Serial.println(lowb);
  Serial.println(highb);
  while(!(((int16_t)robot.theta > lowb) && ((int16_t)robot.theta < highb)))
  {
    getLocation(); //updates angle so that we'll know to stop turning
  }
  mot(0,STOP);
  return;
}
void motors(int16_t leftSpeed, int16_t rightSpeed) // Use mot() instead
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
    int8_t val = (leftSpeed * (-1));
    //Serial.println(val);
    //analogWrite(LEFT_PWM, (uint8_t)(leftSpeed * (-1)));
    analogWrite(LEFT_PWM, val);
  }
  else
  {
    digitalWrite(LEFT_DIR, HIGH);
    analogWrite(LEFT_PWM, leftSpeed);
  }
  if(rightSpeed < 0)
    {
    digitalWrite(RIGHT_DIR, LOW);
    analogWrite(RIGHT_PWM, (rightSpeed *(-1)));
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
  moveTo(robot.x + 1000, robot.y); // Move one meter to right
  turn2(THREE_QUARTER, turnSpeed); // Turn to point down
  moveTo(robot.x, robot.y - 250); // Move 25 cm downward
  turn2(HALF_CIRCLE, turnSpeed); // Turn to point to left
  moveTo(robot.x - 250, robot.y); // Move 25 cm to left
  turn2(THREE_QUARTER, turnSpeed); // Turn to point down
  /*while(robot.theta > ONE_QUARTER)
  {
    motors(defaultSpeed*-1, defaultSpeed);
    getLocation();
    rf.print("X: ");
  rf.println((int)robot.x);
  rf.print("Y: ");
  rf.println((int)robot.y);
  rf.print("Theta: ");
  rf.println((int)robot.theta);
  }
  motors(0, 0);
  delay(500);
  while(robot.theta < HALF_CIRCLE)
  {
    motors(defaultSpeed, defaultSpeed*-1);
    getLocation();
    rf.print("X: ");
  rf.println((int)robot.x);
  rf.print("Y: ");
  rf.println((int)robot.y);
  rf.print("Theta: ");
  rf.println((int)robot.theta);
  }
  motors(0, 0);
  delay(500);
  while(robot.theta > ONE_QUARTER)
  {
    motors(defaultSpeed*-1, defaultSpeed);
    getLocation();
    rf.print("X: ");
  rf.println((int)robot.x);
  rf.print("Y: ");
  rf.println((int)robot.y);
  rf.print("Theta: ");
  rf.println((int)robot.theta);
  }
  motors(0, 0);
  //rf.endMission();*/
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
  moveToUntilObstacle(pool.x, pool.y, 2); // Drives further, then stops when we are about 2 cm from pool.
  rf.navigated();
  // Deploy pH probe here
  getPH();
  rf.endMission();
  
}
void mot(const uint8_t speed, const uint8_t mode) // a better motor function
{ // This function requires the black wires from all motors to be in the + side of the terminal
  // and for the right motors to connect to terminal 1
  switch(mode)
  {
    case FORWARD:
      digitalWrite(LEFT_DIR, 1);
      digitalWrite(RIGHT_DIR, 1);
      analogWrite(LEFT_PWM, speed);
      analogWrite(RIGHT_PWM, speed);
      return;
    case BACKWARD:
      digitalWrite(LEFT_DIR, 0);
      digitalWrite(RIGHT_DIR, 0);
      analogWrite(LEFT_PWM, speed);
      analogWrite(RIGHT_PWM, speed);
      return;
    case LEFT:
      digitalWrite(LEFT_DIR, 1);
      digitalWrite(RIGHT_DIR, 0);
      analogWrite(LEFT_PWM, speed);
      analogWrite(RIGHT_PWM, speed);
      return;
    case RIGHT:
      digitalWrite(LEFT_DIR, 0);
      digitalWrite(RIGHT_DIR, 1);
      analogWrite(LEFT_PWM, speed);
      analogWrite(RIGHT_PWM, speed);
      return;
    default:
      digitalWrite(LEFT_DIR, 0);
      digitalWrite(RIGHT_DIR, 0);
      analogWrite(LEFT_PWM, 0);
      analogWrite(RIGHT_PWM, 0);
      return;
  }
}
void status(const uint8_t color)
{
  switch(color)
  {
    case RED:
      digitalWrite(RED, 0);
      digitalWrite(GREEN, 1);
      digitalWrite(BLUE, 1);
      return;
    case GREEN:
      digitalWrite(RED, 1);
      digitalWrite(GREEN, 0);
      digitalWrite(BLUE, 1);
      return;
    case BLUE:
      digitalWrite(RED, 1);
      digitalWrite(GREEN, 1);
      digitalWrite(BLUE, 0);
      return;
    case MAGENTA:
      digitalWrite(RED, 0);
      digitalWrite(GREEN, 1);
      digitalWrite(BLUE, 0);
      return;
    case CYAN:
      digitalWrite(RED, 1);
      digitalWrite(GREEN, 0);
      digitalWrite(BLUE, 0);
      return;
    case YELLOW:
      digitalWrite(RED, 0);
      digitalWrite(GREEN, 0);
      digitalWrite(BLUE, 1);
      return;
    case WHITE:
      digitalWrite(RED, 0);
      digitalWrite(GREEN, 0);
      digitalWrite(BLUE, 0);
      return;
      
    default:
      digitalWrite(RED, 1);
      digitalWrite(GREEN, 1);
      digitalWrite(BLUE, 1);
      return; 
  }
}
