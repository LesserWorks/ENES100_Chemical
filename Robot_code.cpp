#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#define STOP 0U
#define FORWARD 1U
#define BACKWARD 2U
#define LEFT 3U
#define RIGHT 4U
#define LEFT_SWING 5U
#define RIGHT_SWING 6U
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
#include "Servo.h"

// 3, 5, 6, 9, 10, 11 are PWM
#define MARKER_ID 73
// Other colors for status LED
#define MAGENTA 4
#define CYAN 5
#define YELLOW 6
#define WHITE 7
#define OFF 0

// Pin defines
#define PH_PIN A0 // Pin for pH meter
#define RED A1 // Pins for status LED
#define GREEN A2
#define BLUE A3
#define RIGHT_SONAR A4
#define LEFT_SONAR A5
#define RIGHT_TOUCH 0 // Pins of two touch sensors
#define LEFT_TOUCH 1
#define NOT_USED 2 
#define PUMP_PIN 3 // PWM for pump
#define LEFT_DIR 4 // Motor terminal 1
#define LEFT_PWM 5
#define RIGHT_PWM 6 // Motor terminal 2
#define RIGHT_DIR 7
#define ALSO_NOT_USED 8 
#define BASE_PINCH 9 // Base hose pinch
#define ACID_PINCH 10 // Acid hose pinch
#define ARM 11 // Arm servo
#define RX_PIN 12 // Pins for APC
#define TX_PIN 13
static const int16_t defaultSpeed = 200; // A positive number passed to the motors() function 
                                         // makes the motor turn forward
static const int16_t halfDefault = 255;  
static const int16_t turnSpeed = 220;    
static const int16_t slowForward = 150;
static const int16_t slowTurn = 170;                                   
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

#define ARM_DOWN 95 // Servo angles for up and down positions
#define ARM_UP 5
#define ACID_OPEN 200 // aka Front servo
#define ACID_CLOSED 65
#define BASE_OPEN 110 // aka back servo
#define BASE_CLOSED 10
#define tol 100 // Tolerance in millimeters (or milliradians)
#define degToRad(degree) ((degree * M_PI) / 180.0)
#define radToDeg(radian) ((radian * 180.0) / M_PI)
#define closeEnough(a,b) (a <= (b + tol) && a >= (b - tol)) // True if a is within (b - tol, b + tol)

Enes100 rf("The Swiss Army Bot", CHEMICAL, MARKER_ID, RX_PIN, TX_PIN);
NewPing rsense(RIGHT_SONAR, RIGHT_SONAR, 255);
NewPing lsense(LEFT_SONAR, LEFT_SONAR, 255);
Servo acidPinch;
Servo basePinch;
Servo arm;

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
float getPH(void);
void moveToWall(void);
void turnTest(void);
void radioTest(void);
void baseObjectiveTest(void);

/* New functions */
void mot(const uint8_t speed, const uint8_t mode);
void turn2(const uint16_t heading, const uint8_t speed);
void moveToNoCorrection(int16_t destx, int16_t desty);
uint8_t moveToNoCorrectionUntilObstacle(int16_t destx, int16_t desty, uint8_t clearance_cm);
void setupPins(void);

void setup() // The code currently in void setup() gets us to within 10 cm of the pool using the Vision system and sonar sensors only.
{
  setupPins();
  status(RED);
  delay(750);
  status(WHITE);
  delay(750);
  status(GREEN);
  delay(750);
  bool success = rf.retrieveDestination();
  if(!success)
  {
    rf.println("Failed to get destination.");
  }
  pool.x = (uint16_t)(1000.0 * rf.destination.x); // Convert from meters to millimeters
  pool.y = (uint16_t)(1000.0 * rf.destination.y);
  rf.print("Pool coordinates: (");
  rf.print((int)pool.x);
  rf.print(", ");
  rf.print((int)pool.y);
  rf.println(") mm");
  turn2(ONE_QUARTER / 3, turnSpeed); // To to point right and slightly up
  moveToNoCorrection(robot.x + 500, robot.y); // Gets us over the rocky area
  delay(300);
  turn2(headingToDestination(pool.x, pool.y), turnSpeed); // Turn to point towards pool
  while(!moveToNoCorrectionUntilObstacle(pool.x, pool.y, 15)) // While we keep encountering obstacles
  {
    if(abs(pool.x - robot.x) > 100 || abs(pool.y - robot.y) > 100) // Are we too far away (> 100 mm) from pool?
    {
      if(robot.y > 1000) // Are we in upper half of arena?
      {
        turn2(THREE_QUARTER + 100, turnSpeed); // Turn to point down and a bit right
        moveToNoCorrection(robot.x + 100, robot.y - 200); // Move down past obstacle
        turn2(FULL_CIRCLE - (ONE_QUARTER / 3), turnSpeed); // Turn to point right and slightly downward
      }
      else // We are in lower half of arena
      {
        turn2(ONE_QUARTER - 100, turnSpeed);
        moveToNoCorrection(robot.x + 100, robot.y + 200); // Move up past obstacle
        turn2(ONE_QUARTER / 3, turnSpeed); // Turn to point right and slightly upward
      }
      moveToNoCorrection(robot.x + 30, robot.y); // Move a bit forward
      turn2(headingToDestination(pool.x, pool.y), turnSpeed); // Turn to point towards pool
    }
    else // We are close to pool
    {
      break; // End while
    }
  }
  // At this point we should be within 10 cm of pool
  turn2(headingToDestination(pool.x, pool.y), turnSpeed); // Turn to point towards pool
  mot(slowForward, FORWARD);
  while(!digitalRead(LEFT_TOUCH) && !digitalRead(RIGHT_TOUCH)); // Wait till one of the switches gets pushed
  mot(120, BACKWARD); // Brake motors by briefly applying reverse voltage
  delay(100);
  mot(0, STOP);
  if(digitalRead(LEFT_TOUCH)) // The left sensor touched first
  {
    mot(slowForward, LEFT_SWING);
    while(!digitalRead(RIGHT_TOUCH));
  }
  else // The right sensor touched first
  {
    mot(slowForward, RIGHT_SWING);
    while(!digitalRead(LEFT_TOUCH));
  }
  mot(0, STOP); // At this point both tough sensors should be pressed
  for(uint8_t i = ARM_UP; i < ARM_DOWN; i++)
  {
    arm.write(i);
    delay(70); // Deploy arm slowly
  }
  rf.navigated();
  status(WHITE);
  analogWrite(PUMP_PIN, 128); // Begin drawing sample
  delay(10000); // Insert delay long enough to draw sample
  analogWrite(PUMP_PIN, 0); // Turn off pump
  float phValue = getPH();
  rf.baseObjective(phValue);
  if(phValue < 7.0) // Pool is acidic, inject base
  {
    status(RED); // Like litmus paper
    while(phValue < 6.0) // We must neutralize it to between 6-8
    {
      basePinch.write(BASE_OPEN); // Open base tube
      delay(1000); // Inject for 1 second
      basePinch.write(BASE_CLOSED); // Close base tube
      delay(5000); // Let mixture settle for 5 seconds
      phValue = getPH();
    }
  }
  else // Pool is basic, inject acid
  {
    status(MAGENTA); // Like litmus paper
    while(phValue > 8.0)
    {
      acidPinch.write(ACID_OPEN); // Open acid tube
      delay(1000); // Inject for 1 second
      acidPinch.write(ACID_CLOSED); // Close acid tube
      delay(5000); // Let mixture settle for 5 seconds
      phValue = getPH();
    }
  }
  status(GREEN);
  rf.bonusObjective(phValue); // Send neutralized phValue
  for(uint8_t i = ARM_DOWN; i > ARM_UP; i--)
  {
    arm.write(i);
    delay(70); // Raise arm back up
  }
  rf.endMission();
}

void loop() {}
void setupPins(void)
{
  pinMode(BASE_PINCH, 1);
  pinMode(ACID_PINCH, 1);
  pinMode(ARM, 1);
  basePinch.attach(BASE_PINCH);
  acidPinch.attach(ACID_PINCH);
  arm.attach(ARM);
  basePinch.write(BASE_CLOSED); // Write default positions here
  acidPinch.write(ACID_CLOSED);
  arm.write(ARM_UP);
  pinMode(PH_PIN, 0);
  pinMode(RED, 1);
  pinMode(BLUE, 1);
  pinMode(GREEN, 1);
  pinMode(RIGHT_SONAR, 1);
  pinMode(LEFT_SONAR, 1);
  pinMode(RIGHT_TOUCH, 0);
  pinMode(LEFT_TOUCH, 0);
  pinMode(PUMP_PIN, 1);
  pinMode(LEFT_DIR, 1);
  pinMode(LEFT_PWM, 1);
  pinMode(RIGHT_PWM, 1);
  pinMode(RIGHT_DIR, 1);
  pinMode(RX_PIN, 0);
  pinMode(TX_PIN, 1);
  status(OFF);
  mot(0, STOP);
  return;
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
  // The following line references theta from east instead of north, if necessary
  //robot.theta = robot.theta > THREE_QUARTER ? robot.theta - THREE_QUARTER : robot.theta + ONE_QUARTER;
  return;
}
void moveToNoCorrection(int16_t destx, int16_t desty)
{
  if(abs((int16_t)robot.x - destx) < abs((int16_t)robot.y - desty)) // Closer to x, so move while checking y
  {
    mot(defaultSpeed, FORWARD);
    while(!closeEnough((int16_t)robot.y, desty))
    {
      getLocation();
    }
  }
  else // Closer to y, so move while checking x
  {
    mot(defaultSpeed, FORWARD);
    while(!closeEnough((int16_t)robot.x, destx))
    {
      getLocation();
    }
  }
  mot(0, STOP);
  return;
}
uint8_t moveToNoCorrectionUntilObstacle(int16_t destx, int16_t desty, uint8_t clearance_cm)
{
  if(abs((int16_t)robot.x - destx) < abs((int16_t)robot.y - desty)) // Closer to x, so move while checking y
  {
    mot(defaultSpeed, FORWARD);
    while(!closeEnough((int16_t)robot.y, desty))
    {
      getLocation();
      uint8_t rdist = rsense.ping_cm();
      uint8_t ldist = lsense.ping_cm();
      if((rdist || ldist) && (rdist < clearance_cm || ldist < clearance_cm)) // Are either nonzero and if so, within allowable clearance?
      {
        mot(0, STOP);
        return 0; // We did not reach destination
      }
    }
  }
  else // Closer to y, so move while checking x
  {
    mot(defaultSpeed, FORWARD);
    while(!closeEnough((int16_t)robot.x, destx))
    {
      getLocation();
      uint8_t rdist = rsense.ping_cm();
      uint8_t ldist = lsense.ping_cm();
      if((rdist || ldist) && (rdist < clearance_cm || ldist < clearance_cm)) // Are either nonzero and if so, within allowable clearance?
      {
        mot(0, STOP);
        return 0; // We did not reach destination
      }
    }
  }
  mot(0, STOP);
  return 1; // We did reach destination
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

float getPH(void)
{
  uint16_t total = 0;
  for(uint8_t i = 0; i < 10; i++)
  { 
    total += analogRead(PH_PIN);
    delay(10);
  }
  float phValue = ((float)total) / 10.0;
  phValue = map(phValue, 0.0, 1023.0, 0.0, 14.0);
  return phValue - 0.3;
}
// Below are MS5 functions


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
    case LEFT: // Turns left motors backwards and right motors forward
      digitalWrite(LEFT_DIR, 1);
      digitalWrite(RIGHT_DIR, 0);
      analogWrite(LEFT_PWM, speed);
      analogWrite(RIGHT_PWM, speed);
      return;
    case RIGHT: // Turns right motors backwards and left motors forward
      digitalWrite(LEFT_DIR, 0);
      digitalWrite(RIGHT_DIR, 1);
      analogWrite(LEFT_PWM, speed);
      analogWrite(RIGHT_PWM, speed);
      return;
    case LEFT_SWING: // Turns right motors forward and left motors off
      digitalWrite(LEFT_DIR, 1);
      digitalWrite(RIGHT_DIR, 0);
      analogWrite(LEFT_PWM, speed);
      analogWrite(RIGHT_PWM, 0);
      return;
    case RIGHT_SWING: // Turns left motors forward and right motors off
      digitalWrite(LEFT_DIR, 0);
      digitalWrite(RIGHT_DIR, 1);
      analogWrite(LEFT_PWM, 0);
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
