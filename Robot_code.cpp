
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
#define MARKER_ID 12
#define RX_PIN 7
#define TX_PIN 8

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
struct coord // Use this instead of provided coordinate class because theirs uses floats
{
  uint16_t x;
  uint16_t y;
  uint16_t theta;
};

struct coord robot; // Updated with coordinates whenevey they are received (the current location of robot).
struct coord pool;
int16_t clearance = 500; // Clearance in mm between robot and obstacle

void getLocation(void);
uint16_t headingToDestination(uint16_t destx, uint16_t desty);

void setup() 
{
  
  rf.retrieveDestination();
  pool.x = (uint16_t)(1000.0 * rf.destination.x); // Convert from meters to millimeters
  pool.y = (uint16_t)(1000.0 * rf.destination.y);
  getLocation();
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
  int16_t heading = 1000.0 * atan2(dy, dx); // Warning: Expensive calculation!
  if(heading < 0)
  {
    heading = FULL_CIRCLE + heading; // (0, 360) instead of (-180, 180)
  }
  return (uint16_t)heading;
}
void getLocation() // This function updates the robots's coordinates
{
  rf.updateLocation();
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
    motors(128, 128);
  }
  
  motors(0, 0); // Stop moving
  return;
}
// Moves to destination until an obstacle is encountered. 
// If an obstacle is encountered, the robot stops and the function returns 0. 
// If it reaches the destination, the robot stops and the function returns 1.
uint8_t moveToUntilObstacle(uint16_t destx, uint16_t desty) 
{
  unit16_t heading = headingToDestination(destx, desty);
  moveTo(destx, desty);
  int16_t obstacleX = analogRead(RX_PIN);
  getLocation();
  int16_t xClearance = robot.x- obstacleX;
  int16_t yClearance = robot.y- (//y-coordinate of obstacle);
}
void turnTo(uint16_t heading) // This function turns the robot until it is pointing in the given direction
{
  getLocation();
  int16_t xVal = robot.x;
  int16_t yVal = robot.y;
  int16_t dist = sq(xVal) + sq(yVal);
  int16_t theta = sin(xVal);
  
  
}
void motors(uint8_t leftSpeed, uint8_t rightSpeed) // This function turns the motors on at the given speeds (from 0-255)
{
  
}

