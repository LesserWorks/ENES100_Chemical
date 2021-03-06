#ifndef Enes100_h
#define Enes100_h

#include "Arduino.h"
#include "SoftwareSerial.h"
#include "Enums.h"

struct coord
{
  uint16_t x;
  uint16_t y;
  uint16_t theta;
};
extern struct coord robot;
extern struct coord pool;

class Coordinate {
public:
    Coordinate();
    Coordinate(double x, double y);
    Coordinate(double x, double y, double theta);
    double x;
    double y;
    double theta;
private:
    void init(double x, double y, double theta);
};

class Enes100 {
public:
    Enes100(const char* teamName, int teamType, int markerId, int rxPin, int txPin);
    void baseObjective(int value);
    void baseObjective(double value);
    void baseObjective(Coordinate& value);
    void bonusObjective(int value);
    void bonusObjective(double value);
    void endMission();
    void navigated();
    void print(const char* message);
    void print(int message);
    void print(double message);
    void println(const char* message);
    void println(int message);
    void println(double message);
    bool retrieveDestination();
    unsigned long updateLocation();
    
    Coordinate location; // This is what the coordinates are stored in that we access with rf.location.x
    Coordinate destination;
    
private:
    int mId;
    SoftwareSerial* mSoftwareSerial;
};

#endif /* Enes100_h */
