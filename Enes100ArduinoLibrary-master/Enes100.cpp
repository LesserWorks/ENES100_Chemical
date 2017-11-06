#include "Enes100.h"

Coordinate::Coordinate() {
    init(0, 0, 0);
}

Coordinate::Coordinate(double x, double y) {
    init(x, y, 0);
}

Coordinate::Coordinate(double x, double y, double theta) {
    init(x, y, theta);
}

void Coordinate::init(double x, double y, double theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}
struct coord robot = {0,0,0};
struct coord pool = {0,0,0};
// Constructor
Enes100::Enes100(const char* teamName, int teamType, int markerId, int rxPin, int txPin) {
    mId = markerId;
    mSoftwareSerial = new SoftwareSerial(rxPin, txPin); // Here it makes instance of SoftwareSerial class
    mSoftwareSerial->begin(9600);
    
    mSoftwareSerial->print("#start ");
    mSoftwareSerial->print(teamType);
    mSoftwareSerial->print(" ");
    mSoftwareSerial->print(teamName);
    mSoftwareSerial->print("*");
    mSoftwareSerial->flush();
	
}
// If we write Enes100 rf("The Swiss Army Bot", CHEMICAL, 12, 2, 3);
// Then it sends over serial "#start 1 The Swiss Army Bot*"

// MARK: baseObjective
void Enes100::baseObjective(int value) { // We won't be using this
    mSoftwareSerial->print("#base ");
    mSoftwareSerial->print(value);
    mSoftwareSerial->print("*");
    mSoftwareSerial->flush();
}

void Enes100::baseObjective(double value) {
    mSoftwareSerial->print("#base ");
    mSoftwareSerial->print(value);
    mSoftwareSerial->print("*");
    mSoftwareSerial->flush();
}
// If we write rf.baseObjective(9.23);
// Then it sends "#base 9.23*"

void Enes100::baseObjective(Coordinate& value) { // We won't be using this
    mSoftwareSerial->print("#base ");
    mSoftwareSerial->print(value.x);
    mSoftwareSerial->print(",");
    mSoftwareSerial->print(value.y);
    mSoftwareSerial->print("*");
    mSoftwareSerial->flush();
}

// MARK: bonusObjective
void Enes100::bonusObjective(int value) { // We won't be using this
    mSoftwareSerial->print("#bonus ");
    mSoftwareSerial->print(value);
    mSoftwareSerial->print("*");
    mSoftwareSerial->flush();
}

void Enes100::bonusObjective(double value) {
    mSoftwareSerial->print("#bonus ");
    mSoftwareSerial->print(value);
    mSoftwareSerial->print("*");
    mSoftwareSerial->flush();
}
// If we write rf.baseObjective(7.01);
// Then it sends "#bonus 7.01*"

// MARK: endMission
void Enes100::endMission() {
    mSoftwareSerial->print("#end*");
    mSoftwareSerial->flush();
    //while(1); 
}

// MARK: navigated
void Enes100::navigated() { // This will cause second 5 min timer to start
    mSoftwareSerial->print("#navigated*");
    mSoftwareSerial->flush();
}

// MARK: print
void Enes100::print(const char *msg) {
    mSoftwareSerial->print(msg);
    mSoftwareSerial->flush();
}

void Enes100::print(int msg) {
    mSoftwareSerial->print(msg);
    mSoftwareSerial->flush();
}

void Enes100::print(double msg) {
    mSoftwareSerial->print(msg);
    mSoftwareSerial->flush();
}

// MARK: println
void Enes100::println(const char *msg) {
    mSoftwareSerial->println(msg);
    mSoftwareSerial->flush();
}

void Enes100::println(int msg) {
    mSoftwareSerial->println(msg);
    mSoftwareSerial->flush();
}

void Enes100::println(double msg) {
    mSoftwareSerial->println(msg);
    mSoftwareSerial->flush();
}

// MARK: retrieveDestination
bool Enes100::retrieveDestination() {
    mSoftwareSerial->print("#destination*");
    mSoftwareSerial->flush();
    
    unsigned long start = millis();
    int state = 0;
    
    while((millis() - start) < 600) { // loop runs for 600 ms
        if (mSoftwareSerial->available()) {
            switch(state) {
                case 0:
                    destination.x = mSoftwareSerial->parseFloat();
					pool.x = (uint16_t)(1000.0 * destination.x);
                    state++;
                    break;
                case 1:
                    destination.y = mSoftwareSerial->parseFloat();
					pool.y = (uint16_t)(1000.0 * destination.y);
                    state++;
                    break;
                case 2:
                    return true;
                    break;
                default: // Should never get here
                    return false;
                    break;
            }
        }
    }
	// In 600 ms, it parses a float for x coordinate, then a float for y coodinate,
	// then returns true
    
    return false;
}

// MARK: updateLocation
unsigned long Enes100::updateLocation() {
    mSoftwareSerial->print("#");
    mSoftwareSerial->print(mId);
    mSoftwareSerial->print("*");
    mSoftwareSerial->flush();
    // Sends "#12*" for ID = 12
    unsigned long start = millis();
    int state = 0;
    
    while((millis() - start) < 600) {
        if(mSoftwareSerial->available()) {
            switch(state) {
                case 0:
                    if (mSoftwareSerial->parseInt() == mId)
                        state++;
                    break;
                case 1:
                    location.x = mSoftwareSerial->parseFloat();
					robot.x = (uint16_t)(1000.0 * location.x);
                    state++;
                    break;
                case 2:
                    location.y = mSoftwareSerial->parseFloat();
					robot.y = (uint16_t)(1000.0 * location.y);
                    state++;
                    break;
                case 3:
                    location.theta = mSoftwareSerial->parseFloat();
					robot.theta = (uint16_t)(1000.0 * location.theta);
                    return millis() - start;
                    break;
                default:
                    return 0;
                    break;
            }
        }
    }
    // If the first parseInt doesn't return 12 (our ID), it keeps trying to parse for 600 ms.
	// Parses x, then y, then theta coordinates and returns how long it took to parse.
    return 0;
}
