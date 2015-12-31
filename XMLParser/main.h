#include "mbed.h"
#include "spdomparser.hpp"
#include "spxmlnode.hpp"
#include "spxmlhandle.hpp"
#include "spxmlhandle.hpp"
#include "spxmlnode.hpp"
#include "spxmlutils.hpp"
#include "spxmlevent.hpp"
#include <string.h>
#include "LinkedList.h"
#include <stdlib.h>
#include "rtos.h"
#include "mbed_rpc.h"
#include "MBed_Adafruit_GPS.h"
#include "Adafruit_9DOF.h"
#include "MODSERIAL.h"

struct LatLong {
    double latitude;
    double longitude;   
};

struct Waypoint {
    int waypointId;
    double latitude;
    double longitude;
    long segmentNum;
};

struct Segment {
    long segId;
    char name[50];
    double time;
    double distance;
    char turn[50];
    double turnAngle;
    double startBearing;
    double endBearing;
};
    

typedef struct LatLong LatLong;
typedef struct Waypoint Waypoint;
typedef struct Segment Segment;

LinkedList<node> segmentList;
LinkedList<node> waypointList;
node *tmp;
Ticker compassTicker;
float compassValue;
float *ax, *ay, *az, *mx, *my, *mz;
bool compassTriggered = false;

//Threads
Thread *threadCompass;
Thread *threadBluetooth;
Thread *threadGPS;

//Mutexes
Mutex compassMutex; 

void parseXML(char *buf);
void printList(LinkedList<node> list);
void clearList(LinkedList<node> list);
float getCompassValue();
void ThreadCompass(void const *args);
void ThreadBluetooth(void const *args);
void ThreadGPS(void const *args);
void compassTickerHandler();
void compassTickerCallback();
void initSensors();
void setup(void);
