//https://developer.mbed.org/users/hlipka/code/spxmltest_weather/file/ee591985c885/main.cpp
//https://developer.mbed.org/users/sam_grove/code/LinkedList/file/4ed66162aaa8/LinkedList.cpp
#include "main.h"

//Serial monitor
MODSERIAL pc(USBTX,USBRX);
//Bluetooth
MODSERIAL bluetooth(p13,p14);
//Serial bluetooth(p28,p27);
//GPS
MODSERIAL * gps_Serial;

//Compass
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

//Use the RPC enabled wrapped class  - see RpcClasses.h for more info
RpcDigitalOut leftTop(p21,"leftTop");
RpcDigitalOut leftBottom(p22,"leftBottom");
RpcDigitalOut rightTop(p23,"rightTop");
RpcDigitalOut rightBottom(p24,"rightBottom");

DigitalOut debug1(LED1);
DigitalOut debug2(LED2);
DigitalOut debug3(LED3);

void checkForInputInterrupt(void){
    threadBluetooth -> signal_set(0x1);
}

int main(){
    
    debug1 = 0;
    debug2 = 0;
    debug3 = 0;
    
    /* Initialise the Compass sensors */
//    initSensors();
    pc.baud(9600);
    
    //Baud rates
    bluetooth.baud(115200);
    
    //Start threads
    threadBluetooth = new Thread(ThreadBluetooth);
//    threadBluetooth -> set_priority(osPriorityHigh);// osPriorityAboveNormal , osPriorityHigh
//    threadCompass = new Thread(ThreadCompass);
//    threadCompass -> set_priority(osPriorityBelowNormal);
    threadGPS = new Thread(ThreadGPS);
//    threadGPS -> set_priority(osPriorityBelowNormal);
    
    char BigBuf[2000] = {0};
    strcpy(BigBuf, "<test vehicle=\"pedestrian\" start_lat=\"39.955081\" start_lon=\"-75.209975\" target_lat=\"39.952841\" target_lon=\"-75.191986\" complete_distance = \"1721.2902\" complete_time = \"1249.3289\"><segment id=\"12184012\" start=\"9\" end=\"11\" time = \"97.688\" rtime = \"0.0\" name = \"South 44th Street\" distance = \"128.73334\" turn = \"Go ahead\" turn_angle = \"0.0\" start_bearing = \"-168.11134\" end_bearing = \"-170.93776\" coordinates = \"39.95504175605655,-75.21005272865295\" description = \"Go ahead and go 128.73 meters\" /><segment id=\"12184012\" start=\"9\" end=\"11\" time = \"97.688\" rtime = \"0.0\" name = \"South 44th Street\" distance = \"128.73334\" turn = \"Go ahead\" turn_angle = \"0.0\" start_bearing = \"-168.11134\" end_bearing = \"-170.93776\" coordinates = \"39.95504175605655,-75.21005272865295\" description = \"Go ahead and go 128.73 meters\" /><segment id=\"12165855\" start=\"11\" end=\"0\" time = \"468.0327\" rtime = \"364.52325\" name = \"Locust Street\" distance = \"643.10095\" turn = \"Turn left\" turn_angle = \"-89.73518\" start_bearing = \"99.32706\" end_bearing = \"99.21103\" coordinates = \"39.95389858630569,-75.21028876304626;39.953734099876996,-75.20900130271912;39.95363540782989,-75.20822882652283;39.95358606175295,-75.20778894424438;39.95356138870112,-75.20759582519531;39.953520266928294,-75.20726323127747;39.953446247674904,-75.20667314529419;39.953281760158355,-75.20533204078674;39.953125496651225,-75.20410895347595;39.953092599025275,-75.2037763595581;39.95299390605251,-75.20293951034546\" description = \"Turn left and go 784.97 meters\" /></test>" );
    parseXML(BigBuf);
//    printList(waypointList);
}

void ThreadBluetooth(void const *args)
{
    char outbuf[80], buf[80];
    while (1) {
        if(bluetooth.readable()){
            debug2 = !debug2;
//            debug2 = 0;
            bluetooth.gets(buf, 80);
            pc.printf("%s\n", buf);
            RPC::call(buf, outbuf);
            bluetooth.puts(outbuf);    
        }
    }
}

void ThreadGPS(void const *args)
{
    gps_Serial = new MODSERIAL(p13,p14); //serial object for use w/ GPS
    Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
    char c; //when read via Adafruit_GPS::read(), the class returns single character stored here
    Timer refresh_Timer; //sets up a timer for use in loop; how often do we print GPS info?
    const int refresh_Time = 5000; //refresh time in ms
    
    myGPS.begin(9600);  //sets baud rate for GPS communication; note this may be changed via Adafruit_GPS::sendCommand(char *)
                        //a list of GPS commands is available at http://www.adafruit.com/datasheets/PMTK_A08.pdf
    
    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //these commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    myGPS.sendCommand(PGCMD_ANTENNA);
    Thread::wait(1000);
    refresh_Timer.start();  //starts the clock on the timer
    
    while (1) {
        if(!gps_Serial->readable()){
            Thread::yield();
//            debug3 = 1;
        }
        else{
//            debug3 = !debug3;
            c = myGPS.read();   //queries the GPS            
            if (c) { pc.printf("%c", c); } //this line will echo the GPS data if not paused
            
            //check if we recieved a new message from GPS, if so, attempt to parse it,
            if ( myGPS.newNMEAreceived() ) {
                if ( !myGPS.parse(myGPS.lastNMEA()) ) {
                    continue;   
                }    
            }
            //check if enough time has passed to warrant printing GPS info to screen
            //note if refresh_Time is too low or pc.baud is too low, GPS data may be lost during printing
            if (refresh_Timer.read_ms() >= refresh_Time) {
                debug3 = !debug3;
                refresh_Timer.reset();
                Thread::wait(50);
                if (myGPS.fix) {
    
                }
            }
        }
    }
}

void ThreadCompass(void const *args)
{
    setup();
    compassTicker.attach(&compassTickerCallback, 5); // the address of the function to be attached (flip) and the interval (2 seconds)   
    while(1)
    {
        compassMutex.lock();
        if(compassTriggered){
            debug2 = !debug2;       
            compassTickerHandler();
        }
        compassMutex.unlock();
    }
}

float getCompassValue(){
    float result = 0.0;
    compassMutex.lock();
    result = compassValue;
    compassMutex.unlock();
    return result; 
}

void parseXML(char *buf){
    SP_XmlDomParser parser;
    int waypointId = 0;
    long segmentNum;
    parser.append( buf, strlen(buf));
    
    //Clear the previos segment and waypoint lists
    clearList(segmentList);
    clearList(waypointList);
    
    SP_XmlHandle rootHandle( parser.getDocument()-> getRootElement() );
    SP_XmlElementNode *root = rootHandle.toElement();
//    pc.printf("Root = %s\n",root -> getName());
    SP_XmlHandle handle(root);
    const SP_XmlNodeList *nodeList = root -> getChildren();
    int i;
    for( i = 0; i < nodeList -> getLength(); i++){
        SP_XmlHandle handle(nodeList -> get(i));
        SP_XmlElementNode *tempc =  handle.toElement();
        if (tempc) {
//            pc.printf("Points = %s\n",tempc -> getName());
//            pc.printf("%s,%s,%s,%s,%s,%s,%s,%s\n",tempc -> getAttrValue("id"),tempc -> getAttrValue("coordinates")
//                , tempc -> getAttrValue("turn"), tempc -> getAttrValue("turn_angle"), tempc -> getAttrValue("name")
//                ,tempc -> getAttrValue("start_bearing"), tempc -> getAttrValue("end_bearing"), tempc -> getAttrValue("distance")
//                ,tempc -> getAttrValue("time"));
                
            //Create the segment list from the xml    
            Segment *segment = (Segment*) malloc (sizeof(Segment));
            segment -> segId = atol(tempc -> getAttrValue("id"));
            segmentNum = segment -> segId;
            strcpy(segment -> name, tempc -> getAttrValue("name"));
            segment -> time = atof(tempc -> getAttrValue("time"));
            segment -> distance = atof(tempc -> getAttrValue("distance"));
            strcpy(segment -> turn, tempc -> getAttrValue("turn"));
            segment -> turnAngle = atof(tempc -> getAttrValue("turn_angle"));
            segment -> startBearing = atof(tempc -> getAttrValue("start_bearing"));
            segment -> endBearing = atof(tempc -> getAttrValue("end_bearing"));
            segmentList.append(segment);
            
            //Create the waypoint list from the xml
            char coordinates[1000];
            strcpy(coordinates, tempc -> getAttrValue("coordinates"));
            char *end_str;
            char *token = strtok_r(coordinates, ";", &end_str);
            int flag = 0;
            while( token != NULL ) 
            {
                flag = 0;
                char *end_token;
                char *token1 = strtok_r(token, ",", &end_token);
                Waypoint *waypoint = (Waypoint*) malloc (sizeof(Waypoint));
                waypoint -> waypointId = waypointId++;
                waypoint -> segmentNum = segmentNum;
                while( token1 != NULL ) 
                {
//                    printf( " %s,", token1);
                    if(flag == 0){
                       waypoint -> latitude = atof(token1);
                       flag = 1; 
                    }
                    else{
                       waypoint -> longitude = atof(token1); 
                       flag = 0;
                    }
                    token1 = strtok_r(NULL, ",", &end_token);
                }
                waypointList.append(waypoint);
//                printf( "\n");
                token = strtok_r(NULL, ";", &end_str);
            }
        }
    }
    if ( NULL != parser.getError() ) {
        pc.printf( "\n\nerror: %s\n", parser.getError() );
    }
}

void clearList(LinkedList<node> list){
    for(int i=1; i<=list.length(); i++){
        tmp = list.remove(i);
    }
}

void printList(LinkedList<node> list){
    pc.printf("Size: %d\n", list.length() );
    for(int i=1; i<=list.length(); i++){
        tmp = list.pop(i);
        pc.printf("SegmentNum: %ld Lat: %f\n", ((Waypoint *)(tmp -> data)) -> segmentNum, ((Waypoint *)(tmp -> data)) -> latitude);
    }
}

void initSensors(){
  if(!accel.begin()){
   /* There was a problem detecting the LSM303 ... check your connections */
    pc.printf(("Ooops, no LSM303 detected ... Check your wiring!\n"));
    while(1);
  }
  
  if(!mag.begin()){
    /* There was a problem detecting the LSM303 ... check your connections */
    pc.printf("Ooops, no LSM303 detected ... Check your wiring!\n");
    while(1);
  }
}

void setup(void){
    pc.printf(("Adafruit 9 DOF Pitch/Roll/Heading Example\n"));
    pc.printf("\n");
    
    /* Initialise the sensors */
    initSensors();
}

void compassTickerCallback() {
    compassTriggered = true;
}

void compassTickerHandler() {

    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_vec_t   orientation;

    /* Calculate pitch and roll from the raw accelerometer data */
    accel.getEvent(&accel_event);
    if (dof.accelGetOrientation(&accel_event, &orientation)){
        /* 'orientation' should have valid .roll and .pitch fields */
        pc.printf(("Roll: "));
        pc.printf("%f",orientation.roll);
        pc.printf(("; "));
        pc.printf(("Pitch: "));
        pc.printf("%f",orientation.pitch);
        pc.printf(("; "));
    }

    /* Calculate the heading using the magnetometer */
    mag.getEvent(&mag_event);
    if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)){
        /* 'orientation' should have valid .heading data now */
        pc.printf(("Heading: "));
        pc.printf("%f", orientation.heading);
        pc.printf(("; "));
    }
    
    pc.printf((""));
    compassTriggered =  false;
}
