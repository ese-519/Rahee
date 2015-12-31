 #include "mbed.h"
 #include "string"
 #include "SDHCFileSystem.h"
 #include "MBed_Adafruit_GPS.h"
 
Serial * gps_Serial;
Serial pc (USBTX, USBRX);
SDFileSystem sd(p5, p6, p7, p8, "sd"); // mosi, miso, sclk, cs
   
 int main() {
    printf("Hello World!\n");   
 
    mkdir("/sd/mydir", 0777);
    
    FILE *fp = fopen("/sd/mydir/sdtest.txt", "a");
    if(fp == NULL) {
        error("Could not open file for write\n");
    }
    fprintf(fp, "Hello fun SD Card World!\n");
    
    pc.baud(115200); //sets virtual COM serial communication to high rate; this is to allow more time to be spent on GPS retrieval
    
    gps_Serial = new Serial(p28,p27); //serial object for use w/ GPS
    Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
    char c; //when read via Adafruit_GPS::read(), the class returns single character stored here
    Timer refresh_Timer; //sets up a timer for use in loop; how often do we print GPS info?
    const int refresh_Time = 5000; //refresh time in ms
    
    myGPS.begin(9600);  //sets baud rate for GPS communication; note this may be changed via Adafruit_GPS::sendCommand(char *)
                        //a list of GPS commands is available at http://www.adafruit.com/datasheets/PMTK_A08.pdf
    
    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //these commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    myGPS.sendCommand(PGCMD_ANTENNA);
    
    pc.printf("Connection established at 115200 baud...\n");
    
    wait(1);
    
    refresh_Timer.start();  //starts the clock on the timer
    
    while(true){
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
            refresh_Timer.reset();
            FILE *fp = fopen("/sd/mydir/sdtest.txt", "a");
            if(fp == NULL) {
                error("Could not open file for write\n");
            }
            fprintf(fp, "-------------------------------------\n");
            wait(0.05);
            fprintf(fp, "Fix: %d Quality: %d\n", (int) myGPS.fix, (int) myGPS.fixquality);
            wait(0.05);
            if (myGPS.fix) {
                fprintf(fp, "Location: %5.14f%c, %5.14f%c\n", myGPS.latitude, myGPS.lat, myGPS.longitude, myGPS.lon);
                wait(0.05);
                fprintf(fp, "Speed: %5.2f knots\n", myGPS.speed);
                wait(0.05);
                fprintf(fp, "Angle: %5.7f\n", myGPS.angle);
                wait(0.05);
                fprintf(fp, "Altitude: %5.7f\n", myGPS.altitude);
                wait(0.05);
                fprintf(fp, "Satellites: %d\n", myGPS.satellites);
                wait(0.05);
            }
            fclose(fp);
        }
    }
 }