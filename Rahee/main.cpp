#include "mbed.h"
#include "rtos.h"
#include "mbed_rpc.h"
#include "string.h"

/**
 *  This example program has been updated to use the RPC implementation in the new mbed libraries.
 *  This example demonstrates using RPC over serial
 */
RpcPwmOut leftTop(p21,"leftTop");
RpcPwmOut leftBottom(p25,"leftBottom");
RpcPwmOut rightTop(p23,"rightTop");
RpcPwmOut rightBottom(p24,"rightBottom");
Serial pc(p28, p27);
InterruptIn markPoint(p17);


Serial usb(USBTX, USBRX); // tx, rx

Thread *threadRPC;
extern "C" void mbed_reset();
void initSeq();

// Initialize a pins to perform analog input and digital output fucntions
AnalogIn   flex(p20);
DigitalOut press(LED1);
DigitalOut mark(LED2);
DigitalOut led(LED3);

int count = 0;
bool isMarked = false;
int markCount = 0;

void marking() {
    markCount++;
}

int main() {

    pc.baud(115200);
//    threadRPC = new Thread(ThreadRPC);
    char buf[256], outbuf[256];
    RPC::call("/leftTop/period 2\n\r", outbuf);
    RPC::call("/leftBottom/period 2\n\r", outbuf);
    RPC::call("/rightTop/period 2\n\r", outbuf);
    RPC::call("/rightBottom/period 2\n\r", outbuf);
    
    markPoint.rise(&marking);
    
    while(1){
        
        if(pc.readable()){
            pc.gets(buf, 256);
            //Call the static call method on the RPC class
            usb.printf(buf);
            if(strcmp(buf, "reset\n")==0){
                usb.printf("Reset called\n");
                pc.printf("Device reset sucessfully.\n");
                initSeq();
                mbed_reset();
            }else{
                RPC::call(buf, outbuf); 
                pc.printf("%s\n", outbuf);
            }
        }
        
        if(markCount > 0){
            count++;
            if(count == 4){
                usb.printf("Mark Point\n");
                pc.printf("marked\n");
                count = 0;    
            }
            markCount = 0;
        }
    }
}

void initSeq(){
    char outbuf[256];
    RPC::call("/leftTop/write 1\n\r", outbuf);
    wait(0.5);
    RPC::call("/leftBottom/write 1\n\r", outbuf);
    wait(0.5);
    RPC::call("/rightTop/write 1\n\r", outbuf);
    wait(0.5);
    RPC::call("/rightBottom/write 1\n\r", outbuf);
    wait(0.5);
    
    RPC::call("/leftTop/write 0\n\r", outbuf);
    wait(0.5);
    RPC::call("/leftBottom/write 0\n\r", outbuf);
    wait(0.5);
    RPC::call("/rightTop/write 0\n\r", outbuf);
    wait(0.5);
    RPC::call("/rightBottom/write 0\n\r", outbuf);
    
}



  
