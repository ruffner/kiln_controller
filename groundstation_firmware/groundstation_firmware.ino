// Groundstation firmware for AMTPS-FTA project
// Matt Ruffner 2021
// This software is meant to run on the receiving end of debug radio 
// and interpret in flight telemetry from the capsule, display on OLED 
// and log to SD card

// https://medium.com/@benjaminmbrown/real-time-data-visualization-with-d3-crossfilter-and-websockets-in-python-tutorial-dba5255e7f0e

#include <SPI.h>
#include <Wire.h>
//#include <U8g2lib.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_NeoPixel.h>
#include <FreeRTOS_SAMD21.h>
#include <SerialCommands.h>
#include <semphr.h>
#include <SD.h>

//#define DEBUG 1
#define DEBUG_RAD 1
//#define PRINT_RX_STATS 1

#include "include/delay_helpers.h" // rtos delay helpers
#include "include/config.h"        // project wide defs
#include "include/packet.h"        // data packet defs
#include "include/commands.h"      // command spec
#include "pins.h"                  // groundstation system pinouts

// freertos task handles
TaskHandle_t Handle_radTask;
TaskHandle_t Handle_serTask;

// freeRTOS semaphores
SemaphoreHandle_t dbSem; // serial debug logging (Serial)

#define SERIAL Serial
#define SCSERIAL Serial
#define RADIO_SERIAL Serial1

volatile bool newCmdToSend = false;
volatile bool newStrToSend = false;
 char strToSend[4] = "000";
volatile unsigned char cmdToSend;

// prototypes for serial commands
void printDirectory(SerialCommands* sender, File dir, int numTabs);

char serial_command_buffer_[32]; // max received command length

// serial command parser object
SerialCommands serial_commands_(&SCSERIAL, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");


// serial command handler for initiating a send of the C02 paraachute deploy
void cmd_send_pdep(SerialCommands* sender)
{
  sender->GetSerial()->print("sending parachute deploy...");
  cmdToSend = CMDID_DEPLOY_DROGUE;
  newCmdToSend = true;
  // do work
}


void cmd_set_0(SerialCommands* sender)
{
  sender->GetSerial()->println("Sending set0 (all off)");
  cmdToSend = '0';
  newCmdToSend = true;
}

void cmd_set_1(SerialCommands* sender)
{
  sender->GetSerial()->println("Sending set1");
  cmdToSend = '1';
  newCmdToSend = true;
}

void cmd_set_2(SerialCommands* sender)
{
  sender->GetSerial()->println("Sending set2");
  cmdToSend = '2';
  newCmdToSend = true;
}

void cmd_set_3(SerialCommands* sender)
{
  sender->GetSerial()->println("Sending set3");
  cmdToSend = '3';
  newCmdToSend = true;
}

void cmd_set_4(SerialCommands* sender)
{
  sender->GetSerial()->println("Sending set4");
  cmdToSend = '4';
  newCmdToSend = true;
}

void cmd_set_5(SerialCommands* sender)
{
  sender->GetSerial()->println("Sending set5");
  cmdToSend = '5';
  newCmdToSend = true;
}

void cmd_set_6(SerialCommands* sender)
{
  sender->GetSerial()->println("Sending set6");
  cmdToSend = '6';
  newCmdToSend = true;
}

void cmd_set_7(SerialCommands* sender)
{
  sender->GetSerial()->println("Sending set7");
  cmdToSend = '7';
  newCmdToSend = true;
}

void cmd_set_8(SerialCommands* sender)
{
  sender->GetSerial()->println("Sending set8");
  cmdToSend = '8';
  newCmdToSend = true;
}
void cmd_set_9(SerialCommands* sender)
{
  sender->GetSerial()->println("Sending set9");
  cmdToSend = '9';
  newCmdToSend = true;
}

void cmd_set_send(SerialCommands* sender)
{
  sender->GetSerial()->println("Sending set...");
  
  char* port_str = sender->Next();
  if (port_str == NULL)
  {
    sender->GetSerial()->println("ERROR value to set!");
    return;
  }

  if( strlen(port_str) != 3 ){
    sender->GetSerial()->println("ERROR set string must be [000]-[999] to set!");
    return;
  }

  sender->GetSerial()->print("Set string: ");
  sender->GetSerial()->println(port_str);

  memcpy(strToSend,port_str,3);
  newStrToSend = true;
}



// This is the default handler, and gets called when no other command matches. 
void cmd_help(SerialCommands* sender, const char* cmd)
{
  sender->GetSerial()->print("Unrecognized command [");
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}


SerialCommand cmd_set0_("0", cmd_set_0);
SerialCommand cmd_set1_("1", cmd_set_1);
SerialCommand cmd_set2_("2", cmd_set_2);
SerialCommand cmd_set3_("3", cmd_set_3);
SerialCommand cmd_set4_("4", cmd_set_4);
SerialCommand cmd_set5_("5", cmd_set_5);
SerialCommand cmd_set6_("6", cmd_set_6);
SerialCommand cmd_set7_("7", cmd_set_7);
SerialCommand cmd_set8_("8", cmd_set_8);
SerialCommand cmd_set9_("9", cmd_set_9);
SerialCommand cmd_set_("set", cmd_set_send);

unsigned long lastSendTime = 0;

void safeSend(char *printbuf, int dlen) {
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.write(printbuf, dlen);
    SERIAL.println();
    xSemaphoreGive( dbSem );
  }
}

/**********************************************************************************
 *  Radio task
*/
#define RBUF_SIZE 300
#define SBUF_SIZE 240

class RYLR896 {
public:
  RYLR896() {};
  static void thread( void *pvParam );
  static uint8_t rbuf[RBUF_SIZE];
  static char sbuf[SBUF_SIZE];
  static char printbuf[1000];
  static tlm_t rxtlm;
};
tlm_t RYLR896::rxtlm;
char RYLR896::printbuf[1000];
char RYLR896::sbuf[SBUF_SIZE];
uint8_t RYLR896::rbuf[RBUF_SIZE];
//void RYLR896::thread( void *pvParameters )


//char printbuf[100];
//char sbuf[SBUF_SIZE];
//uint8_t rbuf[RBUF_SIZE];
void RYLR896::thread( void *pvParameters )
//void radioThread( void *pvParameters )
{

  tlm_t dat;
  rxtlm_t outDat;

  /* 0: waiting for +OK from reset
     1: configuring address
     2:   waiting for address config +OK
     3: configuring network id
     4:   waiting for network id config +OK
     5: configuring rf params
     6:   waiting for rf param config +OK
     7: ready
  */
  int state = 0;  

  
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("Radio thread started");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  
  pinMode(PIN_LORA_RST, OUTPUT); // nreset of lora
  digitalWrite(PIN_LORA_RST, LOW);
  myDelayMs(100);
  digitalWrite(PIN_LORA_RST, HIGH);
  
  
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    SERIAL.println("Reset radio");
    xSemaphoreGive( dbSem );
  }
  #endif
  
  memset(rbuf, 0, RBUF_SIZE);
  memset(sbuf, 0, SBUF_SIZE);
  
  //RADIO_SERIAL.print("AT+RESET\r\n");
  
  while(1) {
  
    if( state == 1 ){
      int len = sprintf(sbuf, "AT+ADDRESS=2\r\n");
      RADIO_SERIAL.write(sbuf, len);
      state = 2;
    }
    else if( state == 3 ){
      int len = sprintf(sbuf, "AT+NETWORKID=1\r\n");
      RADIO_SERIAL.write(sbuf, len);
      state = 4;
    }
    else if( state == 5 ){
      int len = sprintf(sbuf, "AT+PARAMETER=10,7,1,7\r\n"); // recommended for less than 3km
      //int len = sprintf(sbuf, "AT+PARAMETER=12,4,1,7\r\n"); // recommended for more than 3km
      RADIO_SERIAL.write(sbuf, len);
      state = 6;
    }
  
    if( newCmdToSend && state == 7){
      // START SENDING 
      const int dataSize = sizeof(uint8_t);
      sprintf(sbuf, "AT+SEND=0,%d,", dataSize); // where 2 is the address
      int pre = strlen(sbuf);
         
      // embed command
      sbuf[pre] = cmdToSend;
      
      sbuf[pre+dataSize] = '\r';
      sbuf[pre+dataSize+1] = '\n';
      sbuf[pre+dataSize+2] = 0;
      
      
      #ifdef DEBUG_RAD
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        SERIAL.print("sending radio binary packet of size ");
        SERIAL.println(pre+dataSize+2);
        SERIAL.print("actual data was (bytes): ");
        SERIAL.println(dataSize);
        //SERIAL.println("DONE");
        xSemaphoreGive( dbSem );
      }
      #endif
      
      // send to lora module
      RADIO_SERIAL.write(sbuf, pre+dataSize+2);
      //SERIAL_LOR.write(sbuf, pre);
      SERIAL.write(sbuf, pre+dataSize+2);
      
      // go to state 8 so that we wait for a response
      state = 8;

      newCmdToSend = false;
    }

    if( newStrToSend && state == 7){
      // START SENDING 
      const int dataSize = sizeof(strToSend)-1; // dont include null byte
      sprintf(sbuf, "AT+SEND=0,%d,", dataSize); // where 2 is the address
      int pre = strlen(sbuf);
         
      // embed command
      memcpy(&sbuf[pre], strToSend,dataSize);
      //sbuf[pre] = cmdToSend;
      
      sbuf[pre+dataSize] = '\r';
      sbuf[pre+dataSize+1] = '\n';
      sbuf[pre+dataSize+2] = 0;
      
      
      #ifdef DEBUG_RAD
      if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
        SERIAL.print("sending radio binary packet of size ");
        SERIAL.println(pre+dataSize+2);
        SERIAL.print("actual data was (bytes): ");
        SERIAL.println(dataSize);
        //SERIAL.println("DONE");
        xSemaphoreGive( dbSem );
      }
      #endif
      
      // send to lora module
      RADIO_SERIAL.write(sbuf, pre+dataSize+2);
      //SERIAL_LOR.write(sbuf, pre);
      SERIAL.write(sbuf, pre+dataSize+2);
      
      // go to state 8 so that we wait for a response
      state = 8;

      newStrToSend = false;
    }

    // handle incoming message from LORA radio
    // AT message from module
    bool eol = false;
    int pos = 0;
    bool timeout = false;
    int sawComma = 0;
    unsigned long timeoutStart = 0;
    int expectedDataLen = 0;
    
    char rbts[4]; // receive buffer text length i.e "127"
    int rbtsLen = 0; // number of chars in rbts
    int payloadSize = 0;

    
    if( RADIO_SERIAL.peek() == '+' ){
      //SERIAL.print("in peek '");
      //SERIAL.write((char)RADIO_SERIAL.peek());
    
      unsigned long timeoutStart = xTaskGetTickCount();
      while(!eol && !timeout && pos < RBUF_SIZE-1) {

        if( RADIO_SERIAL.available() ){
          rbuf[pos] = RADIO_SERIAL.read();
          if( pos > 1 ){
           
            // look for payload length in receive at command
            // +RCV=50,5,HELLO,-99,40
            if( sawComma == 1 ){
              if( rbuf[pos] == ',' ){
                sawComma = 2;
                if( rbtsLen < 4 ){
                  rbts[rbtsLen] = 0;
                }
                payloadSize = atoi(rbts);
                
              } else {
                rbts[rbtsLen] = rbuf[pos];
                rbtsLen++;
              }
            }
            if( ( sawComma == 0) && ( rbuf[pos] == ',' ) ){
              sawComma = 1;
            }
          
            /*
            if( rbuf[pos]=='\n' && rbuf[pos-1]=='\r' && sawComma==2 ){
              memset(&rbuf[pos+1], 0, RBUF_SIZE-(pos+1));
              eol = true;
              Serial.println("saw eol");
            }
            */
            
            if( rbuf[pos]=='\n' && rbuf[pos-1]=='\r' ){
              memset(&rbuf[pos+1], 0, RBUF_SIZE-(pos+1));
              eol = true;
            }
          }
          
          if( pos++ >= RBUF_SIZE ){
            break;
          }
        }
        if( xTaskGetTickCount() - timeoutStart > 5000 ){
          memset(rbuf, 0, RBUF_SIZE);
          timeout = true;
        }
      }
    
    
      if( timeout ){
        #ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("Radio rx timed out");
          xSemaphoreGive( dbSem );
        }
        #endif
      } else if (!timeout && eol) {
        #ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("Radio got packet!");
          xSemaphoreGive( dbSem );
        }
        #endif
      } else if( !timeout && !eol) {
        #ifdef DEBUG
        if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
          SERIAL.println("Radio receive buffer overrun!");
          xSemaphoreGive( dbSem );
        }
        #endif
      }
      
      // if first byte is non-zero then we received data ( also check timeout and eol vars to be safe)
      // now process the data line received
      if( (rbuf[0] == '+') && !timeout && eol){
        
        
        int eqpos = 1;
        while( rbuf[eqpos] != '=' &&  eqpos < pos){
          eqpos++;
        }
        if( eqpos == pos ){
          #ifdef DEBUG
          if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
            SERIAL.print("We think we got a +READY or +OK message, we actually got: ");
            SERIAL.write(rbuf, pos);
            xSemaphoreGive( dbSem );
          }
          #endif
          
          if( state < 7 ){
            state ++;
            if( state == 7 ){
              #ifdef DEBUG_RAD
              if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
                SERIAL.println("STATE = 7, successfully configured radio!");
                xSemaphoreGive( dbSem );
              }
              #endif
              taskYIELD();
            }
          } else if( state > 7 ){
            state = 7;
            #ifdef DEBUG_RAD
            if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
              SERIAL.println("STATE was 8, received +OK from a data send operation!");
              xSemaphoreGive( dbSem );
            }
            #endif
            taskYIELD(); 
          }
          
        } else {
          // found an '=', parse rest of message
          #ifdef DEBUG
          if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
            SERIAL.print("We think we got a message with an '=' in it, we actually got: ");
            SERIAL.write(rbuf, pos);
            xSemaphoreGive( dbSem );
          }
          #endif
          
          
          // check if its a receive message
          if( rbuf[0]=='+' &&
              rbuf[1]=='R' &&
              rbuf[2]=='C' &&
              rbuf[3]=='V'){
            
            
            #ifdef PRINT_RX_STATS
            //int pblen = sprintf(printbuf, "Received %d bytes from address %d\n  rssi: %d, snr: %d\n", datalen, addr, rssi, snr);
            if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
              //SERIAL.write(printbuf, pblen);
              SERIAL.print("data: "); SERIAL.write(rbuf, pos);
              SERIAL.println();
              xSemaphoreGive( dbSem );
            }
            #endif
            
            // parse data
            // example rx string: +RCV=50,5,HELLO,-99,40
            const char *comma = ",";
            char *token;
            char *data;
            int rssi;
            int snr;
            int addr = -1;
            int datalen = -1;
            int cc = 0;
            
            // find start of data chunk
            int dataPos = 0;
            while( dataPos < pos){
              if( rbuf[dataPos] == ',' ){
                cc++;
                if( cc == 2 ){
                  dataPos++;
                  break;
                }
                  
              }
              dataPos++;
            }
            
            //SERIAL.print("dataPos is ");
            //SERIAL.println(dataPos);
            
            // assume that data coming from the capsule 
            // is a specific data structure for now
//            memcpy((void*)&rxtlm, (void *)&rbuf[dataPos], sizeof(tlm_t));
//
//   
            
            // parse target address
            token = strtok((char *) &rbuf[5], comma);
            addr = atoi(token);
            
            // extract data length
            token = strtok(NULL, comma);
            datalen = atoi(token);
            
            // get pointer to start of data 
            //data = strtok(NULL, comma);
            
            // get the rssi
            token = strtok((char *) &rbuf[8+datalen], comma);
            token = strtok(NULL, comma);
            rssi = atoi(token);
            
            // get the SNR
            token = strtok(NULL, comma);
            snr = atoi(token);


            Serial.write(&rbuf[dataPos], datalen);
//                      
//            
//            String latstr = String(rxtlm.lat);
//            String lonstr = String(rxtlm.lon);
//            String velstr = String(rxtlm.vel);
//            String alt_gpsstr = String(rxtlm.alt_gps);
//            String alt_barstr = String(rxtlm.alt_bar);
//            String barpstr = String(rxtlm.barp);
//            String tmpstr = String(rxtlm.tmp);
//            String batstr = String(rxtlm.bat);
//            String tcstr = "";
//            String nanstr = "nan";
//            
//            if( latstr.compareTo(nanstr) == 0 ){
//              latstr = "\"nan\"";
//            }
//            
//            for( int i=0; i<NUM_TC_CHANNELS; i++ ){
//              String istr = String(i+1);
//              String tstr = String(rxtlm.tc.data[i]);
//              if( tstr.compareTo("nan")==0 ){
//                tstr = "'nan'";
//              }
//              tcstr += "\"tc" + istr + "\":" + tstr;
//              if( i < NUM_TC_CHANNELS-1 ){
//                tcstr += ",";
//              }
//            }
//            String prstr = "";
//            for( int i=0; i<NUM_PRS_CHANNELS; i++ ){
//              String istr = String(i+1);
//              String pstr = String(rxtlm.prs.data[i]);
//              if( pstr.compareTo("nan")==0 ){
//                pstr = "'nan'";
//              }
//              prstr += "\"prs" + istr + "\":" + pstr;
//              if( i < NUM_PRS_CHANNELS-1 ){
//                prstr += ",";
//              }
//            }
//            
//            //{ timestamp: timestamp, value: this.state[id], id: id};
//            
//            // ported pressure sensor values
//            String istr = String(rxtlm.prs.data[0]);
//            int dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"p1\"}",rxtlm.t,istr.c_str());
//            safeSend(printbuf, dlen);
//
//            istr = String(rxtlm.prs.data[1]);
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"p2\"}",rxtlm.t,istr.c_str());
//            safeSend(printbuf, dlen);
//            
//            istr = String(rxtlm.prs.data[2]);
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"p3\"}",rxtlm.t,istr.c_str());
//            safeSend(printbuf, dlen);
//
//            istr = String(rxtlm.prs.data[3]);
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"p4\"}",rxtlm.t,istr.c_str());
//            safeSend(printbuf, dlen);
//
//            istr = String(rxtlm.prs.data[4]);
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"p5\"}",rxtlm.t,istr.c_str());
//            safeSend(printbuf, dlen);
//
//            // thermocouple values
//            istr = String(rxtlm.tc.data[0]);
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"tc1\"}",rxtlm.t,istr.c_str());
//            safeSend(printbuf, dlen);
//            istr = String(rxtlm.tc.data[1]);
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"tc2\"}",rxtlm.t,istr.c_str());
//            safeSend(printbuf, dlen);
//            istr = String(rxtlm.tc.data[2]);
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"tc3\"}",rxtlm.t,istr.c_str());
//            safeSend(printbuf, dlen);
//            istr = String(rxtlm.tc.data[3]);
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"tc4\"}",rxtlm.t,istr.c_str());
//            safeSend(printbuf, dlen);
//            istr = String(rxtlm.tc.data[4]);
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"tc5\"}",rxtlm.t,istr.c_str());
//            safeSend(printbuf, dlen);
//            istr = String(rxtlm.tc.data[5]);
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"tc6\"}",rxtlm.t,istr.c_str());
//            safeSend(printbuf, dlen);
//            istr = String(rxtlm.tc.data[6]);
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"tc7\"}",rxtlm.t,istr.c_str());
//            safeSend(printbuf, dlen);
//            istr = String(rxtlm.tc.data[7]);
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"tc8\"}",rxtlm.t,istr.c_str());
//            safeSend(printbuf, dlen);
//            istr = String(rxtlm.tc.data[8]);
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"tc9\"}",rxtlm.t,istr.c_str());
//            safeSend(printbuf, dlen);
//            istr = String(rxtlm.tc.data[9]);
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"tc10\"}",rxtlm.t,istr.c_str());
//            safeSend(printbuf, dlen);
//            istr = String(rxtlm.tc.data[10]);
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"tc11\"}",rxtlm.t,istr.c_str());
//            safeSend(printbuf, dlen);
//            istr = String(rxtlm.tc.data[11]);
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"tc12\"}",rxtlm.t,istr.c_str());
//            safeSend(printbuf, dlen);
//
//            // battery voltage
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"bat\"}",rxtlm.t,batstr.c_str());
//            safeSend(printbuf, dlen);
//
//            // iridium signal
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%d,\"id\":\"irsig\"}",rxtlm.t,rxtlm.irsig);
//            safeSend(printbuf, dlen);
//
//            // parachute deploy
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%d,\"id\":\"pardep\"}",rxtlm.t,(uint8_t)rxtlm.pardep);
//            safeSend(printbuf, dlen);
//
//            // capsule internal pressure
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"internal_pressure\"}",rxtlm.t,barpstr.c_str());
//            safeSend(printbuf, dlen);
//
//            // capsule internal pressure
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"internal_temperature\"}",rxtlm.t,tmpstr.c_str());
//            safeSend(printbuf, dlen);
//
//            // capsule barometer altitude
//            dlen = sprintf(printbuf, "{\"timestamp\":%d, \"value\":%s,\"id\":\"barometer_altitude\"}",rxtlm.t,alt_barstr.c_str());
//            safeSend(printbuf, dlen);

            // print out all received data in JSON format
            /*int dlen = sprintf(printbuf, "{\"time\": %d,\"lat\":%s,\"lon\":%s,\"vel\":%s,\"alt_gps\":%s,\"alt_bar\":%s,\"barp\":%s,\"tmp\":%s,\"bat\":%s,\"irsig\":%d,\"pardep\":%d,\"tc\":{%s},\"prs\":{%s}}",
                   rxtlm.t,
                   latstr.c_str(),
                   lonstr.c_str(),
                   velstr.c_str(),
                   alt_gpsstr.c_str(),
                   alt_barstr.c_str(),
                   barpstr.c_str(),
                   tmpstr.c_str(),
                   batstr.c_str(),
                   rxtlm.irsig,
                   (uint8_t)rxtlm.pardep,
                   tcstr.c_str(),
                   prstr.c_str());
            


            if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
              SERIAL.write(printbuf, dlen);
              SERIAL.println();
              xSemaphoreGive( dbSem );
            }
            */
            
            
          }
          
        }
      }
    }
    
    taskYIELD();
  }
  
  vTaskDelete( NULL );  
}

RYLR896 radio;

void serialTask( void *param ){

  serial_commands_.SetDefaultHandler(cmd_help);
	serial_commands_.AddCommand(&cmd_set0_);
	serial_commands_.AddCommand(&cmd_set1_);
  serial_commands_.AddCommand(&cmd_set2_);
  serial_commands_.AddCommand(&cmd_set3_);
  serial_commands_.AddCommand(&cmd_set4_);
  serial_commands_.AddCommand(&cmd_set5_);
  serial_commands_.AddCommand(&cmd_set6_);
  serial_commands_.AddCommand(&cmd_set_);
  
  while (1) {
    serial_commands_.ReadSerial();
  }
  
  vTaskDelete (NULL);

}

void setup() {
  SERIAL.begin(115200);
  delay(10);
  RADIO_SERIAL.begin(115200);
  delay(10);
  
  delay(4000);
  
  #if DEBUG
  SERIAL.println("Starting...");
  #endif

  // CREATE RTOS QUEUES 
  
  // setup debug serial log semaphore
  if ( dbSem == NULL ) {
    dbSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( dbSem ) != NULL )
      xSemaphoreGive( ( dbSem ) );  // make available
  }
  
  /**************
  * CREATE TASKS
  **************/
  xTaskCreate(RYLR896::thread, "Radio Control", 1000, NULL, tskIDLE_PRIORITY + 2, &Handle_radTask);
  xTaskCreate(serialTask, "Serial Interface", 1000, NULL, tskIDLE_PRIORITY + 2, &Handle_serTask);
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 4, &Handle_monitorTask);
  
  #if DEBUG
  SERIAL.println("Created tasks...");
  #endif
  
  delay(100);
  
  // start the scheduler
  vTaskStartScheduler();

  // error scheduler failed to start
  while(1)
  {
	  SERIAL.println("Scheduler Failed! \n");
	  delay(1000);
  }
  
}

void loop() {
  // tasks!
}
