//I borrowed code from: https://torque-bhp.com/wiki/Arduino_code
//The intent of this code is to facilitate the customization
//of an OBD interface for Torque. Extra hardware is needed with the arduino
//in order to interface with the car, for instance a CAN shield
//or in this example, an LM339 comparator used as a 5V to 12V
//voltage level shifter. This example includes commands for ISO-9141

#include <AltSoftSerial.h>//to interface with OBD
#include <SoftwareSerial.h>//to interface with the HC05 bluetooth shield
#include <String.h>

#define bluetoothRx 2 //these pins can be adjusted in case you need to, but watch out for incompatibility with the AltSoftSerial functions
#define bluetoothTx 3
#define OBD_Rx 8 //these pins are fixed for the Arduino UNO and AltSoftSerial library, pin 10 is also obviated for interrupt behavior
#define OBD_Tx 9
#define byteTimeout 1000 //milliseconds


/**
 * Torque for Android Arduino sample program by Ian Hawkins <ian@torque-bhp.com> http://torque-bhp.com/
 * You will require Torque Pro 1.8.82 or newer
 * 
 * Setup the bluetooth device as an OBD2 adapter in Torque.  Torque will automatically recognise the Arduino and will
 * import sensors automatically based on the configuration settings below.
 * 
 * This code is released under the LGPL v2 
 * 
 * This has been tested on the Arduino UNO board
 * 
 * Connect pins 2 and 3 to your bluetooth device's rx and tx pins (2 to txd on bluetooth device, 3 to rxd)
 * 
 * This code allows you to gather sensors from the arduino to display in the app. It can be extended to whatever you want.
 */
 
//commands for ELM327 are 'AT' commands
//many of the strings here are ELM327 commands, but some are sent by the Torque app or are mis-read spots
const String ATE = "ATE"; // Echo off/on if followed by 0 or 1
const String ATI = "ATI"; // Version id
const String ATZ = "ATZ"; // Reset all
const String ATS = "ATS"; // Set protocol X
const String ATH = "ATH"; // Headers off / on
const String ATL = "ATL"; // Linefeeds off/on
const String ATM = "ATM"; // Memory off/on
const String PROMPT = ">"; //this is the command from an ELM327 to Torque that we are ready for another request
const String ISO9141 = "3";// ISO 9141-2 (5 baud init, 10.4 kbaud)
const String ATDPN = "ATDPN"; //Describe the Protocol by Number
const String ATDESC = "AT@1";
const String ATAT = "ATAT";
const String LF = "\n";//line feed
//const String VERSION = "Torque Protocol Interface v0.0.1"; // Don't change this - it's used by Torque so it knows what interface it is connected to
const String VERSION = "ELM327 v2.1";//let's pretend to be the ELM327!
const String VERSION_DESC = "Torque For Android Protocol Interface";
const String OK = "OK";//we must respond to command settings with 'OK' to ack the setting change
const String SLOW_INIT = "ATSI";//the command telling us to perform a slow init for ISO-9141

//strange string commands used for special Arduino analog or digital sensors/actuators- we're not using these
const String GETDEFINITIONS = "GETDEFINITIONS"; // Get sensor definitions
const String GETCONFIGURATION = "GETCONFIGURATION"; // Get config of app (hide car sensors, devices sensors, etc)
const String GETSENSORS = "G"; // Get sensor values, one shot.
const String SETSENSOR = "S"; // Set a sensor value
const String CONFIGURATION = "NO_CAR_SENSORS,NO_DEVICE_SENSORS"; 

 
String fromTorque = "";//a memory buffer to read in commands from Torque
 
/**
 * Configuration directives for the app to hide various things. Comma separated. Remove to enable visibility in Torque
 *  - handy if your project isn't car related or you want to make sensor selections relatively easy.
 *  
 *  Supported types:
 *    NO_CAR_SENSORS  - hide any car related sensors
 *    NO_DEVICE_SENSORS - hide any device (phone) sensors
 *    
 */

 
// Setup bluetooth module on pins 2 and 3 (you can't use these digial pins in the sensor list or it'll break comms)
SoftwareSerial mySerial(bluetoothRx,bluetoothTx); // define the serial port interface with the bluetooth module

//  Setup OBD interface with LM339 voltage level shifter
AltSoftSerial OBD_K_line;
const int OBD_baud = 10400;//baud rate for ISO-9141

//let's abuse global variables!!
boolean initDone = false;// a flag to keep track of the state of our init
uint8_t OBD_msg_buf_send[8] = {0x68,0x6A,0xF1,0x0,0x0,0x0,0x0,0x0};//try not to change the 0x3 header bytes
uint8_t OBD_msg_send_len = 3;

uint8_t OBD_msg_buf_read[10] = {0x48,0x6B,0x10,0x41,0,0,0,0,0,0};//these bytes are for Mode 0x1, but will change with each read event
uint8_t OBD_msg_read_len = 0;

void setup() {
  Serial.begin(9600);    //we'll use the native Serial interface for troubleshooting code
  delay(600);
  mySerial.begin(9600);  //and we'll use the SoftwareSerial interface for bluetooth comms
  delay(600);
  OBD_K_line.begin(OBD_baud);//and we'll use an altSoftSerial interface for OBD            
 }
 
void loop() {
 
  /**
   * Grab data from the bluetooth module, parse it.
   */
  if (mySerial.available()) {
     char c = mySerial.read();   
     if ((c == '\n' || c == '\r') && fromTorque.length() > 0) {
        fromTorque.toUpperCase();
        processCommand(fromTorque);
        fromTorque = "";
     } else if (c != ' ' && c != '\n' && c !='\r') {
        // Ignore spaces, but continue reading in characers until we get a line feed or carriage return
        fromTorque += c; 
     }
  }
}
 
/**
 * Parse the commands sent from Torque
 */
void processCommand(String command) {
 
   // Debug - see what torque is sending on your serial monitor
   Serial.println(command);
 
   // Simple command processing from the app to the arduino..
   // check for any possible AT command for the ELM327
   if (command.equals(ATZ)) {
      //reset command
       mySerial.print(VERSION);
       mySerial.print(LF); 
       mySerial.print(OK);
   } else if (command.startsWith(ATE)) {
      //set echo on or off with ATE1 or ATE0
       mySerial.print(OK); 
   } else if(command.startsWith(ATI)) {
      //print the version ID
       mySerial.print(VERSION);
       mySerial.print(LF);
       mySerial.print(OK);
   } else if (command.startsWith(ATDESC)) {
      //this does not appear to be an ELM327 response, but we can respond anyway
       mySerial.print(VERSION_DESC); 
       mySerial.print(LF);
       mySerial.print(OK);
   } else if (command.startsWith(ATL)) {
      //print line feed on or off is ATL1 or ATL0
       mySerial.print(OK);
   } else if (command.startsWith(ATAT)) {
      //this does not appear to be an ELM327 command
       mySerial.print(OK);
   } else if (command.startsWith(ATH)) {
      //print header on or off is ATH1 or ATH0
       mySerial.print(OK);
   } else if (command.startsWith(ATM)) {
      //monitor all ATMA, monitor receive ATMR, monitor transmit ATMT
       mySerial.print(OK);
   } else if (command.startsWith(ATS)) {
       // Set protocol - we really don't want to change the protocol unless our hardware supports more than one type
       //I've only included the ISO-9141 stuff, though.
       mySerial.print(OK);
   } else if (command.startsWith(ATDPN)) {
      //print the currently selected protocol number
       mySerial.print(ISO9141);
   } else if (command.startsWith(SLOW_INIT)) {
      //we should perform the slow init
      slowInit();
   } else {
      //anything else must be a command for a PID read!
      uint8_t numBytes_in_command = update_OBD_msg_buff(command);//the command will not be preceeded with 'AT' when we're requesting mode/PID data

      //send the init if this is not done yet
      if(!initDone)
      {
        slowInit();
      }

      //now perform the send and receive cycle
      mySerial.end();//disable the interrupts involved with SoftwareSerial

      send_rec_OBD_msg();
            
      //finally, report this response to Torque via the bluetooth adapter
      mySerial.begin(9600);//re-enable the bluetooth command line
      //then send the response
      for(uint8_t i = 3 + numBytes_in_command; i < OBD_msg_read_len - 1; i++)
      {
        //print all bytes past the header, excluding the checksum
        //this may need modified in case the ELM327 commands requesting to see the header/sum are invoked
        mySerial.print(OBD_msg_buf_read[i],HEX);
      }
   }
 
   mySerial.print(LF); //when we're done responding, print a line feed
   mySerial.print(PROMPT); //then print the prompt to get the next command
 
 
}


void slowInit() {
  //perform the slow init sequence for ISO-9141
  //send 0x33 at 5 baud, then at 10.4 kbaud read 3 bytes (0x55 timing byte, then the key twice), twiddle the final byte and send it back
  //finally read in the confirm response
  //0x33 = 00110011
  boolean oh_x_thirtythree[10] = {0,1,1,0,0,1,1,0,0,1};//write a start bit, then the 8 bits from lowest order to highest, then a stop bit
  //stop the AltSoftSerial interface so we can bit-bang these out
  OBD_K_line.end();
  for(uint8_t i = 0; i<10; i++)
  {
    digitalWrite(OBD_Tx,oh_x_thirtythree[i]);//write the pin state
    delay(200);//5 baud is 200 ms per bit
  }
  //then restart the AltSoftSerial interface
  OBD_K_line.begin(OBD_baud);//this had better happend fast...
  uint8_t response[4] = {0,0,0,0};
  uint8_t index = 0;

  //loop to read in a response, but mind a timeout
  uint32_t timeout = millis();
  while(millis() - timeout < byteTimeout || index < 3)
  {
    if(OBD_K_line.available())
    {
      response[index] = OBD_K_line.read();
      index += 1;
      timeout = millis();
    }
  }

  //then bitwise invert the final byte and send it to the ECU
  uint8_t send_back = ~response[3];
  OBD_K_line.print(send_back);
  timeout = millis();

  //then read in one more byte
  while(millis() - timeout < byteTimeout || index < 4)
  {
    if(OBD_K_line.available())
    {
      response[index] = OBD_K_line.read();
      index += 1;
      timeout = millis();
    }
  }

  //then set a flag that we've completed the init successfully
  if(response[3] == 0xCC)
  {
    initDone = true;
  }
  else
  {
    initDone = false;
  }
}

uint8_t checksum(uint8_t *msg_buffer, uint8_t buf_leng)
{
  //this function calculates the sum of (buf_leng-1) bytes in the input msg_buffer
 
  uint8_t sum = 0;
  //this function computes the checksum
  for(uint8_t i = 0; i < buf_leng - 1; i++)
  {
    sum+= msg_buffer[i];
  }
  return sum;
}
uint8_t update_OBD_msg_buff(String newMode_PID)
{
  //this function parses the input string as if it were hexadecimal MODE and PID bytes,
  //then it puts them after the header in the OBD command message buffer
  //finally it updates the checksum and message length global variable
  
  //a character array with with to convert from hexadecimal characters to integers
  char hexChars[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
  
  //reset the message length accounting
  OBD_msg_send_len = 3;//we don't want to overwrite the header!
  uint8_t numCommandBytes = 0;//keep track of the number of command bytes we're adding
  
  //first, parse the string: assume no strings and that all data comes in sets of two characters
  uint8_t parsedData = 0;
  for(int i = 0; i < newMode_PID.length(); i++)
  {
    char thisChar = newMode_PID.charAt(i);//read the current character
    //loop over all possible hex characters to find a match
    for(uint8_t j = 0; j < 16; j++)
    {
      if(hexChars[j] == thisChar)
      {
        parsedData = (parsedData << 4) + j;//then shift the previous character and add the new one
        break;
      }
    }

    if(i % 2 == 1)
    {
      //we've read an entire character, so put it into the message buffer
      OBD_msg_buf_send[OBD_msg_send_len] = parsedData;
      parsedData = 0;
      OBD_msg_send_len += 1;
      numCommandBytes += 1;
    }
  }

  //we've finished reading in and parsing the string data
  //update the checksum
  OBD_msg_send_len += 1;//add a byte space for the checksum
  uint8_t sum = checksum(OBD_msg_buf_send,OBD_msg_send_len);
  OBD_msg_buf_send[OBD_msg_send_len-1] = sum;

  return numCommandBytes;
}


void send_rec_OBD_msg()
{
  //this function sends what ever is in the send buffer and then reads back
  //a response into the read buffer; if the read fails then we'll change the init status
  
  //send the data requesting OBD data via the ISO-9141 protocol from the car
  for(uint8_t i = 0; i < OBD_msg_send_len; i++)
  {
    OBD_K_line.write(OBD_msg_buf_read[i]);
  }
  
  //then read in a response from the car until we read the checksum or reach a timeout
  OBD_msg_read_len = 0;
  boolean sumRead = false;
  uint32_t timeout = millis();
  uint8_t sum = 0;
  while(millis() - timeout < byteTimeout)
  {
    if(OBD_K_line.available())
    {
      OBD_msg_buf_read[OBD_msg_read_len] = OBD_K_line.read();
      OBD_msg_read_len += 1;//increment the index to read in the next byte
      
      if(sum == OBD_msg_buf_read[OBD_msg_read_len - 1])
      {
        //we just read in the sum byte so we're done
        sumRead = true;
        break;
      }
      sum += OBD_msg_buf_read[OBD_msg_read_len - 1];//add the byte we just read in to our account of the sum
      delay(1);//it takes 962 microseconds to read in a byte, do we need a delay for better data fidelity?
      timeout = millis();
    }
  }

  if(!sumRead)
  {
    initDone = false;//we reached here after a timeout, so we will probably need to re-init
  }
}

