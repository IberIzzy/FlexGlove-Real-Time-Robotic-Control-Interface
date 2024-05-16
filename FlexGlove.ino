#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

#include "stdio.h" //DOBOTSETUP
#include "Protocol.h"
#include "command.h"
#include "FlexiTimer2.h"

#define SERIAL_TX_BUFFER_SIZE 64 //DOBOT SETUP
#define SERIAL_RX_BUFFER_SIZE 256

LiquidCrystal lcd(12, 11, 5, 4, 3, 2); 

const int Pinky = A0; //Five Finger analog pins
const int Middle = A2;
const int Ring = A1; 
const int Index = A3; 
const int Thumb = A4;

const int THUMB_NUM = 1 << 4;  //FOUND ONLINE
const int INDEX_NUM = 1 << 3;  
const int MIDDLE_NUM = 1 << 2; 
const int RING_NUM = 1 << 1;   
const int PINKY_NUM = 1;  

const int Switch_pin = 13;  //sideswich
const int Piezo = 8; //piezo

int value_Pinky; //Five fingers
int value_Index;
int value_Ring;
int value_Thumb;
int value_Middle;

bool previousState; //State for piezo sound

EndEffectorParams gEndEffectorParams; //DOBOT SETUP

JOGJointParams  gJOGJointParams;
JOGCoordinateParams gJOGCoordinateParams;
JOGCommonParams gJOGCommonParams;
JOGCmd          gJOGCmd;

PTPCoordinateParams gPTPCoordinateParams;
PTPCommonParams gPTPCommonParams;
PTPCmd          gPTPCmd;

uint64_t gQueuedCmdIndex;

void setup() 
{
 pinMode(Switch_pin, INPUT_PULLUP);  //side switch 
 pinMode(Piezo, OUTPUT); //piezo
 //Serial.begin(9600); 
 //Serial.println("ENTER AT Commands:");
 // BTSerial.begin(9600);
 lcd.begin(16, 2); //LCD screen

  Serial.begin(115200); //DOBOT SETUP
  Serial1.begin(115200); 
  printf_begin();
  //Set Timer Interrupt
  FlexiTimer2::set(100,Serialread); 
  FlexiTimer2::start();
}

void Serialread() //DOBOT FUNCTION
{
  while(Serial1.available()) {
        uint8_t data = Serial1.read();
        if (RingBufferIsFull(&gSerialProtocolHandler.rxRawByteQueue) == false) {
            RingBufferEnqueue(&gSerialProtocolHandler.rxRawByteQueue, &data);
        }
  }
}

int Serial_putc( char c, struct __file * ) //DOBOT FUNCTION
{
    Serial.write( c );
    return c;
}

void printf_begin(void) //DOBOT FUNCTION
{
    fdevopen( &Serial_putc, 0 );
}

void InitRAM(void) //DOBOT FUNCTION
{
    //Set JOG Model
    gJOGJointParams.velocity[0] = 100;
    gJOGJointParams.velocity[1] = 100;
    gJOGJointParams.velocity[2] = 100;
    gJOGJointParams.velocity[3] = 100;
    gJOGJointParams.acceleration[0] = 80;
    gJOGJointParams.acceleration[1] = 80;
    gJOGJointParams.acceleration[2] = 80;
    gJOGJointParams.acceleration[3] = 80;

    gJOGCoordinateParams.velocity[0] = 100;
    gJOGCoordinateParams.velocity[1] = 100;
    gJOGCoordinateParams.velocity[2] = 100;
    gJOGCoordinateParams.velocity[3] = 100;
    gJOGCoordinateParams.acceleration[0] = 80;
    gJOGCoordinateParams.acceleration[1] = 80;
    gJOGCoordinateParams.acceleration[2] = 80;
    gJOGCoordinateParams.acceleration[3] = 80;

    gJOGCommonParams.velocityRatio = 50;
    gJOGCommonParams.accelerationRatio = 50;
   
    gJOGCmd.cmd = AP_DOWN;
    gJOGCmd.isJoint = JOINT_MODEL;

    //Set PTP Model
    gPTPCoordinateParams.xyzVelocity = 100;
    gPTPCoordinateParams.rVelocity = 100;
    gPTPCoordinateParams.xyzAcceleration = 80;
    gPTPCoordinateParams.rAcceleration = 80;

    gPTPCommonParams.velocityRatio = 50;
    gPTPCommonParams.accelerationRatio = 50;

    gPTPCmd.ptpMode = MOVL_XYZ;
    gPTPCmd.x = 200;
    gPTPCmd.y = 0;
    gPTPCmd.z = 0;
    gPTPCmd.r = 0;

    gQueuedCmdIndex = 0;

}

bool ON_OFF() //side switch reader
{
  int switch_read = digitalRead(Switch_pin);

  if (switch_read == HIGH)
  {
    return true;
  }

  else
  {
    return false;
  }

}

void On_sound() //on sound values
{
  tone(Piezo, 500, 200); 
  delay(150);        
  tone(Piezo, 600, 300); 
  delay(150);
  tone(Piezo, 700, 400); 
}

void Off_sound() //off sound values
{
  tone(Piezo, 700, 400);
  delay(150);
  tone(Piezo, 600, 300); 
  delay(150);
  tone(Piezo, 500, 200); 
}

void RobotControl(int num) //Dobot integration 
{
  int fingerState = num;

  if (fingerState == 16)
      {
        //MOVE BACK
        gPTPCmd.x -= 20;
        gPTPCmd.y += 0;
        gPTPCmd.z += 0;
        gPTPCmd.r += 0;
        SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
        ProtocolProcess();
      }
      else if (fingerState == 20)
      {
        //MOVE FORWARD
        gPTPCmd.x += 20;
        gPTPCmd.y += 0;
        gPTPCmd.z += 0;
        gPTPCmd.r += 0;
        SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
        ProtocolProcess();
      }
      else if (fingerState == 21)
      {
        //MOVE UP
        gPTPCmd.x += 0;
        gPTPCmd.y -= 0;
        gPTPCmd.z += 20;
        gPTPCmd.r += 0;
        SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
        ProtocolProcess();
      }
      else if(fingerState == 19)
      {
        //MOVE DOWN
        gPTPCmd.x += 0;
        gPTPCmd.y += 0;
        gPTPCmd.z -= 20;
        gPTPCmd.r += 0;
        SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
        ProtocolProcess();
      }
      else if(fingerState == 15)
      {
        //MOVE RIGHT
        gPTPCmd.x += 0;
        gPTPCmd.y += 20;
        gPTPCmd.z += 0;
        gPTPCmd.r += 0;
        SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
        ProtocolProcess();
      }
      else if(fingerState == 18)
      {
        //MOVE LEFT
        gPTPCmd.x += 0;
        gPTPCmd.y -= 20;
        gPTPCmd.z += 0;
        gPTPCmd.r += 0;
        SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
        ProtocolProcess();
      } 
      else if (fingerState == 22)
      {
        //OPEN GRIPPER
        SetEndEffectorGripper(true, true, &gQueuedCmdIndex);
        ProtocolProcess();
      }
      else if(fingerState == 4)
      {
        SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
        ProtocolProcess();
      }
      else if(fingerState == 1)
      {
        SetEndEffectorGripper(false, true, &gQueuedCmdIndex);
        ProtocolProcess();
      }

      else if (fingerState == 100)
      {
        //DEFULT
        gPTPCmd.x += 0;
        gPTPCmd.y += 0;
        gPTPCmd.z += 0;
        gPTPCmd.r += 0;
        SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
        ProtocolProcess();
      }
      else
      {
        //BACKUP DEFULT
        gPTPCmd.x += 0;
        gPTPCmd.y += 0;
        gPTPCmd.z += 0;
        gPTPCmd.r += 0;
        SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
        ProtocolProcess();
      }

}

void loop() 
{

  InitRAM(); //DOBOT CODE

  ProtocolInit();
    
  SetJOGJointParams(&gJOGJointParams, true, &gQueuedCmdIndex);
    
  SetJOGCoordinateParams(&gJOGCoordinateParams, true, &gQueuedCmdIndex);
    
  SetJOGCommonParams(&gJOGCommonParams, true, &gQueuedCmdIndex);
    
  printf("\r\n======Enter demo application======\r\n");
    
  SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);


  for(; ;)
  {
    int on_off = ON_OFF(); //calling piezo function

    if (on_off == true)
    {
      if(!previousState)
      {
        On_sound(); //playing on sound
        previousState = true; //setting state to true so wont be called again(untill turned off)
      }

      lcd.clear(); 
      lcd.setCursor(0, 0);
      lcd.print("Gauntlet Active");

      value_Pinky = analogRead(Pinky); //reading each finger value
      value_Index = analogRead(Index); 
      value_Ring = analogRead(Ring); 
      value_Thumb = analogRead(Thumb); 
      value_Middle = analogRead(Middle);
      
      //value_Pinky = map(value_Pinky, 236, 634, 0, 180); //map function to set values from 236-634 to 0-180
     // value_Index = map(value_Index, 236, 634, 0, 180); 
     // value_Ring = map(value_Ring, 236, 634, 0, 180); 
    // value_Thumb = map(value_Thumb, 236, 634, 0, 180); 
      //value_Middle = map(value_Middle, 236, 634, 0, 180); 

      int fingerState = Bitwise(value_Thumb, value_Index, value_Middle, value_Ring, value_Pinky);
      int state = Finger_read(fingerState); //getting state value
      //Serial.println(state);

      RobotControl(state); //calling robot control
      
      //BTSerial.println(state);
      //BTSerial.println("This is sent via Bluetooth");
      delay(500);
    }

    else
    {
      if(previousState = true) 
      {
        Off_sound(); //calling piezo sound
        previousState = false; //setting state to false so wont be called again(untill turned on)
      }

      //Serial.println("OFF");
      scrollText("Gauntlet Deactivated"); //scrolling text
    }

  }

}


int Bitwise(int thumb, int index, int middle, int ring, int pinky) //Bitwise function (FOUND ONLINE)
{
  int state = 0;
  state = state | (thumb < 100) << 4;
  state = state | (index < 100) << 3;
  state = state | (middle < 100) << 2;
  state = state | (ring < 100) << 1;
  state = state | (pinky < 100);
  return state;
}

int Finger_read(int state) //Return function used to return values
{
  switch (state)
  {
    case 0: 
      //Serial.println ("All fingers up"); 
      state = 1;
      break;
    
    case PINKY_NUM: 
      //Serial.println("Pinky bent");
      state = 2;
      break;
    
    case RING_NUM: 
      //Serial.println("Ring bent");
        state = 3;
      break;
    
    case RING_NUM | PINKY_NUM:
      //Serial.println("Ring and Pinky bent");
      state = 4;
      break;
    
    case MIDDLE_NUM:
      //Serial.println("Middle bent");
      state = 5;
      break;
    
    case MIDDLE_NUM | PINKY_NUM:
      //Serial.println("Middle and Pinky bent");
      state = 6;
      break;
    
    case MIDDLE_NUM | RING_NUM:
      //Serial.println("Middle and Ring bent");
      state = 7;
      break;
    
    case MIDDLE_NUM | THUMB_NUM | PINKY_NUM:
      //Serial.println("Middle, Thumb, and Pinky bent");
      state = 8;
      break;
    
    case INDEX_NUM: 
      //Serial.println("Index bent");
      state = 9;
      break;
    
    case INDEX_NUM | PINKY_NUM:
      //Serial.println("Index and Pinky bent");
      state = 10;
      break;
    
    case INDEX_NUM | RING_NUM:
      //Serial.println("Index and Ring bent");
      state = 11;
      break;
    
    case INDEX_NUM | THUMB_NUM:
      //Serial.println("Index and Thumb bent");
      state = 12;
      break; 
    
    case INDEX_NUM | MIDDLE_NUM:
      //Serial.println("Index and Middle bent");
      state = 13;
      break;
    
    case INDEX_NUM | MIDDLE_NUM | PINKY_NUM:
      //Serial.println("Index, Middle, and Pinky bent");
      state = 14;
      break;
    
    case INDEX_NUM | MIDDLE_NUM | RING_NUM:
      //Serial.println("Index, Middle, and Ring bent");
      state = 15;
      break;
    
    case INDEX_NUM | MIDDLE_NUM | RING_NUM | PINKY_NUM:
      //Serial.println("Index, Middle, Ring, and Pinky bent");
      state = 16;
      break;
    
    case THUMB_NUM:
      //Serial.println("Thumb bent");
      state = 17;
     break;
    
    case THUMB_NUM | PINKY_NUM:
      //Serial.println("Thumb and Pinky bent");
      state = 18;
      break;
    
    case THUMB_NUM | RING_NUM:
      //Serial.println("Thumb and Ring bent");
      state = 19;
      break;
    
    case THUMB_NUM | RING_NUM | INDEX_NUM | MIDDLE_NUM:
      //Serial.println("Thumb, Ring, Index and Middle bent");
      state = 20;
      break;

    case THUMB_NUM | RING_NUM | MIDDLE_NUM | PINKY_NUM:
      //Serial.println("Thumb, Ring, Pinky and Middle bent");
      state = 21;
      break;

    case THUMB_NUM | RING_NUM | INDEX_NUM | MIDDLE_NUM | PINKY_NUM:
      //Serial.println("All Fingers Down");
      state = 22;
      break;
    
    default:
      //Serial.println("UNDIFINED");
        state = 100;
        break;
  }
    
  return state; 
}
  
  void scrollText(String msg) //Function used for scrolling text when gauntlet deactivated
  {
    msg = "   " + msg + "   "; 

    for (int i = 0; i < msg.length() - 16; i++) 
    {
      lcd.clear(); 
      lcd.setCursor(0, 0);
      lcd.print(msg.substring(i, i + 16));
      delay(700); 
    }

  }

 /* Equation for degrees to value
 int y = 0.00193623pow(x,2) + (1.89289x) + (234.579) 
 
 TESTING VALUES:
  Serial.print("Pinky is at: ");
  Serial.println(value_Pinky);
  Serial.print("Index is at: ");
  Serial.println(value_Index);
  Serial.print("Ring is at: ");
  Serial.println(value_Ring);
  Serial.print("Thumb is at: ");
  Serial.println(value_Thumb);
  Serial.print("Middle Finger is at: ");
  Serial.println(value_Middle);
  Serial.println();
  delay(2000); */
