/*
  This project contains two parts of functions:
  1.using the Hardware I2C to get the IMU data from the Slaves.
  2.using BLE to send the IMU data to PC.
*/
#include <ArduinoBLE.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"


// IIC initial setup
#define maxlength 128
#define buffer_size 26
#define slave_num 5
char realNodeNumber[maxlength];  // the maximum quantity of IIC slave node is 128
char buff[maxlength][buffer_size] = {""};

String str = "";
int count = 0;

int startup_id = 0;

String addr = "";

//BLE initial setup
BLEService IMUsMonitorService("00001101-0000-1000-8000-00805f9b34fb");  // setup the uuid
BLEStringCharacteristic txString("00001143-0000-1000-8000-00805f9b34fb", BLERead | BLENotify, 512);
BLEBooleanCharacteristic uidCharacteristic("00001142-0000-1000-8000-00805f9b34fb", BLERead | BLEWrite);


void do_blink(PinName led_id, unsigned long interval = 1000);
void clear_blink(PinName led_id, bool low_on = true);

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

  startup_id = analogRead(A0) * 8192 + analogRead(A1);

  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(2000000);
  delay(3000);
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }

/*******BLE Initialization*********/

  Serial.println("Initializing");

  Serial.print("Startup ID = ");
  Serial.println(startup_id);
  
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);

  if (!BLE.begin()) 
  {
    Serial.println("Starting BLE failed!");
    while (1) {
      digitalWrite(LED_RED, LOW);
      delay(500);
      digitalWrite(LED_RED, HIGH);
      delay(500);
    }
  }
  
// BLE initialization
  BLE.setLocalName("IMUsMonitor");
  BLE.setAdvertisedService(IMUsMonitorService); // add the service UUID
  IMUsMonitorService.addCharacteristic(txString); // add the characteristic
  IMUsMonitorService.addCharacteristic(uidCharacteristic); // add the characteristic
  BLE.addService(IMUsMonitorService); // Add the service

  // initial value
  uidCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");
  Serial.print("value size = ");
  Serial.println(txString.valueSize());

  Serial.print("Bluetooth address: ");
  Serial.println(BLE.address());

  addr = BLE.address();

  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, HIGH);

/*******Slaves Initialization*********/
// detect slave nodes


  char data[buffer_size];
  String strdata = "";
  Serial.println("Current slave devices includes: ");
////
  for(int j=0; j<10; j++) // repeat to detect IIC signals
  {
    for(int i=0; i<maxlength; i++)
    {
      Wire.requestFrom(i, buffer_size);    // request buffer_size bytes from slave device #xxx
      count = 0;
      
      while(Wire.available())    // slave may send less than requested
      {
        do_blink(LED_RED, 500);
        data[count++] = Wire.read();    // receive a byte as character
      }
      
      strdata += data;
      if(strdata.substring(0, strdata.indexOf(' ')).toInt() == i)
      {
        realNodeNumber[i] = '1';
//        Serial.println((int)data[0]);
      }
      strdata = "";
//      delayMicroseconds(1); 
      delay(1);
     }
     delay(100);

    clear_blink(LED_RED);
  }

  for(int i=0; i<maxlength; i++)
  {
    if(realNodeNumber[i] == '1')
    {
      Serial.print(i);
      Serial.print(' ');
    }    
  }
  Serial.println(' ');
  
  pinMode(LED_BLUE, OUTPUT);
  for(int i = 0; i < 5; i++) {
    digitalWrite(LED_BLUE, LOW);
    delay(200);
    digitalWrite(LED_BLUE, HIGH);
    delay(200);
  }

  clear_blink(LED_GREEN);
}


bool noserial = false;
bool connected = false;

bool uid = false;


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop()
{
  updateIMUsMonitor();
 
  // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) 
  {
    // while(Serial.available()) {
    //   int ctrl = Serial.read();
    //   if(ctrl == '1') {
    //     uid = true;
    //   } else if(ctrl == '0') {
    //     uid = false;
    //   } else if(ctrl == '3') {
    //     uid = !uid;
    //   } else {
    //     continue;
    //   }
    //   digitalWrite(LED_RED, uid ? LOW : HIGH);
    // }
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);

    noserial = true;
    connected = true;
    clear_blink(LED_GREEN);

    // while the central is connected:
    while (central.connected()) 
    {
      updateIMUsMonitor();

      if (uidCharacteristic.written()) {
        uid = uidCharacteristic.value();
        digitalWrite(LED_RED, uid ? LOW : HIGH);
      }
    }

    noserial = false;
    connected = false;
    clear_blink(LED_BLUE);
    
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }

}


// ================================================================
// ===                    Custom Functions                      ===
// ================================================================

unsigned long blink_next = 0;
bool blink_status = false;

void clear_blink(PinName led_id, bool low_on) {
  digitalWrite(led_id, low_on ? HIGH : LOW);
  blink_status = false;
  blink_next = 0;
}

void do_blink(PinName led_id, unsigned long interval) {
  unsigned long cur_time = millis();
  if(cur_time > blink_next) {
    blink_next = cur_time + 1000;
    blink_status = !blink_status;
    if(blink_status) {
      digitalWrite(led_id, LOW);
    } else {
      digitalWrite(led_id, HIGH);
    }
  }
}

void updateIMUsMonitor() 
{
  uint8_t node_count = 0;

  if(connected) {
    do_blink(LED_BLUE);
  } else {
    do_blink(LED_GREEN);
  }

  str += addr;
  str += "%";
  str += startup_id;
  str += "+";
  str += millis();
  str += ";";

  
  for(uint8_t i=0; i<maxlength; i++)
  {    
    if(realNodeNumber[i] == '1')
    {
      node_count ++;
      Wire.requestFrom(i, buffer_size);    // request buffer_size bytes from slave device #xxx
      count = 0;

      while(Wire.available())    // slave may send less than requested
      {
        buff[i][count++] = Wire.read();    // receive a byte as character
        if(count > buffer_size)
          break;
      }

      str += buff[i];
      str += ";";
    
      delayMicroseconds(1); 

//      if(node_count % 9 == 0)
//      {
//         txString.writeValue(str);  // 蓝牙发送数据，由于蓝牙一次性发生数据长度有限，所以设置每9个数据打包发送一次
//         Serial.println(str);
//         str = "";
//      }
      
    }
  }
   
  txString.writeValue(str); // 蓝牙发送数据
  if(!noserial) {
    Serial.println(str);
  }
  str = "";
}
