//original Manhattan MAC Address 7C:9E:BD:45:77:A4
// updated Manhattan MAC Address 94:B9:7E:E9:0F:94
// cooler MCU MAC Address 94:B9:7E:DA:B8:C4

#include <Arduino.h>

/*
   Video: https://www.youtube.com/watch?v=oCMOYS71NIU
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
   Ported to Arduino ESP32 by Evandro Copercini, with some additional code by pcbreflux

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   Other Libreries:
   Included in the GitHub repository https://github.com/diybar/firmware/Libraries
*/

const bool debug = true;
const bool debugLoop = false;
const int numMotors = 9;
const int maxMotorsRunning = 4;
const int minGlassDistance = 780;
const int blinkTimes = 10; //Number if blinks if there is no glass
int fadeAmount = 10;    // how many points to fade the LED by

// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0

// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT  13

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     5000

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN            12

// Distance Sensor PIN
//#define PWM_PIN            22

bool inProgress = false;
bool backwardsOn = false;
int blinkedTimes = 0;
int brightness = 0; 
bool ledOn = false;
bool turnLedOn = false;
bool turnLedOff = false;
bool ledBlink = false;

// Arduino like analogWrite
// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}

//Multiple cores
#include <Streaming.h>      // Ref: http://arduiniana.org/libraries/streaming/
#include "Workload.h"
#include "Task1.h"
TaskHandle_t TaskA;
#include <QueueList.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "L9110Driver.h"
#include <vector>


#include <SPI.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

QueueList <String> motorsQueue;

std::vector<L9110_Motor> motor(numMotors);
int timeToCompletion[numMotors]={0,0,0,0,0,0,0,0,0};
int motorsRunning = 0;

BLEServer* pServer = NULL;
BLECharacteristic *pCharacteristic;
BLEDescriptor *pDescriptor;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool deviceNotifying = false;
String notification;
uint8_t value = 0;

void sendBTNotification(String message) {
    if (deviceConnected && deviceNotifying) {
        char charBuf[10];
        String(message).toCharArray(charBuf, 10);
        pCharacteristic->setValue(charBuf);
        pCharacteristic->notify();
    }
}

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID               "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX     "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX     "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

//Receiver MAC
//uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0xDA, 0xB8, 0xC4};


uint8_t broadcastAddress[] = { 0x30 , 0xC6 , 0xF7 , 0x23 , 0x86 , 0x68};

// Variable to store if sending data was successful
String success;

//Message structure
typedef bool bottle_status[8]; 

typedef struct struct_message {
  char version;
  float light;
  int glass_distance;
  int motors_running;
  bool BT_connected;
  char bottle;
} struct_message;

struct_message MCU_OUT_message;
struct_message MCU_IN_message;

esp_now_peer_info_t peerInfo;

unsigned long TimeToSend = 0;
static const int  rate_data_sending = 500;


// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (debug) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  }
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&MCU_IN_message, incomingData, sizeof(MCU_IN_message));
  if (debug) {
    Serial.print("Bytes received: ");
    Serial.println(len);
  }

}




void startLedBlink() {
    brightness = 0;
    blinkedTimes = 0;
    ledBlink = true;
}

// setMotors() Command String = motor_number-dirrection-duration 
// motor 0 - 8 for pumps.
// direction f (forward) or b (backward).
// duration in miliseconds, if duration = 0 it will run until a stop command is send.  

void setMotors(String command) {
    bool motorStarted = false;
    int index = command.indexOf('-');
    int secondIndex = command.indexOf('-', index + 1);
  
    int motorNumber = atoi(command.substring(0, index).c_str());
    String motorDirection = command.substring(index + 1, secondIndex);
    long duration = atoi(command.substring(secondIndex + 1).c_str()); 
  
    if (motorNumber < numMotors) {  
      if (motorDirection == "s"){   
        motor[motorNumber].run (BRAKE);
        if (motorsRunning > 0) {
          motorsRunning = motorsRunning - 1;
        }
        sendBTNotification("stop:" + String(motorNumber));
        if (debug) {
            Serial.printf("Motor %d stopped\n", motorNumber);
        }
      } else if (motorsRunning < maxMotorsRunning) {
          if (motorDirection == "b") {
              motor[motorNumber].run (BACKWARD | RELEASE);
              motorStarted = true;
          } else {   
              if (glassDistance < minGlassDistance && glassDistance > 0) {
                  motor[motorNumber].run (FORWARD | RELEASE);
                  motorStarted = true;
              } else {
                  sendBTNotification("noGlass");
                  if (debug) {
                      Serial.println("Glass not ready");
                  }
                  if (!ledBlink) {
                      startLedBlink();
                  }
              } 
          }
          if (motorStarted) {
              if (motorsRunning == 0) {
                  sendBTNotification("start");
              }
              motorsRunning += 1; 
              if (duration > 0) {
                unsigned long currentMillis = millis();
                timeToCompletion[motorNumber] = duration + currentMillis; 
              }
              sendBTNotification("start:" + String(motorNumber)); 
              if (debug) {
                  Serial.printf("Motor %d started\n", motorNumber);
              }
           }  
        } else {
            //Add pending command to a queue
            motorsQueue.push(command);
        }
    } else {
       sendBTNotification("noMotorNum");
       if (debug) {
          Serial.println("The motor number doesn't exist");
       }
    }     
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        notification = "";
        for (int i = 0; i < rxValue.length(); i++) {
          notification = notification + rxValue[i];
        }
        if (debug) {
          Serial.print("Received Value: ");
          Serial.print(notification);
          Serial.println();
        }
        if (notification == "lockOn") {
          inProgress = true;
        } else if (notification == "backwardsOn"){
          backwardsOn = true;
        } else {
          if (inProgress == false) {
            setMotors(notification);
          } else {
            sendBTNotification("lockOn");
            if (debug) {
              Serial.println("Lock is on!");
            } 
          }
        }
      }
    }
};

class MyDisCallbacks: public BLEDescriptorCallbacks {
    void onWrite(BLEDescriptor *pDescriptor) {
      uint8_t* rxValue = pDescriptor->getValue();

      if (pDescriptor->getLength() > 0) {
        if (rxValue[0]==1) {
          deviceNotifying=true;
          if (debug) {
            Serial.println("Notifications enabled");
          }
        } else {
          deviceNotifying=false;
          if (debug) {
            Serial.println("Notifications disabled");
          }
        }
      }
    }
};



void send_data(struct_message Message) {
        // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Message, sizeof(Message));
    
    if (debug) { 
      if (result == ESP_OK) {
      
        Serial.print("Sent with success ");Serial.print(Message.glass_distance); Serial.print(" BT ");Serial.println(Message.BT_connected);
      }
      else {
        Serial.println("Error sending the data");
      }
    }
    //vTaskDelay(rate_data_sending/portTICK_PERIOD_MS);
}




void setup() {
  // Ref: http://esp32.info/docs/esp_idf/html/db/da4/task_8h.html#a25b035ac6b7809ff16c828be270e1431
  xTaskCreatePinnedToCore(
     Task1,                  /* pvTaskCode */
     "Workload1",            /* pcName */
     1000,                   /* usStackDepth */
     NULL,                   /* pvParameters */
     1,                      /* uxPriority */
     &TaskA,                 /* pxCreatedTask */
     0);                     /* xCoreID */


  if (debug) {
    Serial.begin(115200);
    // wait until serial port opens for native USB devices
    while (! Serial) {
      delay(1);
    }
  
  }

  Serial.println("MyBar v1.91 - Clockwise wires");
  
  motor[0].initialize(32, 33);
  motor[1].initialize(25, 26);
  motor[2].initialize(27, 14);
  motor[3].initialize(13, 23);
  motor[4].initialize(21, 3);
  motor[5].initialize(19, 18);
  motor[6].initialize( 5, 17);
  motor[7].initialize(16, 4);
  motor[8].initialize(15, 2);

  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);

  for (int bright = 255; bright >= 0; bright--) {
      ledcAnalogWrite(LEDC_CHANNEL_0, bright);
      delay(10);
  }
  
  // engage the motor's brake
  for (int i = 0; i <= numMotors; ++i) {
      motor[i].run (BRAKE);
  }
  
  // Create the BLE Device
  BLEDevice::init("DIYBar");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY                
                    );

  pDescriptor = new BLE2902();
  pCharacteristic->addDescriptor(pDescriptor);

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pDescriptor->setCallbacks(new MyDisCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  if (debug) {
    Serial.println("Waiting a client connection to notify...");
  }  

    // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    if (debug) {
      Serial.println("Error initializing ESP-NOW");
    }
    return;
  } else {
    Serial.println("ESP-NOW Initialized");
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    if (debug) {
      Serial.println("Failed to add peer");
    }
    return;
  } else {
    Serial.println("ESP-NOW Peer added");
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  //initialize ESP Now data sending timer
  TimeToSend = millis() + rate_data_sending;

  Serial.println(WiFi.macAddress());
}

void loop() { 
  //glassDistance = pulseIn(PWM_PIN, HIGH);
     
  if (debug && debugLoop) {
     Serial.printf("Motors running: %d\n", motorsRunning);
     Serial.printf("Glass Distance: %d\n", glassDistance);
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }

  unsigned long currentMillis = millis();

  if ( currentMillis >= TimeToSend ) {
    MCU_OUT_message.glass_distance = glassDistance;
    MCU_OUT_message.motors_running = motorsRunning;
    MCU_OUT_message.BT_connected = deviceConnected;
    MCU_OUT_message.version = 33;
    if ( debug) {
      //Serial.println(MCU_OUT_message.glass_distance);
      //Serial.println(MCU_OUT_message.motors_running);
      //Serial.println(MCU_OUT_message.BT_connected);
    }
    send_data(MCU_OUT_message);
    TimeToSend = currentMillis + rate_data_sending;
  }


  // If motors are in queue, start one at a time
  if ((motorsRunning < maxMotorsRunning) && (!motorsQueue.isEmpty())) {
    setMotors(motorsQueue.pop());
  }
  
  if (turnLedOn) {
    ledcAnalogWrite(LEDC_CHANNEL_0, 255);
    ledOn = true;
    turnLedOn = false;
  }
  
  if (turnLedOff) {
    ledcAnalogWrite(LEDC_CHANNEL_0, 0);
    ledOn = false;
    turnLedOff = false;
    if (!(backwardsOn)) {
      if (motorsRunning > 0) {
        // engage the motor's brake
        for (int i = 0; i < numMotors; ++i) {
          motor[i].run (BRAKE);
          timeToCompletion[i] = 0;
        }
        motorsRunning = 0;
        sendBTNotification("motorsStp");
      }
      while (!motorsQueue.isEmpty())
        motorsQueue.pop();
    }
  }    
  
  if (ledBlink) {
    // change the brightness for next time through the loop:
    brightness = brightness + fadeAmount;
    ledcAnalogWrite(LEDC_CHANNEL_0, brightness);
    // reverse the direction of the fading at the ends of the fade:
    if (brightness <= 0 || brightness >= 255) {
      fadeAmount = -fadeAmount;
      blinkedTimes += 1;
    }
    if (blinkedTimes >= blinkTimes) {
      ledBlink = false;
      ledOn = false;
      ledcAnalogWrite(LEDC_CHANNEL_0, 0);
      brightness = 0;
    }
    delay(10);
  } 

  if (glassDistance < minGlassDistance && glassDistance > 0) {
    if (!(ledOn)) turnLedOn = true;
  } else {
    if (ledOn or (motorsRunning > 0)) turnLedOff = true;    
  }

  if (inProgress && motorsRunning <= 0) {
    inProgress = false;
    backwardsOn = false;
  }

  if (inProgress && motorsRunning != 0) { 
    // Stop motors as time is up
    for (int thisMotor = 0; thisMotor < numMotors; thisMotor++) {
      bool lastMotor = false;
      if (timeToCompletion[thisMotor] > 0 && timeToCompletion[thisMotor] <= currentMillis) {
        motor[thisMotor].run (BRAKE);
        if (motorsRunning > 0) {
          motorsRunning = motorsRunning - 1;
          if (motorsRunning == 0) {
            lastMotor = true;
          }
        }  
        timeToCompletion[thisMotor] = 0; 
        sendBTNotification("stop:" + String(thisMotor));
        if (debug) {
          Serial.printf("Motor %d stopped\n", thisMotor);
        }
        if (lastMotor) {
          startLedBlink();
          sendBTNotification("finish");
          if (debug) {
            Serial.printf("All Motors stopped");
          }
        }      
      }
    }
  }
}
