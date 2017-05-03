#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <FS.h>
#include <WebSockets.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <I2Cdev.h>
#define DEBUG
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>

//#include <ArduinoOTA.h>

#include "Motors.h"
#include "Controller.h"
#include "WifiCredentials.h"
#include "TickerScheduler.h"

#undef WIFI_MODE_AP
//#define WIFI_MODE_AP

ESP8266WebServer srv(80);
WebSocketsServer webSocket(81);
Motors motors(D4, D3, D2, D1);
Controller ctl;
TickerScheduler sched(1);

boolean powerOn = false;
double pitchZero = -1.4, rollZero = -4.0;

double pitchP = 1.0, pitchI = -0.1, pitchD = -0.2;
double yawP = 0.5, yawI = 0.0, yawD = -0.1;

double throttleSet = 80;
double pidRange = 60, throttleRange = 120;

double pitchSet = 0, pitchIn = 0, pitchOut = 0;
double rollSet = 0, rollIn = 0, rollOut = 0;
double yawSet = 0, yawIn = 0, yawOut = 0;
PID pitchPid(&pitchIn, &pitchOut, &pitchSet, pitchP, pitchI, pitchD, DIRECT);
PID rollPid(&rollIn, &rollOut, &rollSet, pitchP, pitchI, pitchD, DIRECT);
PID yawPid(&yawIn, &yawOut, &yawSet, yawP, yawI, yawD, DIRECT);

double mot1, mot2, mot3, mot4;

MPU6050 mpu;

// MPU control/status vars
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion variables
Quaternion q;
VectorFloat gravity;
float euler[3];
float ypr[3];

const float RADIANS_TO_DEGREES = 57.2958; //180/3.14159

// This global variable tells us how to scale gyroscope data
float    GYRO_FACTOR;

// This global varible tells how to scale acclerometer data
float    ACCEL_FACTOR;

void dmpDataReady() {
    mpuInterrupt = true;
}

String getContentType(String filename){
  if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

bool handleFileRead(String path){
  String contentType = getContentType(path);
  if(SPIFFS.exists(path)){
    File file = SPIFFS.open(path, "r");
    size_t sent = srv.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

static uint8_t connectedClient = 0xff;

// handle a websocket traffic. Note, that only one client connection is allowed at any time.
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {

  switch(type) {
    case WStype_DISCONNECTED: //if a connected client disconnects, free the slot
      Serial.printf("[%u] Disconnected!\n", num);
      if (num == connectedClient) {
        connectedClient = 0xff; //'magic' number, don't want to keep another flag
      }
      break;
    case WStype_CONNECTED: { //a new client has connected
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        if (connectedClient != 0xff || num == 0xff) {
          //we must drop a connection with number 0xff, otherwise there would be a glitch
          //client should retry connecting
          webSocket.sendTXT(num, "NO WAY!");
          webSocket.disconnect(num);
        } else {
          connectedClient = num;
//          webSocket.sendTXT(num, "GO ON");
          webSocket.sendTXT(num, String("P") + pitchP + "," + pitchI + "," + pitchD);
        }
      }
      break;
    case WStype_TEXT: {
        if (num != connectedClient) {
          webSocket.sendTXT(num, "NO WAY!");
          webSocket.disconnect(num);
          break;
        }
//        Serial.printf("[%u] got Text: %s\n", num, payload);
        String cmd((char *)payload);
        ctl.parseMessage(cmd);
      }
      break;
  }
}

void indicateReadiness() {
  motors.setSpeeds(20, 0, 0, 0);
  delay(300);
  motors.setSpeeds(0, 20, 0, 0);
  delay(300);
  motors.setSpeeds(0, 0, 20, 0);
  delay(300);
  motors.setSpeeds(0, 0, 0, 20);
  delay(300);
  motors.setSpeed(0);
}

void setUpWifi() {
#ifdef WIFI_MODE_AP
  //set up an access point
  Serial.println("Setting soft-AP ... ");
  boolean result = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  if (result == true) {
    Serial.println(F("Ready"));
    IPAddress myIP = WiFi.softAPIP();
    Serial.print(F("AP IP address: "));
    Serial.println(myIP);
  } else {
    Serial.println(F("Failed!"));
    delay(10000);
    //don't know if it's a right thing to do...
    ESP.reset();
  }
#else
  //conect to an existing WiFi network
  Serial.print(F("Connecting to the WiFi."));
  WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.print(F("\nConnected, IP address: "));
  Serial.println(WiFi.localIP());
  //TODO: need to figure out a way to show an IP address without a serial connection
#endif

}

void setUpHttp() {
  //HTTP redirect on / location -> index.html
  srv.on("/", [](){
    srv.sendHeader("Location", String("/index.html"), true);
    srv.send(302, "text/plain", "");
  });
  //default routing -> look for a file on the SPIFFS and serve
  srv.onNotFound([](){
    if(!handleFileRead(srv.uri()))
      srv.send(404, "text/plain", "FileNotFound");
  });
  //start a HTTP server on port 80
  srv.begin();
  Serial.println("HTTP server started");

  //set up a file subsystem
  if (!(SPIFFS.begin())) {
    Serial.println(F("SPIFFS.begin failed"));
  }
}

void setUpMpu() {
  Wire.begin(D6, D7);
  pinMode(D5, INPUT);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection..."));
    attachInterrupt(digitalPinToInterrupt(D5), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // Set the full scale range of the gyro
    uint8_t FS_SEL = 0;
    //mpu.setFullScaleGyroRange(FS_SEL);

    // get default full scale value of gyro - may have changed from default
    // function call returns values between 0 and 3
    uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();
    Serial.print("FS_SEL = ");
    Serial.println(READ_FS_SEL);
    GYRO_FACTOR = 131.0/(FS_SEL + 1);

    // get default full scale value of accelerometer - may not be default value.  
    // Accelerometer scale factor doesn't reall matter as it divides out
    uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange();
    Serial.print("AFS_SEL = ");
    Serial.println(READ_AFS_SEL);
    //ACCEL_FACTOR = 16384.0/(AFS_SEL + 1);
    
    // Set the full scale range of the accelerometer
    //uint8_t AFS_SEL = 0;
    //mpu.setFullScaleAccelRange(AFS_SEL);

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    return;
  }
  
}

//void setUpOTA() {
//  ArduinoOTA.onStart([]() {
//    String type;
//    if (ArduinoOTA.getCommand() == U_FLASH)
//      type = "sketch";
//    else // U_SPIFFS
//      type = "filesystem";
//
//    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
//    Serial.println("Start updating " + type);
//  });
//  ArduinoOTA.onEnd([]() {
//    Serial.println("\nEnd");
//  });
//  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
//    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
//  });
//  ArduinoOTA.onError([](ota_error_t error) {
//    Serial.printf("Error[%u]: ", error);
//    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
//    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
//    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
//    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
//    else if (error == OTA_END_ERROR) Serial.println("End Failed");
//  });
//  ArduinoOTA.begin();  
//}

void setup() {
  Serial.begin(115200);
  Serial.println();

// set up Over The Air updates!

//  setUpOTA();
  
//==============================
// set up the WiFi connection
//==============================
  setUpWifi();

//==============================
// set up a web server on port 80
//==============================

  setUpHttp();
  
//==============================
// start a websocket server on port 81
//==============================

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

//==============================
// set up MPU connection
//==============================

  setUpMpu();
  
//==============================
// set up PID loops
//==============================

  pitchPid.SetSampleTime(50);
  pitchPid.SetMode(MANUAL);
  pitchPid.SetOutputLimits(-pidRange, pidRange);
  rollPid.SetSampleTime(50);
  rollPid.SetMode(MANUAL);
  rollPid.SetOutputLimits(-pidRange, pidRange);
  yawPid.SetSampleTime(50);
  yawPid.SetMode(MANUAL);
  yawPid.SetOutputLimits(-pidRange, pidRange);

// set up scheduled tasks
// -- websocket client update
  sched.add(0, 1000, sendWsData, (void *)0, false); //call every 200 ms
// now we're ready.
  indicateReadiness();
}

void readMpuData() {
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
      
      // Obtain Euler angles from buffer
      //mpu.dmpGetQuaternion(&q, fifoBuffer);
      //mpu.dmpGetEuler(euler, &q);
      
      // Obtain YPR angles from buffer
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

     // Output complementary data and DMP data to the serial port.  The signs on the data needed to be
     // fudged to get the angle direction correct.
//       Serial.print("DMP:");
//       Serial.print(ypr[2]*RADIANS_TO_DEGREES, 2);
//       Serial.print(":");
//       Serial.print(-ypr[1]*RADIANS_TO_DEGREES, 2);
//       Serial.print(":");
//       Serial.println(ypr[0]*RADIANS_TO_DEGREES, 2);
  }    

}

void sendWsData(void *par) {
  if (connectedClient != 0xff) {
    webSocket.sendTXT(connectedClient, String("G1") + map(pitchIn, -30, 30, 0, 100));
    webSocket.sendTXT(connectedClient, String("G2") + map(rollIn, -30, 30, 0, 100));
    webSocket.sendTXT(connectedClient, String("G3") + map(mot1, 0, 180, 0, 100));
    webSocket.sendTXT(connectedClient, String("G4") + map(mot2, 0, 180, 0, 100));
    webSocket.sendTXT(connectedClient, String("G5") + map(mot3, 0, 180, 0, 100));
    webSocket.sendTXT(connectedClient, String("G6") + map(mot4, 0, 180, 0, 100));
  }
}


void loop() {
  srv.handleClient();
  webSocket.loop();
//  sched.update();
  if (mpuInterrupt) {
    mpuInterrupt = false;
    readMpuData();
    yawIn = ypr[0] * RADIANS_TO_DEGREES;
    pitchIn = -ypr[1] * RADIANS_TO_DEGREES - pitchZero;
    rollIn = ypr[2] * RADIANS_TO_DEGREES - rollZero;
  }
  
  if (ctl.changed()) { //automatically clears the flag
    
//    pitchSet = ctl.pitch;
//    rollSet = ctl.roll;
//    yawSet = ctl.yaw;
    throttleSet = (double) (180 - ctl.throttle) / 2 * (180 / throttleRange);
  }
  if (ctl.pressed()) {
    switch (ctl.button) {
      case 'A': //turn on the powah!
        pitchPid.SetMode(AUTOMATIC);
        rollPid.SetMode(AUTOMATIC);
        yawPid.SetMode(AUTOMATIC);
        powerOn = true;
        break;
      case 'B': //be quiet (and still)!
        pitchPid.SetMode(MANUAL);
        rollPid.SetMode(MANUAL);
        yawPid.SetMode(MANUAL);
        powerOn = false;
        break;
      case 'C':
        break;
      case 'D': //zero IMU readings
        Serial.println("zeroing!");
        pitchZero = -ypr[1] * RADIANS_TO_DEGREES;
        rollZero = ypr[2] * RADIANS_TO_DEGREES;
        break;
    }
  }
  if (ctl.submitted()) {
    pitchP = ctl.newP;
    pitchI = ctl.newI;
    pitchD = ctl.newD;
    pitchPid.SetTunings(pitchP, pitchI, pitchD);
    rollPid.SetTunings(pitchP, pitchI, pitchD);
  }
  
  pitchPid.Compute();
  rollPid.Compute();
  yawPid.Compute();

  mot1 = throttleSet + pitchOut; // +yawOut
  mot3 = throttleSet - pitchOut; // +yawOut

  mot2 = throttleSet + rollOut; // +yawOut
  mot4 = throttleSet - rollOut; // +yawOut

  if (powerOn) {
    motors.setSpeeds((uint8_t) mot1, (uint8_t) mot2, (uint8_t) mot3, (uint8_t) mot4);
  } else {
    motors.setSpeed(0);
  }

//  ArduinoOTA.handle();
}

