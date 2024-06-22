#include "WiFi.h"
#include "StreamIO.h"
#include "VideoStream.h"
#include "RTSP.h"
#include "NNObjectDetection.h"
#include "ObjectClassList.h"
#include "NNAudioClassification.h"
#include "AudioClassList.h"
#include "VideoStreamOverlay.h"
#include "BLEDevice.h"

#define UART_SERVICE_UUID      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define STRING_BUF_SIZE 100
#define MaxNumValue     2

typedef struct {
    bool reciveCMDFlag;
    int ReciveValue;
} _rCMD;

BLEService UartService(UART_SERVICE_UUID);
BLECharacteristic Rx(CHARACTERISTIC_UUID_RX);
BLECharacteristic Tx(CHARACTERISTIC_UUID_TX);
BLEAdvertData advdata;
BLEAdvertData scndata;
bool notify = false;
uint8_t Count;

String CMDRefer[5] = {"SS2", "SS4", "SRT", "SR2", "SRV"};
_rCMD bleReciveData[MaxNumValue];

#define value1 0
#define value2 1

#define CHANNEL   0     // Channel 0 : 1920 x 1080 30FPS H264
#define CHANNELNN 3

// Lower resolution for NN processing
#define NNWIDTH  576
#define NNHEIGHT 320

VideoSetting config(VIDEO_FHD, 30, VIDEO_H264, 0);
VideoSetting configNN(NNWIDTH, NNHEIGHT, 10, VIDEO_RGB, 0);
AudioSetting configA(16000, 1, USE_AUDIO_AMIC); // Sample rate, Channel count, Mic type
Audio audio;
NNAudioClassification audioNN;
NNObjectDetection ObjDet;
RTSP rtsp;
StreamIO videoStreamer(1, 1);
StreamIO audioStreamerNN(1, 1);                 // 1 Input Audio -> 1 Output Audio Classification
StreamIO videoStreamerNN(1, 1);

char ssid[] = "abc";    // your network SSID (name)
char pass[] = "0936386705";        // your network password
int status = WL_IDLE_STATUS;

IPAddress ip;
int rtsp_portnum;

#include "I2Cdev.h"

#include <MPU6050_IMU_libraries/MPU6050_6Axis_MotionApps612.h>
// #include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include <VL53L0X_IR_libraries/VL53L0X.h>
#include <Adafruit_OLED_libraries/Adafruit_GFX.h>
#include <Adafruit_OLED_libraries/Adafruit_SSD1306.h>

#define SCREEN_WIDTH  128    // OLED display width, in pixels
#define SCREEN_HEIGHT 32     // OLED display height, in pixels

#define OLED_RESET     -1      // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C    ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

VL53L0X sensor;
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
// MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */


#define OUTPUT_READABLE_YAWPITCHROLL


// #define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
// #define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;      // set true if DMP init was successful
uint8_t mpuIntStatus;       // holds actual interrupt status byte from MPU
uint8_t devStatus;          // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;        // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;         // count of all bytes currently in FIFO
uint8_t fifoBuffer[128];    // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


#define power_pin 1
#define MotoR_A 3
#define MotoR_B 4
#define MotoL_A 5
#define MotoL_B 6
#define MotoClean 7
#define detect_pin 8

int STATE = 0; //STOP=0;forward=1;back=2;left=3;right=4;

bool power_On = false;
bool det_Clapping = false;
bool getObj = false;
unsigned short NoObj_cnt = 0;
float direction = 0;
unsigned int distance = 0;
float previous_angle = 0;
float obj_angle = 0;
char target_name[15];
bool sweepstate = false;
bool interrupt = false;
bool interred = false;
int i = 0;

void forward(int T)
{
  if(!interrupt){
    analogWrite(MotoL_A, T);
    digitalWrite(MotoL_B, 0);

    analogWrite(MotoR_A, T);
    digitalWrite(MotoR_B, 0);

    STATE = 1;
    delay(50);
  }
}

void backward(int T)
{
    digitalWrite(MotoL_A, 0);
    analogWrite(MotoL_B, T);

    digitalWrite(MotoR_A, 0);
    analogWrite(MotoR_B, T);

    STATE = 2;
    delay(50);
}

void turnLeft(int T)
{
    analogWrite(MotoL_A, T);
    digitalWrite(MotoL_B, 0);

    digitalWrite(MotoR_A, 0);
    analogWrite(MotoR_B, T);

    STATE = 3;
    delay(50);
}

void turnRight(int T)
{
    digitalWrite(MotoL_A, 0);
    analogWrite(MotoL_B, T);

    analogWrite(MotoR_A, T);
    digitalWrite(MotoR_B, 0);

    STATE = 4;
    delay(50);
}

void BrakeAll()
{
    digitalWrite(MotoL_A, 0);
    digitalWrite(MotoL_B, 0);

    digitalWrite(MotoR_A, 0);
    digitalWrite(MotoR_B, 0);

    STATE = 0;
    delay(50);
}

void StopALL(){
    digitalWrite(MotoL_A, 0);
    digitalWrite(MotoL_B, 0);
    digitalWrite(MotoR_A, 0);
    digitalWrite(MotoR_B, 0);
    digitalWrite(MotoClean, 0);

    char text_str[25];
    snprintf(text_str, sizeof(text_str), "What's wrong, master?");
    OSD.drawText(CHANNEL, 900, 680, text_str, OSD_COLOR_RED);

    display.clearDisplay();
    display.setTextSize(1);   
    display.setTextColor(SSD1306_WHITE); 
    display.setCursor(0, 10);
    display.print("What's wrong, master?");
    display.display();

    for(int i=0;i<10;i++){
      blinkState = !blinkState;
      digitalWrite(LED_BUILTIN, blinkState);
      delay(1000);
    }
}

void ISR_back_right(uint32_t id, uint32_t event)
{ 
  interrupt = true;
}

void readCB(BLECharacteristic* chr, uint8_t connID)
{
    printf("Characteristic %s read by connection %d \n", chr->getUUID().str(), connID);
}

void writeCB(BLECharacteristic* chr, uint8_t connID)
{
    // printf("Characteristic %s write by connection %d :\n", chr->getUUID().str(), connID);
    if (chr->getDataLen() > 0) {
        ParseCMDString(chr->readString());
        // Serial.print("Received string: ");
        // Serial.print(chr->readString());
        // Serial.println();
    }
}

void notifCB(BLECharacteristic* chr, uint8_t connID, uint16_t cccd)
{
    if (cccd & GATT_CLIENT_CHAR_CONFIG_NOTIFY) {
        printf("Notifications enabled on Characteristic %s for connection %d \n", chr->getUUID().str(), connID);
        notify = true;
    } else {
        printf("Notifications disabled on Characteristic %s for connection %d \n", chr->getUUID().str(), connID);
        notify = false;
    }
}

void ParseCMDString(String cmd)
{
    int comdLength = cmd.length();
    int chkx;
    int CMDMaxNUM = sizeof(CMDRefer) / sizeof(String);

    for (chkx = 0; chkx < CMDMaxNUM; chkx++) {
        if (cmd.indexOf(CMDRefer[chkx].c_str()) > -1) {
            break;
        }
    }

    if (chkx >= CMDMaxNUM && cmd.charAt(comdLength - 1) != '#') {
        return;
    }

    if (cmd.indexOf("SRT") > -1) {
        int x = 3;
        int ValueIndex = 0;

        while (x < (comdLength - 1)) {
            if ((x + 3) < comdLength) {
                String _NumString = cmd.substring(x, (x + 4));
                // Serial.println(_NumString);
                if (ValueIndex < MaxNumValue) {
                    if (bleReciveData[ValueIndex].ReciveValue != _NumString.toInt()) {
                        bleReciveData[ValueIndex].ReciveValue = _NumString.toInt();
                        bleReciveData[ValueIndex].reciveCMDFlag = true;
                    }
                }
            }
            ValueIndex++;
            x += 4;
        }
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(MotoL_A, OUTPUT);
    pinMode(MotoL_B, OUTPUT);
    pinMode(MotoR_A, OUTPUT);
    pinMode(MotoR_B, OUTPUT);
    pinMode(MotoClean, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(detect_pin, INPUT_IRQ_FALL);
    pinMode(power_pin, INPUT);

    digitalSetIrqHandler(detect_pin, ISR_back_right);

    digitalWrite(MotoL_A, 0);
    digitalWrite(MotoL_B, 0);
    digitalWrite(MotoR_A, 0);
    digitalWrite(MotoR_B, 0);
    digitalWrite(MotoClean, 0);

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;    // Don't proceed, loop forever
    }

    if (!sensor.init()) {
        Serial.println("Failed to detect and initialize sensor!");
        while (1) {
        }
    }
    sensor.startContinuous();
    Init_display();

    //V7RC
    advdata.addFlags(GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED);
    advdata.addCompleteName("AMB82-OCTO");
    scndata.addCompleteServices(BLEUUID(UART_SERVICE_UUID));

    Rx.setWriteNRProperty(true);
    Rx.setWritePermissions(GATT_PERM_WRITE);
    Rx.setWriteCallback(writeCB);
    Rx.setBufferLen(STRING_BUF_SIZE);

    Tx.setReadProperty(true);
    Tx.setReadPermissions(GATT_PERM_READ);
    Tx.setReadCallback(readCB);
    Tx.setNotifyProperty(true);
    Tx.setCCCDCallback(notifCB);
    Tx.setBufferLen(STRING_BUF_SIZE);

    UartService.addCharacteristic(Rx);
    UartService.addCharacteristic(Tx);

    BLE.init();
    BLE.configAdvert()->setAdvData(advdata);
    BLE.configAdvert()->setScanRspData(scndata);
    BLE.configServer(1);
    BLE.addService(UartService);

    BLE.beginPeripheral();

    // attempt to connect to Wifi network:
    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to WPA SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);

        // wait 2 seconds for connection:
        delay(2000);
    }
    ip = WiFi.localIP();

    display.clearDisplay();
    display.setTextSize(1);   
    display.setCursor(0, 0);
    display.print("IP: ");
    display.println(ip);
    display.display();

    // Configure camera video channels with video format information
    // Adjust the bitrate based on your WiFi network quality
    config.setBitrate(2 * 1024 * 1024);    // Recommend to use 2Mbps for RTSP streaming to prevent network congestion
    Camera.configVideoChannel(CHANNEL, config);
    Camera.configVideoChannel(CHANNELNN, configNN);
    Camera.videoInit();

    // Configure RTSP with corresponding video format information
    rtsp.configVideo(config);
    rtsp.begin();
    rtsp_portnum = rtsp.getPort();

    // Configure audio peripheral for audio data format
    audio.configAudio(configA);
    audio.begin();

    audioNN.configAudio(configA);
    audioNN.setResultCallback(ACPostProcess);
    audioNN.modelSelect(AUDIO_CLASSIFICATION, NA_MODEL, NA_MODEL, NA_MODEL, CUSTOMIZED_YAMNET);
    audioNN.begin();

    // Configure object detection with corresponding video format information
    // Select Neural Network(NN) task and models
    ObjDet.configVideo(configNN);
    ObjDet.modelSelect(OBJECT_DETECTION, CUSTOMIZED_YOLOV7TINY, NA_MODEL, NA_MODEL);
    ObjDet.begin();

    // Configure StreamIO object to stream data from video channel to RTSP
    videoStreamer.registerInput(Camera.getStream(CHANNEL));
    videoStreamer.registerOutput(rtsp);
    if (videoStreamer.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    // Start data stream from video channel
    Camera.channelBegin(CHANNEL);
    delay(1000);

    // Configure StreamIO object to stream data from RGB video channel to object detection
    videoStreamerNN.registerInput(Camera.getStream(CHANNELNN));
    videoStreamerNN.setStackSize();
    videoStreamerNN.setTaskPriority();
    videoStreamerNN.registerOutput(ObjDet);
    if (videoStreamerNN.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    //Wire.begin();
    Wire.setClock(400000);    // 400kHz I2C clock. Comment this line if having compilation difficulties
#endif

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    //   pinMode(INTERRUPT_PIN, INPUT);

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(51);
    mpu.setYGyroOffset(8);
    mpu.setZGyroOffset(21);
    mpu.setXAccelOffset(1150);
    mpu.setYAccelOffset(-50);
    mpu.setZAccelOffset(1060);
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println();
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        // Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        // Serial.println(F(")..."));
        // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

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
    }
    
    // Configure StreamIO object to stream data from audio to audio classification
    audioStreamerNN.registerInput(audio);
    audioStreamerNN.registerOutput(audioNN);
    if (audioStreamerNN.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    // configure LED for output
    pinMode(LED_BUILTIN, OUTPUT);

    // Start video channel for NN
    Camera.channelBegin(CHANNELNN);

    // Start OSD drawing on RTSP video channel
    OSD.configVideo(CHANNEL, config);
    OSD.begin();
}

void ACPostProcess(std::vector<AudioClassificationResult> results)
{
    printf("No of Audio Detected = %d\r\n", audioNN.getResultCount());

    if (audioNN.getResultCount() > 0) {
        for (int i = 0; i < audioNN.getResultCount(); i++) {
            AudioClassificationResult audio_item = results[i];
            int class_id = (int)audio_item.classID();
            if (audioNames[class_id].filter) {
                int prob = audio_item.score();
                printf("%d class %d, score: %d, audio name: %s\r\n", i, class_id, prob, audioNames[class_id].audioName);
                det_Clapping = true;
            }
        }
    }
}

void BLE_V7RC()
{
    while (Count < MaxNumValue) {
      if(!interrupt){
        if (bleReciveData[Count].reciveCMDFlag) {
            bleReciveData[Count].reciveCMDFlag = false;
            //print("%d\t%d\n",bleReciveData[value1].ReciveValue,bleReciveData[value2].ReciveValue);

            if (abs(bleReciveData[value1].ReciveValue - 1500) < 100 && abs(bleReciveData[value2].ReciveValue - 1500) < 100) {
                BrakeAll();
            } else if (abs(bleReciveData[value1].ReciveValue - 1500) > abs(bleReciveData[value2].ReciveValue - 1500)) {
                if (bleReciveData[value1].ReciveValue > 1500) {
                    turnRight(255);
                } else {
                    turnLeft(255);
                }
            } else {
                if (bleReciveData[value2].ReciveValue > 1500) {
                    forward(255);
                } else {
                    backward(255);
                }
            }
        }
        Count++;
      }else{
        char text_str[28];
        snprintf(text_str, sizeof(text_str), "Interrupt,please backward!");
        OSD.drawText(CHANNEL, 900, 680, text_str, OSD_COLOR_RED);
      }
    }
    Count = 0;
    delay(1);
}

void MPU_getYPR()
{
  #ifdef OUTPUT_READABLE_YAWPITCHROLL
          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          direction = ypr[0] * 180 / M_PI;
          delay(100);
  #endif
}

void ODPostProcess()
{
    std::vector<ObjectDetectionResult> results = ObjDet.getResult();

    uint16_t im_h = config.height();
    uint16_t im_w = config.width();

    Serial.print("Network URL for RTSP Streaming: ");
    Serial.print("rtsp://");
    Serial.print(ip);
    Serial.print(":");
    Serial.println(rtsp_portnum);
    Serial.println(" ");

    printf("Total number of objects detected = %d\r\n", ObjDet.getResultCount());
    OSD.createBitmap(CHANNEL);

    getObj = false;
    obj_angle = 380;
    strcpy(target_name, "Nothing");

    if (ObjDet.getResultCount() > 0) {
        int closest_obj = 0;
        int target_pos = 0;
        for (int i = 0; i < ObjDet.getResultCount(); i++) {
            int obj_type = results[i].type();
            if (itemList[obj_type].filter) {    // check if item should be ignored
                ObjectDetectionResult item = results[i];
                // Result coordinates are floats ranging from 0.00 to 1.00
                // Multiply with RTSP resolution to get coordinates in pixels
                int xmin = (int)(item.xMin() * im_w);
                int xmax = (int)(item.xMax() * im_w);
                int ymin = (int)(item.yMin() * im_h);
                int ymax = (int)(item.yMax() * im_h);

                // Draw boundary box
                printf("Item %d %s:\t%d %d %d %d\n\r", i, itemList[obj_type].objectName, xmin, xmax, ymin, ymax);
                OSD.drawRect(CHANNEL, xmin, ymin, xmax, ymax, 3, OSD_COLOR_WHITE);

                // Print identification text
                char text_str[25];
                snprintf(text_str, sizeof(text_str), "%s %d", itemList[obj_type].objectName, item.score());
                OSD.drawText(CHANNEL, xmin, ymin - OSD.getTextHeight(CHANNEL), text_str, OSD_COLOR_CYAN);

                if(closest_obj < xmax-xmin) {
                    closest_obj = xmax-xmin;
                    target_pos = round(((xmax+xmin)-im_w)/2);
                    obj_angle = atan2(target_pos,780)* 180 / M_PI;
                    strcpy(target_name, itemList[obj_type].objectName);
                }
                getObj = true;
            }
        }
    }
    OSD.update(CHANNEL);
}

void loop()
{   
    // if programming failed, don't try to do anything
    if (!dmpReady) {
        return;
    }
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {    // Get the Latest packet
        MPU_getYPR();
    }
    distance = sensor.readRangeContinuousMillimeters();

    ODPostProcess();

    power_On = digitalRead(power_pin);
    if (power_On) {
      if (!interrupt) {
        handleNormalOperation();
      }else{
        handleInterrupt();
      }
    }
    DisplayPrint();
    Waittime();
}

void handleNormalOperation() {
  if (BLE.connected(0)) {
    BLE_V7RC();
    i = 0;
    return;
  }

  if (det_Clapping) {
    handleClapping();
    return;
  }

  if (getObj) {
    handleObjectDetection();
    return;
  }

  handleNoObject();
}

void handleClapping() {
  StopALL();
  det_Clapping = false;
}

void handleObjectDetection() {
  if (abs(obj_angle) < 1 && distance < 100) {
    BrakeAll();
    i = 20;
    sweepstate = true;
    digitalWrite(MotoClean, 1);
  } else {
    BrakeAll();
    i = 5;
    Waittime();
    if (obj_angle == 380) {
      i = 0;
    } else if (abs(obj_angle) < 3) {
      Serial.println("forward");
      forward(128);
      i = 3;
    } else if (obj_angle > 0) {
      previous_angle = direction;
      turnRight(128);
      //i = 1;
    } else {
      previous_angle = direction;
      turnLeft(128);
      //i = 1;
    }
  }
}

void handleNoObject() {
  if (NoObj_cnt == 300) {
    NoObj_cnt = 0;
    turnRight(250);
    delay(200);
    BrakeAll();
    delay(100);
  } else {
    NoObj_cnt++;
  }
}

void handleInterrupt() {
  if (interred) {
    i = 1;
    interred = false;
    turnRight(160);
  } else {
    backward(250);
    i = 4;
    interred = true;
  }
}


void Waittime(){
  for (int cnt = 0;cnt<i;cnt++){
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {    // Get the Latest packet
        MPU_getYPR();
    }
    distance = sensor.readRangeContinuousMillimeters();
    ODPostProcess();
    DisplayPrint();
  }
  if(i==4 && interrupt){
    i = 0;
  }else if(i==1 && interrupt){
    interrupt = false;
  }
  if(i==20 && sweepstate){
    digitalWrite(MotoClean, 0);
    sweepstate = false;
  }
  BrakeAll();
  i = 0;
}

void DisplayPrint(){
    display.clearDisplay();
    display.setTextSize(1);   
    display.setCursor(0, 0);
    display.print("yaw   ");
    display.println(direction);
    display.print("distance : ");
    display.println(distance);
    display.print(target_name);
    display.print(" ");
    display.println(obj_angle);
    if(power_On){
      if(BLE.connected(0)){
        display.println("V7RC-control");
      }else if(interrupt){
        display.println("Interrupt!");
      }else{
        switch(STATE){
          case 0:
              display.println("stop");
              break;
          case 1:
              display.println("forward");
              break;
          case 2:
              display.println("backword");
              break;
          case 3:
              display.println("left");
              break;
          case 4:
              display.println("right");
              break;
        }
      }
    }else{
      display.println("POWER OFF");
    }
    display.display();
}

#define LOGO_HEIGHT 32
#define LOGO_WIDTH  56
static const unsigned char PROGMEM logo_bmp[] =
{
B00000000,	B00000000,	B00000000,	B00000000,	B00000000,  B00000000,	B00000000,
B00000000,	B00000001,	B11100000,	B00000000,	B00000000,	B00000000,  B00000000,
B00000000,	B00000001,	B11110000,	B00000000,	B00000000,	B00000000,  B00000000,
B00000000,	B00000011,	B11111000,	B00000000,	B00000000,  B00000000,	B00000000,
B00000000,	B00000001,	B11111100,	B00000000,	B00000000,	B00000000,  B00000000,
B00000000,	B00000000,	B11111100,	B00000000,	B00000000,	B00000000,  B00000000,
B00000000,	B00000000,	B11111110,	B00000000,	B00000000,	B00000000,  B10000000,
B00000000,	B00000000,	B11111110,	B00000000,	B00000000,	B00000111,  B10100000,
B00000000,	B00000000,	B11111111,	B00000000,	B00000000,	B00001111,  B11000000,
B00000000,	B00000000,	B11111111,	B00000000,	B00000000,	B00111111,  B11000000,
B00000000,	B00000000,	B11111111,	B00000000,	B00000000,	B11111111,  B11000000,
B00000000,	B00000000,	B11111111,	B00000000,	B00001111,	B11111111,  B10000000,
B00000000,	B00000000,	B11111111,	B11111111,	B11111111,	B11111111,  B10000000,
B00000000,	B00000000,	B11111111,	B11111111,	B11111111,	B11111111,  B00000000,
B00000000,	B00000000,	B11111111,	B11111111,	B11111111,	B11111110,  B00000000,
B00000000,	B00000000,	B11111111,	B11111111,	B11111111,	B11111110,  B00000000,
B00000000,	B00000000,	B11111111,	B11111111,	B11111111,	B11111110,  B00000000,
B00000000,	B00000000,	B11111111,	B11111111,	B11111111,	B11111100,  B00000000,
B00000000,	B00000000,	B11111111,	B11111111,	B11111111,	B11111100,  B00000000,
B00000000,	B00000000,	B01111111,	B11111111,	B11111111,	B11111000,  B00000000,
B00000000,	B00000000,	B01111111,	B11111111,	B11111111,	B11111000,  B00000000,
B00000000,	B00000000,	B00111111,	B11111111,	B11111111,	B11110000,  B00000000,
B00000000,	B00000000,	B00011111,	B11111111,  B11111111,	B11100000,  B00000000,
B00000000,	B00000000,	B00001111,	B11111111,  B11111111,	B11000000,  B00000000,
B00000000,	B00000000,	B00000111,	B11111111,	B11111111,	B10000000,  B00000000,
B00000000,	B00000000,	B00000011,	B11111111,	B11111111,	B00000000,  B00000000,
B00000000,	B00000000,	B00000000,	B00111100,	B11111110,	B00000000,  B00000000,
B00000000,	B00000000,	B00000000,	B00010000,	B00001000,	B00000000,  B00000000,
B00000000,	B00000000,	B00000000,	B00010000,	B00001000,	B00000000,  B00000000,
B00000000,	B00000000,	B00000000,	B11100000,	B00111000,	B00000000,  B00000000,
B00000000,	B00000000,	B00000000,	B00000000,	B00000000,	B00000000,  B00000000,
B00000000,	B00000000,	B00000000,	B00000000,	B00000000,	B00000000,  B00000000,
};

void Init_display()
{
    display.clearDisplay();
    display.drawBitmap(
        (display.width() - LOGO_WIDTH),
        (display.height() - LOGO_HEIGHT +1),
        logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
    display.setTextSize(2);                 // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);    // Draw white text
    display.setCursor(0, 10);                // Start at top-left corner
    display.println(F("Big_GG"));

    display.display();
}
