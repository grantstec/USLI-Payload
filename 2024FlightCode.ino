#include <A4988.h>
#include <Servo.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SD.h> //Include libraries for SD card usage
#include <SPI.h>
#include <RH_RF95.h>
 
const int chipSelect = BUILTIN_SDCARD; //Tell Teensy 4.1 to use SD card slot
const uint8_t numElements = 8;
const uint8_t runningDataSize = 6; //Index 0 used for landing
File flightData;
float all_data[numElements] = {};
void sd_setup();
void save_data();
void get_data();
char * buffer;
 
//init IMU
Adafruit_ICM20948 icm;
Adafruit_Sensor *icm_temp, *icm_accel, *icm_gyro;

Servo myservo;
//misc
#define limitsw 8 //pin for limit switch at bottom of mount
//recieving
#define MOTOR_STEPS 200 //steps of stepper motor (stay same)
#define DIR 2 //direction pin for stepper driver
#define STEP 1 //step pin for stepper driver
#define enable_stepper 0 //enable pin for stepper driver
//servo
#define servo 9 //servo signal pin
#define servopow 24 //pin of transister to contorl power
//drone power
#define dronepow 25 //pin to control antispark mosfet for drone power
 
#define led 29 //led
 
#define RFM95_CS    10  // Pin definitions
#define RFM95_INT   6
#define RFM95_RST   5
#define RF95_FREQ   908.0 // Default is 434.0 MHz
RH_RF95 rf95(RFM95_CS, RFM95_INT);
 
bool move_down = false;
bool dpow = false;
bool spow = false;
 
A4988 stepper(MOTOR_STEPS, DIR, STEP);
 
//---------------------IMU CODE------------------------
void get_data(){ //Function to read sensors, and update buffer
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  icm_temp->getEvent(&temp);
  icm_accel->getEvent(&accel);
  icm_gyro->getEvent(&gyro);

  // So we dont have to assign values to each element 1 by 1
  float temper[numElements] = {(float)(millis()/1000.0), temp.temperature, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, gyro.gyro.x, gyro.gyro.y, gyro.gyro.z};

  memcpy(all_data, temper, sizeof(temper));
}

// ------------ SD card and Data saving ------------------
void sd_setup(){
  int fileIteration = 0;
  boolean fileCreated = false;
  
  while(!SD.begin(chipSelect)) {
    Serial.println(F("No SD Card detected. Trying again in 2.5 seconds"));
    delay(2500);
  }
  
  while(!fileCreated){
    String fileName = "FLIGHT" + String(fileIteration) + ".csv";

    char str_array[fileName.length() + 1];
    fileName.toCharArray(str_array, fileName.length()+1);
    buffer = strtok(str_array, " ");

    if(!SD.exists(buffer)){
      flightData = SD.open(buffer, FILE_WRITE);
      fileCreated = true;
      delay(500);// idk, might need a delay for it to open, prob not cause the afore SD.open doesnt need one
    }
    fileIteration++;
  }
  flightData.println(F("Time,Temp,acc.x,acc.y,acc.z,gyr.x,gyr.y,gyr.z"));
}
void save_data(){
  String dataEntry = "";
  for(int i=0; i<numElements-1; i++){
    dataEntry += (String(all_data[i]) + ",");
  }
  dataEntry += String(all_data[numElements-1]); // Last data point doesn't need a comma
  flightData.println(dataEntry);
  flightData.flush();
  //Serial.println(dataEntry);
}
 
void setup() {
  // ------------ Serial -----------------
  //Serial.begin(9600);
  //while (!Serial) ; // Wait for serial port to be available, comment out before batt testing
  Serial.println("Serial Done");
  
  // -------------radio init and parameter change-------------
  if (!rf95.init())
    Serial.println("init failed");
 
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  rf95.setSpreadingFactor(9);
  rf95.setSignalBandwidth(62500);
  rf95.setCodingRate4(8);
  rf95.setTxPower(23, false);
 
  //--------- setting pinmodes -----------
  pinMode(limitsw, INPUT_PULLUP);
  pinMode(enable_stepper, OUTPUT);
  pinMode(servopow, OUTPUT);
  pinMode(dronepow, OUTPUT);
  pinMode(led, OUTPUT);
 
  //---------- set output for transistor and mosfets -------------
  digitalWrite(enable_stepper, HIGH);
  digitalWrite(dronepow, HIGH);
  digitalWrite(servopow, HIGH);

  //------------- Stepper Begin -------------
  stepper.begin(200, 1);
 
  //----------------- Setup IMU -----------------
  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
      Serial.println("We are in while");
    }
  }
  icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  icm.setAccelRateDivisor(0);
  icm.setGyroRateDivisor(0);
  icm_temp = icm.getTemperatureSensor();
  icm_accel = icm.getAccelerometerSensor();
  icm_gyro = icm.getGyroSensor();
 
  //---------------- Init SD card-----------------
  sd_setup();
}
 
void loop() {
  //Serial.println(digitalRead(limitsw));
  if (rf95.available()){

    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)){
      Serial.print("got request: ");
      Serial.println((char*)buf);
      //--------- Drone power on (flight controller power and esc power) --------------
      if(strstr((char *)buf, "drone power")){
        digitalWrite(dronepow, LOW);
        dpow = true;
        Serial.println("dpow");
      }
      //}
      // while(dpow == true){
      //   if(strstr((char *)buf, "shutoff")){
      //     digitalWrite(dronepow, HIGH);
      //     dpow = false;
      //   }
      // }

      //--------- leg deployment down ----------
      else if(strstr((char *)buf, "lower legs")){//when the switch is in the middle posistion on the transmitter the arms lower
        Serial.println("lowlegs");
        digitalWrite(enable_stepper, LOW);// enable stepper driver to recive signals and hold position
        delay(200);
        while(digitalRead(limitsw) == LOW){
          stepper.move(-100);
          Serial.println("moving down");
        }
        Serial.println("All the way down");
        digitalWrite(led, HIGH);
        move_down = true;
        digitalWrite(enable_stepper, LOW); //keep stepper motor locked
      }
       
      //-------- servo power and release ---------------
      else if(strstr((char *)buf, "release")){
        digitalWrite(servopow, LOW);
        delay(500);
        myservo.write(42);
        myservo.attach(servo);
        Serial.println("release");
        spow = true;
        myservo.write(82);
        delay(250);
        //digitalWrite(servopow,HIGH);
      }
      else{
        Serial.println("Unknown MSG");
      }
      Serial.println("---------------- Looping");   
    }
  }
  if(move_down){
    get_data();
    save_data();
  }
}
 
