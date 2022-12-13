#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)

const byte HC12RxdPin = 11;                 
const byte HC12TxdPin = 10; 
int led = 2;
Adafruit_BNO055 bno = Adafruit_BNO055(41, 0x28);

SoftwareSerial HC12(HC12TxdPin, HC12RxdPin);

String final1;

void setup() {
  
  Serial.begin(9600);
  HC12.begin(9600); 
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  bno.begin(0x08);
  delay(1000);
  bno.setExtCrystalUse(true);
  int eeAddress = 0;
  long bnoID = 41;
  bool foundCalib = false;
  EEPROM.get(eeAddress, bnoID);
  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;
   bno.getSensor(&sensor);
   
    if (bnoID != sensor.sensor_id)
    {
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    }
    else
    {
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        

        Serial.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;

        digitalWrite(led, HIGH);
    }

  
  
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  float YAW = event.orientation.x;
  float PITCH = event.orientation.y;
  float ROLL = event.orientation.z;
  float AX = acc.x();
  float AY = acc.y();
  float AZ = acc.z();
  
  String yaw = String(YAW);
  String pitch = String(PITCH);
  String roll = String(ROLL);
  String ACX = String(AX);
  String ACY = String(AY);
  String ACZ = String(AZ);
  final1 = yaw +  "," + pitch + "," + roll + "," + ACX + "," + ACY + "," + ACZ + '\n';
  HC12.print(final1);
  Serial.println(final1);
  delay(BNO055_SAMPLERATE_DELAY_MS);

}
