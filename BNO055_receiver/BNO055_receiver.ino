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
Adafruit_BNO055 bno = Adafruit_BNO055(46, 0x28); //Asignar Chip ID de la BNO055 y la dirección I2c

SoftwareSerial HC12(HC12TxdPin, HC12RxdPin);

String final1;
String input;
bool InputComplete = false;


void setup() {
  Serial.begin(9600);
  HC12.begin(9600);
  input.reserve(200); 
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  bno.begin(0x08);
  delay(1000);
  bno.setExtCrystalUse(true);
  int eeAddress = 0;
  long bnoID = 46; //aSIGNAR id bno055 PARA LECTURA DE eeprom
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
  
  final1 = yaw +  "," + pitch +  "," + roll + "," + ACX + "," + ACY + "," + ACZ +',';
  
   while (HC12.available()) {       // If HC-12 has data

      char inChar = (char)HC12.read();

      input += inChar;      
        
    if (inChar == '\n'){

      InputComplete = true;

    }
 

  if(InputComplete) {
   
  Serial.print(final1 + input);


    input = "";

    InputComplete = false;
    
  }
  
  

}
delay(BNO055_SAMPLERATE_DELAY_MS);

}
