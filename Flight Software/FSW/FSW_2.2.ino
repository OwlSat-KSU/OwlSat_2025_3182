// libraries 
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <QMC5883LCompass.h>
#include <MPU6050.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

int16_t ax, ay, az, gx, gy, gz;

// cool purple stuff
#define NUM_SAMPLES 15
#define MotorPin PA15
#define Servo 
#define SD_CS PA4
#define BMP280_ADDRESS_ALT 0x76

// objects
Adafruit_BMP280 bmp;
QMC5883LCompass compass;
MPU6050 mpu;
File myFile;
SoftwareSerial Xbee(PA2,PA3);
Adafruit_GPS GPS(&Wire);





// defining all the variables
const int loadPin = PA0;
const int Camera = PA1;
bool state = false;
bool startup = false;
int steps = 0;
int count = 0;
unsigned long duration = 500000;// 0.5 seconds in microseconds
bool prev_state = false;
unsigned status;
float Samples[NUM_SAMPLES] = {0};
int avg_rpm = 0;
bool SIM_mode = false;
int packt_sent = 0;
String mode[2] = {"F" , "S"};
String phases[5] = {"IDLE","READY","ASCENDING","DESCENDING","RETREIVAL"};
String stage;
String command[7] = {"CX","ST","SIM","SIMP","CAL","MECH"};
String packet = "";
int m = 0;
int s = 0;
uint8_t p = 1;
uint8_t i = 1;
uint8_t d = 1;
const int analogPin = PB0;
const float V_REF = 7.8;
const float max_pressure = 1000.0;




int GetRPS(){
// code to get the rpm of the Auto-Gyro
steps = 0;
unsigned long start = micros();
while (micros() - start < duration){
  bool current_state = digitalRead(PB4);
  if(current_state && !prev_state){
    steps++;
    prev_state = current_state;
  }
  delayMicroseconds(10); // debounce?

}
steps = (steps/4); // *2 to turn 0.5s to 1s, *60 to get RPM , diving by 3 to account for 3 holes.


if(avg_rpm > 0){
  if(abs((steps-avg_rpm)/avg_rpm) < 0.15 ){   // find the error percentage and verifies if the calc value is unreliable data.
    Samples[count] = steps;
    count++;
  }
}else{
  Samples[count] = steps;
  count++;
}



if(count >= NUM_SAMPLES){
  count = 0;
  avg_rpm = 0;
  for(int j = 0; j < NUM_SAMPLES; j++){
    avg_rpm = avg_rpm + Samples[j];
    }
  avg_rpm = avg_rpm/NUM_SAMPLES;
  return avg_rpm;
}

}

String BMP280(){
float temp = bmp.readTemperature();
float pressure = bmp.readPressure() / 100.0F;
float altitude = bmp.readAltitude();
return String(altitude) + "," + String(temp) + "," + String(pressure);
}

int boomStick(){


}

int Orientation(){
    compass.read();
    int x = compass.getX();
    int y = compass.getY();
    int z = compass.getZ();
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

}

void LowPowerMode() {
// turn off mosfet and sensor connection to not waste power
bmp.setSampling(Adafruit_BMP280::MODE_SLEEP);
mpu.setSleepEnabled(true);

}


String GPS_data() {
  static String nmea = "";
    char c = GPS.read();
    if (GPS.newNMEAreceived()) {
      nmea = GPS.lastNMEA();
      if (nmea.startsWith("$GPRMC")){
        return parseRMC(nmea);
      }
    }else{
      nmea = GPS.lastNMEA();
      return parseRMC(nmea);
    }
}

String parseRMC(String sentence) {
  // Split by comma
  int commaIndex = 0;
  int field = 0;
  String tokens[12];

  while (field < 12 && commaIndex != -1) {
    int nextComma = sentence.indexOf(',', commaIndex);
    if (nextComma == -1) break;
    tokens[field++] = sentence.substring(commaIndex, nextComma);
    commaIndex = nextComma + 1;
  }

  // tokens[1] = time (hhmmss.sss)
  // tokens[3] = latitude
  // tokens[5] = longitude
  // tokens[7] = speed
  // tokens[8] = course
  // tokens[9] = date

  String timeStr = tokens[1];
  String hour = timeStr.substring(0, 2);
  String minute = timeStr.substring(2, 4);
  String second = timeStr.substring(4, 6);
  if( tokens[2] == 'A' || tokens[4] == 'N' ){break;}else{String lat = tokens[2];
  String lon = tokens[4];}
  

  // Dummy satellites count = 0 because RMC doesn't include it
  // You could re-enable GGA if you want true satellites in view

  return hour + "," + minute + "," + second + "," + lat + "," + lon + ",0";
}





String getTime(){


}

void Handle_Commands(String command){



}


void Xbee_send(String packet){
packet = "3182," +  String(packt_sent) +","+ mode[m] +","+ phases[s] +  "," + packet;
Xbee.print(packet);
Xbee.flush();
Serial.println(packet);
packt_sent = packt_sent + 1;
}

void Mosfet(bool logic){
digitalWrite(PA0,logic);
}

String getVoltage(){

int rawValue = analogRead(analogPin);
float voltage = rawValue * (V_REF / 4095.0);
return String(voltage);

}


void setup() {
Mosfet(HIGH);
delay(1000);
//Wire1.begin();
Wire.setSCL(PB6);
Wire.setSDA(PB7);
Wire.begin();
GPS.begin(0x10);
delay(1500);
GPS.sendCommand("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"); // RMC + GGA

GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

Serial.begin(115200);
Xbee.begin(9600);     // UART2 (PA2/PA3) for XBee
compass.init();
bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
pinMode(MotorPin,OUTPUT);
pinMode(PA0,OUTPUT);
analogReadResolution(12);
}


void loop(){

// put your main code here, to run repeatedly:
packet = "";
packet =  BMP280() + "," + getVoltage() + "," + String(GetRPS()) + "," + GPS_data();
Xbee_send(packet);


}
