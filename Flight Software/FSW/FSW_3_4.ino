// libraries 
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <QMC5883LCompass.h>
#include <MPU6050.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <Servo.h>

int16_t ax, ay, az, gx, gy, gz;

// cool purple stuff
#define NUM_SAMPLES 15
// #define MotorPin PA15
// #define Servo_pin PB1
#define SD_CS PA4
#define BMP280_ADDRESS_ALT 0x76

// objects
Adafruit_BMP280 bmp;
QMC5883LCompass compass;
MPU6050 mpu;
File myFile;
SoftwareSerial Xbee(PA2,PA3);
Adafruit_GPS GPS(&Wire);
Servo myservo;
Servo motor;
Servo beep;





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
String command[7] = {"CX_on","CX_OFF","ST","SIM","SIMP","CAL","MECH"};
String packet = "";
String lat = "";
String lon = "";
String SIMP = "";
String satCount;
int m = 0;
int s = 0;
int pos = 48;
uint8_t p = 1;
uint8_t i = 1;
uint8_t d = 1;
const int analogPin = PB0;
const float V_REF = 3.6;
const float max_pressure = 1000.0;
const int command_window = 1500;
String hour;
String minute;
String second;

void BEEP(){
  beep.write(pos);
}


void Pin_release(bool logic){

  myservo.write(pos);
}
  




String GetRPS(){
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
  return String(avg_rpm);
}

}

String BMP280(int m, String SIMP){
if(m){
  return SIMP;
}
float temp = bmp.readTemperature();
float pressure = bmp.readPressure() / 100.0F;
float altitude = bmp.readAltitude();
return (String(altitude) + "," + String(temp) + "," + String(pressure));
}

void boomStick(){
motor.attach(pos);

}

String Mag(){
    compass.read();
    int x = compass.getX();
    int y = compass.getY();
    int z = compass.getZ();
    return String(x)+"," + String(y)+"," + String(z);

}


String Gyro(){
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  return String(gx)+"," + String(gy)+"," + String(gz)+"," + String(ax)+"," + String(ay)+"," + String(az);

}




String GPS_data() {
  static String nmea = "";
  int timee = millis();
  int dur = 500;
  
  while (millis() - timee < dur) {
    char c = GPS.read();
  }

    nmea = GPS.lastNMEA();
    if(nmea.indexOf("$GNRMC") == 0){
      nmea = nmea.substring(2);
      return parseRMC(nmea);
    }else if(nmea.indexOf("$GNGGA") == 0){
      nmea = nmea.substring(2);
      return parseGGA(nmea);
    }else{return nmea;}
}



String parseRMC(String sentence) {

  String tokens[16];
  int commaIndex = 0, field = 0;
  while (field < 16 && commaIndex != -1) {
    int nextComma = sentence.indexOf(',', commaIndex);
    if (nextComma == -1) break;
    tokens[field++] = sentence.substring(commaIndex, nextComma);
    commaIndex = nextComma + 1;
  }

  String timeStr = tokens[1]; // hhmmss.sss
  hour = timeStr.substring(0, 2);
  minute = timeStr.substring(2, 4);
  second = timeStr.substring(4, 6);
  String lat = tokens[3];
  String lon = tokens[5];

  return hour + "," + minute + "," + second + "," + lat + "," + lon + "," + satCount;
}



String parseGGA(String sentence) {
  String tokens[16];
  int commaIndex = 0, field = 0;
  while (field < 16 && commaIndex != -1) {
    int nextComma = sentence.indexOf(',', commaIndex);
    if (nextComma == -1) break;
    tokens[field++] = sentence.substring(commaIndex, nextComma);
    commaIndex = nextComma + 1;
  }

  String timeStr = tokens[1]; // hhmmss.sss
  hour = timeStr.substring(0, 2);
  minute = timeStr.substring(2, 4);
  second = timeStr.substring(4, 6);
  String lat = tokens[2];
  String lon = tokens[4];
  satCount = tokens[7];

  return hour + "," + minute + "," + second + "," + lat + "," + lon + "," + satCount;
}

String getTime(){
  

  return hour + minute + second;
}

void Handle_Commands(String command){
if(command.indexOf("3182") == 0){
  String c = command.substring(5);
  if(c.indexOf("CX") == 0){}
  else if(c.indexOf("ST") == 0){}
  else if(c.indexOf("SIM") == 0){}
  else if(c.indexOf("SIMP") == 0){}
  else if(c.indexOf("CAL") == 0){}
  else if(c.indexOf("MEC") == 0){}


   
  }}







void SD_CARD(String data){
myFile = SD.open("data.txt", FILE_WRITE);
myFile.println(data);
myFile.close();

data = String(s);
myFile = SD.open("state.txt", FILE_WRITE);
myFile.println(data);
myFile.close();

data = String(m);
myFile = SD.open("mode.txt", FILE_WRITE);
myFile.println(data);
myFile.close();


}

int SD_STATE_CHECK(){
myFile = SD.open("state.txt", FILE_READ);
String lastLine = "";
  while (myFile.available()) {
    String line = myFile.readStringUntil('\n');
    if (line.length() > 0) {
      lastLine = line;
    }else{return 0;}
  }

  myFile.close();
  return lastLine.toInt();
}


int SD_MODE_CHECK(){
myFile = SD.open("mode.txt", FILE_READ);
String lastLine = "";
  while (myFile.available()) {
    String line = myFile.readStringUntil('\n');
    if (line.length() > 0) {
      lastLine = line;
    }else{return 0;}
  }

  myFile.close();
  return lastLine.toInt();
}


void Xbee_send(String packet){
packet = "3182," + getTime() +","+ String(packt_sent) +","+ mode[m] +","+ phases[s] +  "," + packet;
Xbee.print(packet);
Xbee.flush();
SD_CARD(packet);
Serial.println(packet);
packt_sent = packt_sent + 1;
}

void Mosfet(bool logic){
digitalWrite(PA0,logic);
}

void Mosfet2(bool logic){
digitalWrite(PA8,logic);
}

String getVoltage(){

int rawValue = analogRead(analogPin);
float voltage = rawValue * (V_REF / 4095.0);
return String(voltage);

}

void IDLE(){
if(Xbee.available()){
  String c = Xbee.readStringUntil('\n');
  Handle_Commands(c);
  }else{Xbee_send("Invalid command");}

}



void READY(){}

void ASCENDING(){}

void DESCENDING(){}

void RETREIVAL(){}


void setup() {
pinMode(PA0,OUTPUT);
Mosfet(HIGH);
Serial.begin(115200);
Xbee.begin(9600);    // UART2 (PA2/PA3) for XBee
delay(1000);
Xbee_send("|Mosfet started|");
delay(500);
Wire.setSCL(PB6);
Wire.setSDA(PB7);
Wire.begin();
Xbee_send("|I2C started|");
delay(1500);
while(!GPS.begin(0x10)){}
Xbee_send("|GPS started|");
GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
delay(500);
GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
delay(500);
myservo.attach(PB9);
motor.attach(PB8);  // attaches the servo on pin 9 to the servo object
beep.attach(PB1);
BEEP();
while(!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)){}
Xbee_send("|BMP280 started|");
mpu.initialize();
while(!mpu.testConnection()){}
compass.init();
Xbee_send("|MPU started|");
analogReadResolution(12);
SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
    SPI.setMOSI(PA7);
    SPI.setMISO(PA6);
    SPI.setSCLK(PA5);
while(!SD.begin(SD_CS)){}
delay(5000);
Xbee_send("|SD card started|");
}


void loop(){

// put your main code here, to run repeatedly:
packet = "";
s = SD_STATE_CHECK();
m = SD_MODE_CHECK();


packet = BMP280(m,SIMP)+"," + getVoltage()+"," + Gyro() +"," + Mag()+"," + GetRPS()+"," + GPS_data() +"," + "Waiting for command:";
Xbee_send(packet);
delay(500);
unsigned long time = millis();
while(millis() - time < command_window){
if(Xbee.available()){
  String c = Xbee.readString();
  if(c.indexOf("CMD") == 0){
  c = c.substring(3);
  Handle_Commands(c);

  }
  }
  }

// switch(s){
//   case 0: IDLE();
//   break;
//   case 1: READY();
//   break;
//   case 2: ASCENDING() ;
//   break;
//   case 3: DESCENDING();
//   break;
//   case 4: RETREIVAL();
//   break;
//   }



}
