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
#define RED PB12
#define YELLOW PB13
#define GREEN PB14
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
int reference_altitude = 0;
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
bool SIM_ACTIVE = false;
int packt_sent = 0;
String mode[2] = {"F" , "S"};
String phases[5] = {"IDLE","READY","ASCENDING","DESCENDING","RETREIVAL"};
String stage;
String packet = "";
String lat = "";
String lon = "";
String SIMP = "";
String satCount;
int hh_c = 0, mm_c = 0, ss_c = 0;
int m = 0;
int s = 0;
int pos = 48;
uint8_t p = 1;
uint8_t i = 1;
uint8_t d = 1;
const int analogPin = PB0;
const float V_REF = 3.3;
float max_alt = 400.0;
bool deployed = false;
const float max_pressure = 0;
float pressure = 0;
float altitude = 0;
float i_pressure = 0;
String hour;
String minute;
String second;
int C_tt = 0;
void BEEP(int frequency){
  // 0 == flat beep
  // 48 == no beep
  // 120 fast beep
  beep.write(frequency);
}


void Pin_release(int degrees){
  myservo.write(degrees);
}

void Deploy(){
if(!deployed && (altitude < 0.77*max_alt)){
  Pin_release(103);
  delay(1500);
  boomStick(140);
  digitalWrite(Camera,LOW);
  deployed = true;
}
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

String BMP280(){
if(m && SIM_ACTIVE){
  return SIMP;
}
float temp = bmp.readTemperature();
pressure = bmp.readPressure() / 100.0F;
altitude = bmp.readAltitude() - reference_altitude;
if(max_alt < altitude){
  max_alt = altitude;
  s = 2;
  }

if(!i_pressure){
  i_pressure = pressure;
}

return (String(altitude) + "," + String(temp) + "," + String(pressure));
}

void boomStick(int Lakshay){
// 96 - 86 == no rotation
// 86 - 35 == reverse rotation
// 96 - 145 == forward rotation
motor.write(Lakshay);

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
  GPS_data();
  int D_tt = abs(C_tt - (hour.toInt()*3600 + minute.toInt()*60 + second.toInt()));
  int tot_ss = D_tt%60 + ss_c;
  int tot_mm = int(D_tt/60) + mm_c;
  int tot_hh = int(int(D_tt/60)/60) + hh_c;

  if(tot_ss > 60){
    tot_mm +=1;
    tot_ss -= 60;
  }

  if(tot_mm > 60){
  tot_hh +=1;
  tot_mm -= 60;
  }

  if(tot_hh > 24){
  tot_hh -= 24;
  }

  
  
  
  return String(tot_hh)+":" + String(tot_mm)+":" + String(tot_ss);
}

void Handle_Commands(String sentence){
  String token[6];
  int commaIndex = 0, field = 0;
  while (field < 6 && commaIndex != -1) {
    int nextComma = sentence.indexOf(',', commaIndex);
    if (nextComma == -1) break;
    token[field++] = sentence.substring(commaIndex, nextComma);
    commaIndex = nextComma + 1;
  }

  Serial.println(token[0]);
  Serial.println(token[1]);
  Serial.println(token[2]);
  Serial.println(token[3]);

  // token[0] = Team ID
  // token[1] = Command type, ex. CX, ST
  // token[2] = Value/Device
  // token[3] = ON/OFF -- only for MEC command

if(token[0] == "3182"){
  
  
  if(token[1] == "CX"){
    if(token[2] == "ON"){
      Mosfet(HIGH);
      initializeSensors();
    }
    else if(token[2] == "OFF"){
      Wire.flush();
      delay(1000);
      Mosfet(LOW);
      delay(500);

      

    }
  }


  else if(token[1] == "MEC"){

    if(token[2] == "BoomStick"){
    if(token[3] == "ON"){  boomStick(120);  }
    else if(token[3] == "OFF"){  boomStick(88);  }
    }

    else if(token[2] == "PinRelease"){     
    if(token[3] == "ON"){  Pin_release(180);  }
    else if(token[3] == "OFF"){  Pin_release(90);  }
    }

    else if(token[2] == "BEEP"){
    if(token[3] == "ON"){  BEEP(0);   }
    else if(token[3] == "OFF"){   BEEP(48);   }
    }

  }


  else if(token[1] == "ST"){
    if(token[2] == "GPS"){
      hh_c = 0;
      mm_c = 0;
      ss_c = 0;
      C_tt = 0;
    }
    else{
    C_tt = hour.toInt()*3600 + minute.toInt()*60 + second.toInt();
    hh_c = token[2].substring(0,2).toInt();
    mm_c = token[2].substring(3,5).toInt();
    ss_c = token[2].substring(6).toInt();
    Xbee_send(token[2].substring(0,2));
    Xbee_send(token[2].substring(3,5));
    Xbee_send(token[2].substring(6));

    }
  }


  else if(token[1] == "SIM"){
    if(token[2] == "ENABLE"){m = 1;}
    else if(token[2] == "ACTIVATE"){ SIM_ACTIVE = true;}
    else if(token[2] == "DISABLE"){m = 0;
    SIM_ACTIVE = false;}
  }


  else if(token[1] == "SIMP"){SIMP = token[4];}
  else if(token[1] == "CAL"){
    String c = BMP280();
    reference_altitude = c.substring(0,c.indexOf(",")).toInt();
    Xbee_send(String(reference_altitude));


  }else if(token[1] == "READY"){
    s = 1;
    digitalWrite(PB12, LOW);
    digitalWrite(PB13, LOW);
    digitalWrite(PB14, LOW);
  }else if(token[1] == "DATA"){
     packet = "";
    packet = BMP280()+"," + getVoltage()+"," + Gyro() +"," + Mag()+"," + GetRPS()+"," + GPS_data();
    Xbee_send(packet);
    delay(100);
     packet = "";
    packet = BMP280()+"," + getVoltage()+"," + Gyro() +"," + Mag()+"," + GetRPS()+"," + GPS_data();
    Xbee_send(packet);
    delay(100);
     packet = "";
    packet = BMP280()+"," + getVoltage()+"," + Gyro() +"," + Mag()+"," + GetRPS()+"," + GPS_data();
    Xbee_send(packet);
  }else if(token[1] == "TEST"){
    packet = "";
    if(BMP280() != ""){
      packet += "|BMP280 OK|";
    }else{packet += "|BMP280 NOT OK|";}
    if(GPS_data() != ""){
      packet = "|GPS OK|";
    }else{packet += "|GPS NOT OK|";}
    if(SD_MODE_CHECK() == 0 || SD_MODE_CHECK() == 1){
      packet += "|SD CARD OK|";
    }else{packet += "|SD CARD NOT OK|";}
    if(Gyro() != ""){
      packet += "|GYRO OK|";
    }else{packet += "|GYRO NOT OK|";}
    if(Mag() != ""){
      packet += "|Magnotometer OK|";
    }else{packet += "|Magnotometer NOT OK|";}
    if(getVoltage() != ""){
      packet += "|Voltage sensor OK|";
    }else{packet += "|Voltage sensor NOT OK|";}
    Xbee_send(packet);
    delay(500);
    packet = "";
    packet = BMP280()+"," + getVoltage()+"," + Gyro() +"," + Mag()+"," + GetRPS()+"," + GPS_data();
    Xbee_send(packet);
    delay(500);
    Xbee_send("CAMERA ON");
    digitalWrite(Camera,LOW);


    delay(1000);
    BEEP(120);
    delay(1000);
    BEEP(0);
    delay(1000);
    BEEP(48);
    Xbee_send("BEEP FINISHED");
    delay(2000);


    Pin_release(180);
    delay(2000);
    Pin_release(0);
    delay(2000);
    Pin_release(90);
    delay(2000);
    Xbee_send("SERVO FINISHED");

    boomStick(120);
    delay(2000);
    boomStick(88);
    delay(1000);
    boomStick(20);
    delay(2000);
    boomStick(88);
    Xbee_send("MOTOR FINISHED");
    delay(1000);
    
    digitalWrite(Camera,HIGH);


    Xbee_send("CAMERA FINISHED, TEST ENDED");


    



  }


   
  }
  
  
  }



void initializeSensors(){
myservo.attach(PB9);
motor.attach(PB8);  // attaches the servo on pin 9 to the servo object
beep.attach(PB1);
BEEP(48);
}







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
Xbee.println(packet);
Xbee.flush();
SD_CARD(packet);
Serial.println(packet);
packt_sent = packt_sent + 1;
}

void Mosfet(bool logic){
digitalWrite(PA0,logic);
}


String getVoltage(){

float r = analogRead(PB0);
float voltage = r * (3.3 / 4095.0);
return String(voltage/0.2);

}

void IDLE(){
  int tim = millis();
  int dur = 5000;
  float cary = getVoltage().toInt();
    if(cary >= 7.4){
    digitalWrite(PB12, LOW);
    digitalWrite(PB13, LOW);
    digitalWrite(PB14, HIGH);
    delay(100);
  }
  if(cary >= 5 && (cary < 7.4)){
    digitalWrite(PB12, LOW);
    digitalWrite(PB13, HIGH);
    digitalWrite(PB14, LOW);
    delay(100);
  }
  if(cary <= 5){
    digitalWrite(PB12, HIGH);
    digitalWrite(PB13, LOW);
    digitalWrite(PB14, LOW);
    delay(100);
  }
  Xbee_send("Send Command");
  while (millis() - tim < dur){
  if(Xbee.available()){
    String c = Xbee.readString();
    Serial.println(c);
    if(c.indexOf("CMD") == 0){
    c = c.substring(4);
    Serial.println(c);
    Handle_Commands(c);

  }}}
}



void READY(){
  BMP280();
}

void ASCENDING(){
delay(500);
packet = "";
packet = BMP280()+"," + getVoltage()+"," + Mag()+",";
Xbee_send(packet);
if(altitude < 0.85*max_alt){
  s = 3;
}

}

void DESCENDING(){
  Deploy();
  packet = "";
  packet = BMP280()+"," + getVoltage()+"," + Gyro() +"," + Mag()+"," + GetRPS()+"," + GPS_data();
  Xbee_send(packet);
  if(abs(i_pressure/pressure) <= 1.15){
    s = 4;
  }
}

void RETREIVAL(){
  boomStick(88);
  BEEP(0);
  packet = "|" + String(max_alt) + "|" + "Send Command:";
  Xbee_send(packet);
  int tim = millis();
  int dur = 5000;
  while (millis() - tim < dur){
  if(Xbee.available()){
    String c = Xbee.readString();
    Serial.println(c);
    if(c.indexOf("CMD") == 0){
    c = c.substring(4);
    Serial.println(c);
    Handle_Commands(c);

  }}}
}


void setup() {
Serial.begin(115200);
Xbee.begin(9600);  
pinMode(PA0,OUTPUT);
pinMode(Camera,OUTPUT);
pinMode(PB12,OUTPUT);
pinMode(PB13,OUTPUT);
pinMode(PB14,OUTPUT);
pinMode(PB0, OUTPUT);
analogReadResolution(12);
delay(1000);
Mosfet(HIGH);
Xbee_send("|Mosfet1 started|");
Serial.println("|Mosfet1 started|");
delay(2000);
digitalWrite(Camera,HIGH);
Wire.setSDA(PB7);
Wire.setSCL(PB6);
Wire.begin();
  // UART2 (PA2/PA3) for XBee
Xbee_send("|I2C started|");
Serial.println("|I2C started|");
delay(1500);
while(!GPS.begin(0x10)){}
Xbee_send("|GPS started|");
Serial.println("|GPS started|");
GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
delay(500);
GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
delay(500);
myservo.attach(PB9);
motor.attach(PB8);  // attaches the servo on pin 9 to the servo object
beep.attach(PB1);
BEEP(48);
while(!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)){}
Xbee_send("|BMP280 started|");
Serial.println("|BMP280 started|");
mpu.initialize();
while(!mpu.testConnection()){}
compass.init();
Xbee_send("|MPU started|");
Serial.println("|MPU started|");
SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
    SPI.setMOSI(PA7);
    SPI.setMISO(PA6);
    SPI.setSCLK(PA5);
while(!SD.begin(SD_CS)){}
Xbee_send("|SD card started|");
Serial.println("|SD CARD started|");
}


void loop(){

// put your main code here, to run repeatedly:


switch(s){
  case 0: IDLE();
  break;
  case 1: READY();
  break;
  case 2: ASCENDING() ;
  break;
  case 3: DESCENDING();
  break;
  case 4: RETREIVAL();
  break;
  }



}
