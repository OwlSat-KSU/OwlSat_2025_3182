// libraries 
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <QMC5883LCompass.h>
#include <MPU6050.h>
#include <SD.h>
#include <SoftwareSerial.h>

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



//Xbee set up
SoftwareSerial XBee(PA3, PA2); // RX, TX (XBee)



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


int GPS(){
// transmit the GPS data: Long - Lat

}

void getTime(){
// read the clock in cansat to get the time

}

void Handle_Commands(String command){



}

void Xbee_send(String packet){
packet = "3182," +  String(packt_sent) +","+ mode[m] +","+ phases[s] +  "," + packet;
Xbee.print(packet);
Serial.println(packet);
packt_sent++;
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
// put your setup code here, to run once:
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
packet =  BMP280() + "," + getVoltage() + "," + String(GetRPS());
Xbee_send(packet);
delay(100);
}
