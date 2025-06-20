// defining all the variables
int steps = 0;
int count = 0;
unsigned long duration = 500000;// 0.5 seconds in microseconds
bool prev_state = false;
#define NUM_SAMPLES 50
float Samples[NUM_SAMPLES] = {0};
int avg_rpm = 0;
bool SIM_mode = false;
int packt_sent = 0;
String state[5] = {"IDLE","READY","ASCENDING","DESCENDING","RETREIVAL"};


int GetRPM(){
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
steps = (steps*40); // *2 to turn 0.5s to 1s, *60 to get RPM , diving by 3 to account for 3 holes.


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

  Serial.print(" | Avg RPM:" ); 
  Serial.println(avg_rpm, 2);
}

}



int Orientation(){
// get the Magentometer data

}

void LowPowerMode() {
// turn off mosfet and sensor connection to not waste power

}

int GPS(){
// transmit the GPS data: Long - Lat

}

void getTime(){
// read the clock in cansat to get the time

}

void switchStates(int s){
// switch the states

}

void Xbee_connect(){
//establish connection to radio

}

void setup() {
// put your setup code here, to run once:
Serial.begin(460800);

}

void loop() {
// put your main code here, to run repeatedly:
 GetRPM();
}
