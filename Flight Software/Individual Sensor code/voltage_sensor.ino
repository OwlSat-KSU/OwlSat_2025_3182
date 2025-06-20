const int analogPin = A0;

const float V_REF = 7.8;

void setup() {
  Serial.begin(9600);
  analogReadResolution(12);

}

void loop() {
  // Read the analog value
  int rawValue = analogRead(analogPin);
  
  // Convert raw ADC reading to voltage
  float voltage = rawValue * (V_REF / 4095.0);

 // Serial.print("ADC Raw Value: ");
  //Serial.print(rawValue);
  Serial.print("Voltage: ");
  Serial.println(voltage, 3);  // Print voltage with 3 decimal places
  
  delay(500);
}
