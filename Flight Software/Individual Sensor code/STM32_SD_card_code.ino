#include <SD.h>
#include <SPI.h>

#define SD_CS_PIN PA4  // Change this to your actual CS pin if using SPI

void setup() {
  Serial.begin(115200);
  while (!Serial);

}
void loop() {
  
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");
    File file = SD.open("log.txt", FILE_WRITE);
    if (file) {
      file.println("Hello from STM32 (Arduino)!");
      file.close();
      Serial.println("Message written to log.txt");
    } else {
    Serial.println("Failed to open file.");
    }
  }
