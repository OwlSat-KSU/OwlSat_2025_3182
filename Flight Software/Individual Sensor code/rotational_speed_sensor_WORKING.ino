#define NUM_SAMPLES 500
unsigned long duration = 1000; // 0.001 second in microseconds
int i = 0;
int steps = 0;
bool prev_state = false;
float avg_rps = 0;
float Samples[NUM_SAMPLES] = {0};
int rps = 0;

void setup() {
    Serial.begin(460800);
    pinMode(PB4, INPUT);
}

void loop() {
  steps = 0;
    unsigned long start = micros(); // Start timing

    // one measurement of rps per while loop
    while (micros() - start < duration) {
        bool current_state = digitalRead(PB4);

        if (current_state && !prev_state) { // Detect rising edge
            steps++;
            delayMicroseconds(100); // 100µs debounce
        }

        prev_state = current_state; // Update state tracking
    }

    rps = steps * 1000;

    if(i == NUM_SAMPLES + 1){
     i = 0;
      for(int j = 0; j < NUM_SAMPLES; j++){
      avg_rps = avg_rps + Samples[j];
      }
      Serial.print(" | Avg RPS: "); 
      Serial.println((avg_rps/NUM_SAMPLES), 2);
    }
    Samples[i] = rps/45;
    avg_rps = 0;
    i++;
    
    
 // Short delay before the next reading
 
}
