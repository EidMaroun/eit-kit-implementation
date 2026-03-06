#include <EITKitArduino.h>
// We instantiate the class here, but ALL hardware setup happens in setup()
EITKitArduino eit(16, 1, 4, AD, AD, false);
void setup() {
  Serial.begin(115200);
  
  // Wait for serial to connect (max 2 seconds)
  while (!Serial && millis() < 2000) {} 
  
  Serial.println("=====================================");
  Serial.println("Starting EIT system...");
  Serial.println("=====================================");
  
  // Actually initialize the hardware now that Serial is ready
  eit.begin(); 
  // One-time hardware sanity check (comment out after bring-up if desired)
  eit.diagnose_adc_bus();
}
void loop() {
  // taking measurements will output as `FRAME_DATA, val1, val2, ...` to the monitor
  eit.take_measurements(AD, AD);
  delay(5);
}
