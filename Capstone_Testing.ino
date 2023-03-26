// Pins for wind speed and direction
const int windSpeedPin = A2;
const int windDirectionPin = A0;

// Pin for rain gauge
const int rainPin = A3;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read wind speed and direction
  int windSpeed = analogRead(windSpeedPin);
  int windDirection = analogRead(windDirectionPin);

  // Convert wind speed to m/s (adjust the conversion factor to match your sensor)
  //float windSpeedMs = windSpeed;
  bool windTriggered = false;
  long lastTrigger = 0;
  Serial.print("Analog reading: ");
  Serial.print(windSpeed);
  if(windSpeed == 0 && !windTriggered){
    safeDelay(20);
    if(windSpeed == 0){
      windTriggered = true;
      Serial.print("\ttime since last trigger: ");
      Serial.print(millis() - lastTrigger);
      lastTrigger = millis();
      Serial.println("\tTriggered");
    }
  }else if(windSpeed != 0){
    windTriggered = false;
    Serial.println();
    
  }
  /**
  // Convert wind direction to degrees (adjust the conversion factor to match your sensor)
  float windDirectionDeg = windDirection * 0.36;

  // Read rain gauge
  int rainValue = analogRead(rainPin);

  // Convert rain gauge value to mm of rain (adjust the conversion factor to match your sensor)
  float rainMm = rainValue * 0.2;

  // Print the values
  Serial.print("Wind Speed: ");
  Serial.print(windSpeedMs);
  Serial.print(" m/s\tWind Direction: ");
  Serial.print(windDirectionDeg);
  Serial.print(" deg\tRain: ");
  Serial.print(rainMm);
  Serial.println(" mm");

  // Wait for a short time before reading the sensors again
  //delay(500);
  //*/
}

void safeDelay(long duration_millis){
  long startTime = millis();
  while(millis() - startTime < duration_millis){
    //delay
  }
}
