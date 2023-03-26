// Pins for wind speed and direction
const int windSpeedPin = A2;


bool windTriggered = false;
long lastTrigger = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Read wind speed and direction
  int windSpeed = analogRead(windSpeedPin);

  // Convert wind speed to m/s (adjust the conversion factor to match your sensor)
  //float windSpeedMs = windSpeed;
  
  if(windSpeed == 0 && !windTriggered){
    safeDelay(20);
    if(windSpeed == 0){
      windTriggered = true;
      Serial.print("\ttime since last trigger: ");
      Serial.println(millis() - lastTrigger);
      lastTrigger = millis();
    }
  }else if(windSpeed > 200){
    windTriggered = false;
    
  }
}

void safeDelay(long duration_millis){
  long startTime = millis();
  while(millis() - startTime < duration_millis){
    //delay
  }
}
