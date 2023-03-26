// Pins for wind speed and direction
const int analogWindPin = A2;

int windSpeed; //in m/s

bool windTriggered = false;
long lastTrigger = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Read wind speed and direction
  int analogWind = analogRead(analogWindPin);
  
  if(analogWind == 0 && !windTriggered){
    if(analogWind == 0){
      windTriggered = true;;
      windSpeed = 840/(millis() - lastTrigger);
      lastTrigger = millis();
    }
  }else if(analogWind > 50){
    windTriggered = false;
    
  }
  Serial.println(windSpeed);
}

void safeDelay(long duration_millis){
  long startTime = millis();
  while(millis() - startTime < duration_millis){
    //delay
  }
}