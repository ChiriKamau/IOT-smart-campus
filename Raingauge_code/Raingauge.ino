float pingPin = 2; 
float echoPin = 3; 
float maxDistance=17.79;
float area= 59.3;
float Sarea = 47.1;
float funnel_area =108.2;
float volume;
float amount_of_rain;
void setup() {
   Serial.begin(9600); 
   pinMode(pingPin, OUTPUT);
   pinMode(echoPin, INPUT);
}


void loop() {

water_volume();
   Serial.print(amount_of_rain);
   Serial.print("cm");
   Serial.println();
   delay(2000);
}



 void water_volume(){
  
      float duration, waterLevel,waterHeight;

   digitalWrite(pingPin, LOW);
   delayMicroseconds(2);

   digitalWrite(pingPin, HIGH);
   delayMicroseconds(10);

   digitalWrite(pingPin, LOW);
   pinMode(echoPin, INPUT);
   duration = pulseIn(echoPin, HIGH);
   
   waterLevel = microsecondsToCentimeters(duration);
  waterHeight= maxDistance-waterLevel;
  if(waterHeight<1.5){
     waterHeight= maxDistance-waterLevel;
    volume = waterHeight*Sarea;
  }
  else{
 waterHeight= maxDistance-1.5-waterLevel;
volume= (area*waterHeight)+50;

  }
 amount_of_rain = volume/funnel_area;

 }
float microsecondsToCentimeters(float microseconds) {
   return microseconds / 29 / 2;
}
