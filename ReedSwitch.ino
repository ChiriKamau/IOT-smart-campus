int LEDPIN = 13;
int statePin = 2;
int switchValue;

void setup() {
  // put your setup code here, to run once:
  pinMode(LEDPIN, OUTPUT);
  pinMode(statePin,OUTPUT);

}
void loop() {
  // put your main code here, to run repeatedly:
  switchValue=digitalRead(statePin);

  if (switchValue == HIGH){
    digitalWrite(LEDPIN,HIGH);
    delay(1000);
    }
  else{
    digitalWrite(LEDPIN,LOW);
  }
  
    
  

}
