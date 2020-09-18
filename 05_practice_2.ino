#define PIN_LED 7

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 0); // turn on LED.
  delay(1000);
}

void loop() {
  int i = 0;
  
  while(i < 6) {
  digitalWrite(PIN_LED, 1); 
  delay(100);
  digitalWrite(PIN_LED, 0); 
  delay(100);
  i++;
  }
  
  while(1)  {
        digitalWrite(PIN_LED, 1);
  }
}
