

//#include <../arduino/sam/libraries/SPI/SPI.h>


void setup() {
  // put your setup code here, to run once:
    // initialize the digital pin as an output.
  pinMode(PIN_LED, OUTPUT); 
  pinMode(PIN_LED2, OUTPUT); 
  pinMode(PIN_LED3, OUTPUT);  
  pinMode(20, OUTPUT); 
  digitalWrite(20,HIGH);
 // SPI.begin(BOARD_SPI_DEFAULT_SS);
  
  Serial.begin(115200);
  Serial.println("Hello");
}




void loop() {
  // put your main code here, to run repeatedly:
 //Serial.print("led is ");
 //Serial.println(ledstate);
 digitalWrite(PIN_LED3,HIGH);
 delay(250);
 digitalWrite(PIN_LED3,LOW);
 Serial.println("Hello");
 //byte response = SPI.transfer(BOARD_SPI_DEFAULT_SS, 0xFF);
 delay(250);
 
 //digitalWrite(19,LOW); 
// delay(1000);
}
