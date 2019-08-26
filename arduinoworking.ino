const int leftForward = 8;
const int leftBackward =9;
const int rightForward = 10;
const int rightBackward = 11;


void setup() {
Serial.begin(9600); // set the baud rate
Serial.println("Ready"); // print "Ready" once
pinMode(leftForward,OUTPUT);
pinMode(rightForward,OUTPUT);
pinMode(leftBackward,OUTPUT);
pinMode(rightBackward,OUTPUT);
}
void loop() {

if(Serial.available()){ // only send data back if data has been sent
char inByte = ' ';
inByte = Serial.read(); // read the incoming data
if(inByte=='Z')
{
   Serial.print("moving forward");
   digitalWrite(leftForward,HIGH);
   digitalWrite(rightForward,HIGH);
   digitalWrite(leftBackward,LOW);  
   digitalWrite(rightBackward,LOW); 
   delay(3000);  

}
else if(inByte=='L')
{
    Serial.print("taking left");
    digitalWrite(leftForward,LOW);
    digitalWrite(leftBackward,LOW);
    digitalWrite(rightForward,HIGH);
    digitalWrite(rightBackward,LOW);
    delay(3000);
}
else if(inByte=='R')
{
    Serial.print("Right command initiated");
    digitalWrite(leftForward,HIGH);
    digitalWrite(leftBackward,LOW);
    digitalWrite(rightForward,LOW);
    digitalWrite(rightBackward,LOW);
    delay(3000);
}
else
{
   digitalWrite(leftForward,LOW);
   digitalWrite(rightForward,LOW);
   digitalWrite(leftBackward,LOW);
   digitalWrite(rightBackward,LOW);
   delay(3000);
}
}
}




