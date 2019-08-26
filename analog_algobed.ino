#include <ESP8266WiFi.h>
 
const char* ssid = "Akash"; // SSID i.e. Service Set Identifier is the name of your WIFI
const char* password = "randomshit"; // Your Wifi password, in case you have open network comment the whole statement.
 
const int ledPin = 13; // GPIO13 or for NodeMCU you can directly write D7 

//D1 & D2 pin of NodeMCU is connected to control Motor1.
//D3 & D4 pin of NodeMCU is connected to control Motor2.

const int leftForward = 5;//D1
const int leftBackward =4;//D2
const int rightForward = 0;//D3
const int rightBackward = 2;//D4

//function to start moving the wheel/motor of the bot
int setHigh(int motorPin)
{
  for(int i=0;i<=255;i++)
  {
    analogWrite(motorPin,i);
    delay(5);//Has to be changed according to bot requirements
  }
}

//function to stop the wheel/motor of the bot
int setLow(int motorPin)
{
  for(int i=255;i>=0;i--)
  {
    analogWrite(motorPin,i);
    delay(5);
  }
}

WiFiServer server(80);// Creates a server that listens for incoming connections on the specified port, here in this case port is 80.
 
void setup() {
  Serial.begin(115200);
  delay(10);
 
  pinMode(ledPin, OUTPUT);
  
  // set control pins as Output
  pinMode(leftForward,OUTPUT);
  pinMode(leftBackward,OUTPUT);
  pinMode(rightForward,OUTPUT);
  pinMode(rightBackward,OUTPUT);


  digitalWrite(ledPin, LOW);
 
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
 
  // Start the server
  server.begin();
  Serial.println("Server started");
 
  // Print the IP address
  Serial.print("Use this URL to connect: ");
  Serial.print("http://");
  Serial.print(WiFi.localIP()); //Gets the WiFi shield's IP address and Print the IP address of serial monitor
  Serial.println("/");
 
}
 
void loop() {
  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
 
  // Wait until the client sends some data
  Serial.println("new client");
  while(!client.available()){
    delay(1);
  }

  while(client.available()>0)
  {
    String mssg = client.readStringUntil('\r');
    Serial.println(mssg);
    Serial.flush();//To ensure all the data has been sent and buffer is empty
    mssg.remove(0,5);//Maybe to remove initial garbage values
    mssg.remove(mssg.length()-9,9);//Maybe to remove ending garbage values
    

  // run forward
  if(mssg=="forward")
  {
    setHigh(leftForward);
    setLow(leftBackward);
    setHigh(rightForward);
    setLow(rightBackward);
  
    delay(1000);
  }
    // run backward
    else if(mssg=="backward")
  {
    setLow(leftForward);
    setHigh(leftBackward);
    setLow(rightForward);
    setHigh(rightBackward);
  
  
    delay(1000);
  }
    // run right
    else if(mssg=="right")
  {
    setHigh(leftForward);
    setLow(leftBackward);
    setLow(rightForward);
    setLow(rightBackward);
  
    delay(1000);
  }
    
    // run left
    else if(mssg=="left")
  {
    setLow(leftForward);
    setLow(leftBackward);
    setHigh(rightForward);
    setLow(rightBackward);
  
  
    delay(1000);
  }

  else if(mssg=="stop")
  {
    setLow(leftForward);
    setLow(leftBackward);
    setLow(rightForward);
    setLow(rightBackward);
  
    delay(1000);
  }
  
 }
    delay(1);
    Serial.println("Client disconnected");
    Serial.println("");
}
