#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFiMulti.h>
/* Set these to your desired credentials. */
/*const char *ssid = "Roboducks";
  const char *password = "thiswillwork";*/
ESP8266WiFiMulti WiFiMulti;
WiFiServer server(80); // creates a wifi server.

/* Just a little test message.  Go to http://192.168.4.1 in a web browser
   connected to this access point to see it.
*/
/*void handleRoot() {
	server.send(200, "text/html", "<h1>You are connected</h1>");
  }*/

void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println();
  //Serial.print("Configuring access point...");
  /* You can remove the password parameter if you want the AP to be open. */
  //WiFi.softAP(ssid, password); // softAP means virtual AP

  //IPAddress myIP = WiFi.softAPIP();  /// Stores IP address
  //Serial.print("AP IP address: ");
  //Serial.println(myIP);
  /*server.on("/", handleRoot);
    server.begin();*/
  WiFiMulti.addAP("AndroidAP", "robo1234");

  Serial.println();
  Serial.println();
  Serial.print("Wait for WiFi... ");

  while (WiFiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("HTTP server started");
  server.begin();                            // Tells server to start listenng for incomming calls
  //Serial.printf("Web server started, open %s in a web browser\n", WiFi.localIP().toString().c_stcr());      //converts to string.
}


// prepare a web page to be send to a client (web browser)
String prepareHtmlPage()
{
  String htmlPage =
    String("HTTP/1.1 200 OK\r\n") +
    "Content-Type: text/html\r\n" +
    "Connection: close\r\n" +  // the connection will be closed after completion of the response
    // refresh the page automatically every 5 sec
    "\r\n" +
    "<!DOCTYPE HTML>" +
    "<html>" +
    "<head>" +
    "</head>" +
    "<body>"
    "Analog input:  " + String(analogRead(A0)) +
    "</body>" +
    "</html>" +
    "\r\n";
  return htmlPage;
}

void loop()
/*{
	server.handleClient();
  }*/
{
  WiFiClient client = server.available();
  // wait for a client (web browser) to connect
  if (client)
  {
    Serial.println("\n[Client connected]");
    while (client.connected())
    {
      // read line by line what the client (web browser) is requesting
      if (client.available())
      {
        String line = client.readStringUntil('\r');
        Serial.print(line);
        // wait for end of client's request, that is marked with an empty line
        if (line.length() == 1 && line[0] == '\n')
        {
          client.println(prepareHtmlPage());
          break;
        }
      }
    }
    delay(1); // give the web browser time to receive the data
    // close the connection:
    client.stop();
    Serial.println("[Client disonnected]");
  }
}













