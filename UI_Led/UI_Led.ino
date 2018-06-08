#include<ESP8266WiFi.h>
#include<ESP8266WebServer.h>

ESP8266WebServer server(80);

char* username = "kamal";
char* password = "kamal123";
String html = "kamal";

void setup()
{
  WiFi.begin(username, password);
 
  Serial.begin(115200);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.localIP());
  server.on("/", send_html);
  server.on("/R1", red_on);
  server.on("/R0", red_off);
  server.on("/G1", green_on);
  server.on("/G0", green_off);
  server.on("/B1", blue_on);
  server.on("/B0", blue_off);
  server.begin();
}


void loop()
{
  server.handleClient();
}

void send_html()
{
  html ="<!DOCTYPE html> <html> <head> <title>IISC DESE INTERNSHIP</title> </head> <style type=\"text/css\"> .center{ text-align: center; } .head-color{ color : #092040; } .button { border: none; color: white; padding: 15px 32px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; } </style> <body> <h1 class = \"center head-color\"><strong>IISC DESE INTERNSHIP</strong></h1><br><br> <!-- <h2 class = \"head-color center\" >LED Status: ";
html += "RGB";
html +="</h2> --> <br> <br> <div class = \"center\"> <form action = \"R1\" method=\"GET\"> <input type=\"submit\" style=\"background-color:red\" class = \"button\" value=\"RED ON\"/> </form> <form action = \"R0\" method=\"GET\"> <input type=\"submit\" style=\"background-color:grey\" class = \"button\" value=\"RED OFF\"/> </form> <br> <form action = \"B1\" method=\"GET\"> <input type=\"submit\" style=\"background-color:blue\" class = \"button\" value=\"BLUE ON\"/> </form> <form action = \"B0\" method=\"GET\"> <input type=\"submit\" style=\"background-color:grey\" class = \"button\" value=\"BLUE OFF\"/> </form> <br> <form action = \"G1\" method=\"GET\"> <input type=\"submit\" style=\"background-color:green\" class = \"button\" value=\"GREEN ON\"/> </form> <form action = \"G0\" method=\"GET\"> <input type=\"submit\" style=\"background-color:grey\" class = \"button\" value=\"GREEN OFF\"/> </form> </div> </body> </html>";

  server.send(200, "text/html", html);
}

void red_on()
{
  Serial.println("R1");
  server.send(200, "text/html", html);
}
void red_off()
{
  Serial.println("R0");
  server.send(200, "text/html", html);
}
void green_on()
{
  Serial.println("G1");
  server.send(200, "text/html", html);
}
void green_off()
{
  Serial.println("G0");
  server.send(200, "text/html", html);
}
void blue_on()
{
  Serial.println("B1");
  server.send(200, "text/html", html);
}
void blue_off()
{
  Serial.println("B0");
  server.send(200, "text/html", html);
}

