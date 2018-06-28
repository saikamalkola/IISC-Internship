#include<ESP8266WiFi.h>
#include<ESP8266WebServer.h>

ESP8266WebServer server;
const char *ssid = "SelfNav"; // The name of the Wi-Fi network that will be created
const char *password = "IISCDESE";   // The password required to connect to it, leave blank for an open network

String html = "kamal";

//Static IP address configuration
IPAddress staticIP(192, 168, 43, 90); //ESP static ip
IPAddress gateway(192, 168, 43, 1);   //IP Address of your WiFi Router (Gateway)
IPAddress subnet(255, 255, 255, 0);  //Subnet mask
IPAddress dns(8, 8, 8, 8);  //DNS
  
void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(staticIP,gateway,subnet);
  WiFi.softAP(ssid, password); 
  delay(500);
  
  server.on("/", send_html);
  server.on("/room_num", tx_room_num);
  server.begin();
}


void loop()
{
  server.handleClient();
}

void send_html()
{
  String html ="<!DOCTYPE html> <html> <head> <title>IISC DESE INTERNSHIP</title> </head> <style type=\"text/css\"> .center{ text-align: center; } .head-color{ color : #092040; } select#soflow, select#soflow-color { -webkit-appearance: button; -webkit-border-radius: 2px; -webkit-box-shadow: 0px 1px 3px rgba(0, 0, 0, 0.1); -webkit-padding-end: 20px; -webkit-padding-start: 2px; -webkit-user-select: none; background-image: url(http://i62.tinypic.com/15xvbd5.png), -webkit-linear-gradient(#FAFAFA, #F4F4F4 40%, #E5E5E5); background-position: 97% center; background-repeat: no-repeat; border: 1px solid #AAA; color: #555; font-size: inherit; margin: 20px; overflow: hidden; padding: 5px 10px; text-overflow: ellipsis; white-space: nowrap; width: 300px; } select#soflow-color { color: #000; background-image: url(http://i62.tinypic.com/15xvbd5.png), -webkit-linear-gradient(#3498DB, #3498DB 40%, #3498DB); background-color: #3498DB; -webkit-border-radius: 20px; -moz-border-radius: 20px; border-radius: 20px; padding-left: 15px; } </style> <body> <h1 class = \"center head-color\"><strong>IISC DESE INTERNSHIP</strong></h1><br><br> <br> <br> <div align=\"center\"> <form action = \"room_num\" method=\"GET\"> <!--surround the select box within a \"custom-select\" DIV element. Remember to set the width:--> <select id=\"soflow-color\" name=\"Selected_room\" onchange=\"this.form.submit()\"> <option value=\"-1\">Select Room:</option> <option value=\"00\"> Embedded Systems Lab - Origin</option> <option value=\"01\"> 214 </option> <option value=\"02\"> 227 </option> <option value=\"03\"> 213 </option> <option value=\"04\"> 228 </option> <option value=\"05\"> 229 </option> <option value=\"06\"> 230 </option> <option value=\"07\"> 212 </option> <option value=\"08\"> 211 </option> <option value=\"09\"> 231 </option> <option value=\"10\"> 210 </option> <option value=\"11\"> 232 </option> <option value=\"12\"> 233 </option> <option value=\"13\"> 234 </option> <option value=\"14\"> 209 </option> <option value=\"15\"> End of Corridor 0 </option> <option value=\"16\"> 207 </option> <option value=\"17\"> 235 </option> <option value=\"18\"> 236 </option> <option value=\"19\"> 206 </option> <option value=\"20\"> 205 </option> <option value=\"21\"> End of Corridor 1 </option> <option value=\"22\"> 201 </option> <option value=\"23\"> Door </option> <option value=\"24\"> Lift </option> <option value=\"25\"> Stairs </option> <option value=\"26\"> 230 </option> <option value=\"27\"> Door </option> <option value=\"28\"> 215 </option> <option value=\"29\"> 242 </option> <option value=\"30\"> 240 </option> <option value=\"31\"> Door </option> <option value=\"32\"> 221 </option> <option value=\"33\"> 222 </option> <option value=\"34\"> 223 </option> <option value=\"35\"> 224 </option> <option value=\"36\"> 220 </option> <option value=\"37\"> End of Corridor - 2</option> <option value=\"38\"> 219 </option> <option value=\"39\"> 225 </option> <option value=\"40\"> 226 </option> <option value=\"41\"> 218 </option> <option value=\"42\"> 217 </option> </select> </form> </div> </body> </html>";
  server.send(200, "text/html", html);
}

void tx_room_num()
{
  String room = server.arg("Selected_room");
  Serial.println(room);
  server.send(200, "text/html", html);
}
