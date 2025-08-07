#include <PS4Controller.h>
#include <ps4.h>
#include <ps4_int.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>  // Include OTA library

#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

String recieved = "none";

// Wi-Fi credentials
const char* ssid = "Galaxy s23";      // Replace with your Wi-Fi SSID
const char* password = "tushar1234";  // Replace with your Wi-Fi password

WiFiServer telnetServer(23);  // Telnet server will run on port 23 (standard Telnet port)
WiFiClient telnetClient;

class TelnetRead {
public:
  String data;
  // Function to read data from the Telnet client
  void readData() {
    if (telnetClient && telnetClient.connected() && telnetClient.available()) {
      String inputdata = telnetClient.readStringUntil('\n');
      inputdata.trim();  // Remove any newline or extra spaces
      data = inputdata;
    }
  }
  // Function to return the received data
  String getData() {
    return data;
  }
};
// Instantiate the TelnetRead object
TelnetRead Tel;
char receivedData[64];
byte myByte = B11111111;
byte psdt[5] = { 0 };

int w = 0;
int i = 0;
int aa = 0;
int bb = 0;
void psData() {

  uint8_t l2_val = PS4.L2Value();
  uint8_t r2_val = PS4.R2Value();
  w = l2_val - r2_val;
  aa = PS4.LStickX();
  bb = PS4.LStickY();
}

void setup() {
  Serial.begin(115200);
  PS4.begin();
  PS4.attach(psData);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  Wire.begin(8);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  Serial.println("Connecting to Wi-Fi...");
  removePairedDevices();  // This helps to solve connection issues

  // Connect to Wi-Fi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }
  Serial.println("Connected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());  // Print ESP32 IP address

  // Start the Telnet server
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  Serial.println("Telnet Server started");

  // Start the OTA functionality
  ArduinoOTA.begin();
}
void loop() {
  ArduinoOTA.handle();  // Handle OTA requests
  new_client_connection();
  if (Serial2.available()) {
    recieved = Serial2.readStringUntil('z');
   
  }
   TelnetPrint(recieved + "  vxps:"+String(aa) +"  vyps:"+String(bb)+"  weps:"+String(w/2) );
  Serial.println(recieved);
}
void requestEvent() {

  psdt[2] = map(w, -255, 255, 0, 255);
  psdt[3] = map(aa, -128, 127, 0, 255);
  psdt[4] = map(bb, -128, 127, 0, 255);

  Wire.write(psdt, 5);
}

void onConnect() {
  Serial.println("Connected!");
}

void onDisConnect() {
  Serial.println("Disconnected!");
}
// Function to check and handle new Telnet client connections
void new_client_connection() {
  if (telnetServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();    // Stop the previous client
      telnetClient = telnetServer.available();  // Assign the new client
      Serial.println("New Telnet client connected");
    }
  }
}

void printDeviceAddress() {
  const uint8_t* point = esp_bt_dev_get_address();
  for (int i = 0; i < 6; i++) {
    char str[3];
    sprintf(str, "%02x", (int)point[i]);
    Serial.print(str);
    if (i < 5) {
      Serial.print(":");
    }
  }
}

void receiveEvent(int bytes) {
  int i = 1;  //0

  // Read all available bytes into the buffer
  while (Wire.available() > 0 && i < sizeof(receivedData) - 1) {
    receivedData[i++] = Wire.read();
  }
  receivedData[i] = '\0';  // Null-terminate the string

  // Print the received string
  Serial.print("Received string: ");
  Serial.println(receivedData);
  TelnetPrint(receivedData);
}

// Function to send data to Telnet client
void TelnetPrint(String data) {
  if (telnetClient && telnetClient.connected()) {
    telnetClient.println(data);
  } else {
    Serial.println("Telnet client not connected.");
  }
}

void removePairedDevices() {
  uint8_t pairedDeviceBtAddr[20][6];
  int count = esp_bt_gap_get_bond_device_num();
  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  for (int i = 0; i < count; i++) {
    esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  }
}
