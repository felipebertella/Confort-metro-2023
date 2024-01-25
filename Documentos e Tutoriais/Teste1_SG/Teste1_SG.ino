#include <Arduino.h>
#include <WiFi.h>
#include <esp_wpa2.h>
#include <esp_wifi.h>
#include <HTTPClient.h>
#include "config.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "secrets_test.h"

String GOOGLE_SCRIPT_ID = googleScriptID;
String GOOGLE_SHEET_NAME = sheetName;

const int httpsPort = 443;

WiFiClientSecure client;

void enviarMedicao(void);
void connect_wifi(void);

//---------------------------------------------------------------------------------------------
void setup()
{

  Serial.begin(9600);//115200
  delay(10);
  Serial.println();
  connect_wifi();

}

//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------

void loop()
{
  enviarMedicao();
  Serial.print("POST data to spreadsheet:");
  delay(5000);

}

//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------

void connect_wifi(void)
{

  Serial.print("Connecting to wifi: ");
  Serial.println(ssid);
  Serial.flush();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {

    delay(500);
    Serial.print(WiFi.status());
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

//---------------------------------------------------------------------------------------------

void enviarMedicao()
{

  String url = String("https://script.google.com") + "/macros/s/" + GOOGLE_SCRIPT_ID + "/exec?" + "value1=" + String(19) + "&value2=" + String(18) + "&value3=" + String(9);
  // https://script.google.com/macros/s/

  Serial.print("Making a request");

  HTTPClient http;
  http.begin(url.c_str()); //Specify the URL and certificate
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  int httpCode = http.GET();
  String payload;

  if (httpCode > 0) { //Check for the returning code

    payload = http.getString();
    Serial.println("Payload: " + payload);

  }
  else
  {
    Serial.println("Error on HTTP request");
  }
  http.end();

}
