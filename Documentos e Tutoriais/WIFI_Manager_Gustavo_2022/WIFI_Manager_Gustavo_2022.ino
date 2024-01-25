#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <WiFi.h>
#define button 14

unsigned long start_time; 
unsigned long timed_event;
unsigned long current_time;

void setup() {
    WiFi.mode(WIFI_STA); //inicia o esp32 no modo estação
    Serial.begin(9600);//115200
    pinMode(button, INPUT); //setup do botão usado para resetar as credenciais
    WiFiManager wm; //chama o arquivo principal C++
    bool res;
    // res = wm.autoConnect(); //AP com nome gerado automaticamente pelo chipID
    // res = wm.autoConnect("AutoConnectAP"); //AP desprotegido
    res = wm.autoConnect("ESP32Connect","password"); //AP protegido por senha -> uso esse tipo, então nossa rede tem nome "ESP32Connect" e senha "esp32lmpt"
    if(!res)
    {
        Serial.println("Falha ao conectar...");
    } 
    else {Serial.println("Sucesso.");}
    
    timed_event = 30000;
    current_time = millis();
    start_time = current_time; 
}

void loop() {
  current_time = millis();
  //testa se há conexão wifi
  //se houver, ficará imprimindo "Conectado", para fins de debugging
   if (WiFi.status() == WL_CONNECTED)
   {
    if (current_time - start_time >= timed_event)
    {
      Serial.println("Conectado");
      start_time = current_time;
    }
  }
  //se não houver, o esp tenta se reconectar automaticamente
  if (WiFi.status() != WL_CONNECTED)
  {
    if (current_time - start_time >= timed_event)
    {
      Serial.println("Tentando reconectar...");
      WiFi.disconnect();
      WiFi.reconnect();
      start_time = current_time;
    }
  }
  //apaga as credenciais e reinicia o esp através do botão
  if (digitalRead(button) == HIGH) {
    WiFiManager wm;
    Serial.println("Resetando...");
    wm.resetSettings(); //apaga as credenciais salvas
    ESP.restart(); //reboota o código
  }
}
