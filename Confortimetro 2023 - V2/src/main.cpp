#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <RTClib.h>
#include <Adafruit_SH110X.h>
#include <Ch376msc.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <WiFi.h>
#include "secrets_test.h" //https://script.google.com/macros/s/AKfycbwNyYF7dFzu1bpxnLxsNjKznJCYEQS8ii-UEcvL0s-bosbYbK3AIMEhmlw4EULSwzqI/exec
#include <esp_wpa2.h>
#include <esp_wifi.h>
#include <HTTPClient.h>

#define num_sensor_Tar 36
#define num_sensor_Tglobo 35
#define num_sensor_Tquente 39
#define num_sensor_HR 32

String GOOGLE_SCRIPT_ID = googleScriptID;
String GOOGLE_SHEET_NAME = sheetName;
const int httpsPort = 443;
WiFiClientSecure client;
#define button 27

int read_raw;
esp_adc_cal_characteristics_t adc1_chars;
esp_adc_cal_characteristics_t adc_cal;

int cont = 0;
Ch376msc flashDrive(Serial2);

/* Uncomment the initialize the I2C address , uncomment only one, If you get a totally blank screen try the other*/
#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//----------------------- Pen drive --------------------------------------------

int cont_gravacao = 0;
String nome_arquivo;
String dataString = "";         
bool flagDatalog = true; 
bool flagOLED = false; 
bool flagPost = false; 
bool firstsave = true; 
bool failed_save = false; 
bool firstPost = true;
String logCheck = ""; 

RTC_DS3231 rtc;
String dia,mes,ano,hora,minuto,segundo;

int num_leituras_media = 100;
float a[10], b[10], c[10], d[10], e[10], f[10];
float av, bv, cv, dv, ev, min_delta, max_delta;
float V1,V2,V3,V4,V5,V6,V7,V8;
float Tar, Tglobo, Veloc, UR, Tquente, Delta_T, DT, UV1, UV2, HR;
float soma_Tar_OLED = 0, soma_Tglobo_OLED = 0,soma_Veloc_OLED = 0,soma_UR_OLED = 0, soma_UV1_OLED = 0 ,soma_UV2_OLED = 0, soma_HR_OLED = 0;
float soma_Tar_gravacao = 0, soma_Tglobo_gravacao = 0,soma_Veloc_gravacao = 0,soma_UR_gravacao = 0, soma_UV1_gravacao = 0, soma_UV2_gravacao = 0, soma_HR_gravacao;
int leituras_gravacao = 0, leituras_OLED=0;


char adat[]="Vivamus nec nisl molestie, blandit diam vel, varius mi. Fusce luctus cursus sapien in vulputate.\n";
char adat2[] = "000000000000";


//Declaração de funções que serão definidas depois
double Calibrar_tensao(double tensao, int canal);
double Leitura_Tensao_Canal(int canal, float n_amostras, int canal_calibracao);
void update_sums();
void datalog();
void getTime();
void configure_constants();
void oled();
void PostData();

//Function to inicialize timers and configure interruptions
void inicialize_timers()  //timer for interruptions - esp32u 40Mhz
{
  void IRAM_ATTR data_log();
  void IRAM_ATTR oled_update(); 
  void IRAM_ATTR post_data();

  hw_timer_t *data_log_timer = NULL;
  data_log_timer = timerBegin(0, 8000, true);
  timerAttachInterrupt(data_log_timer, &data_log, true);
  timerAlarmWrite(data_log_timer, 150000, true); //change middle number to change the interruption timer 10000 = 1s
  timerAlarmEnable(data_log_timer);

  hw_timer_t *oled_update_timer = NULL;
  oled_update_timer = timerBegin(1,8000,true);
  timerAttachInterrupt(oled_update_timer, &oled_update, true);
  timerAlarmWrite(oled_update_timer, 35000, true);
  timerAlarmEnable(oled_update_timer);


  hw_timer_t *status_timer = NULL; //every 7 minutes
  status_timer = timerBegin(2, 8000, true);
  timerAttachInterrupt(status_timer, &post_data, true);
  timerAlarmWrite(status_timer, 500000, true); //change middle number to change the interruption timer 10000 = 1s
  timerAlarmEnable(status_timer);
}
void IRAM_ATTR data_log(){
  flagDatalog = true; 
}
void IRAM_ATTR oled_update(){
  flagOLED = true;  
}
void IRAM_ATTR post_data(){
  flagPost = true;
}


bool setupWifi(){
  WiFi.mode(WIFI_STA); //inicia o esp32 no modo estação
  WiFiManager wm; //chama o arquivo principal C++
  bool res;
    // res = wm.autoConnect(); //AP com nome gerado automaticamente pelo chipID
    // res = wm.autoConnect("AutoConnectAP"); //AP desprotegido
  res = wm.autoConnect("ESP32Connect","password"); //AP protegido por senha -> uso esse tipo, então nossa rede tem nome "ESP32Connect" e senha "esp32lmpt"
  if(!res)
  {
    Serial.println("Falha ao conectar...");
    return res;
  } 
  else
  {
    Serial.println("Sucesso.");
    return res; 
  }

}

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  flashDrive.init();

  esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_cal);//Inicializa a estrutura de calibracao
  int read = analogRead(32);
  read+=1;
  Serial.print(read); 
  //pinMode(32, INPUT_PULLDOWN); 
  if(!display.begin(0x3C,true)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }
  
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);

  display.setCursor(3, 0);
  // Display static text
  display.println("Confortimetro");
  display.println("");
  display.println("LabEEE + LabTermo");
  display.println("");
  display.println("UFSC 2023");
  display.display(); 

  display.clearDisplay();

  pinMode(2, OUTPUT);
  Wire.begin();
  rtc.begin();
  //rtc.adjust(DateTime(2023, 10, 30, 12, 00, 00));
  inicialize_timers();
  configure_constants();
  
  setupWifi();

  
}

void loop() {
  //leitura das entradas analógicas
  leituras_gravacao += 1;
  leituras_OLED +=1; 
  Tar = Leitura_Tensao_Canal(num_sensor_Tar, num_leituras_media, 1); 
  Tglobo = Leitura_Tensao_Canal(num_sensor_Tglobo, num_leituras_media, 2);
  Tquente = Leitura_Tensao_Canal(num_sensor_Tquente, num_leituras_media, 4); 
  HR = Leitura_Tensao_Canal(num_sensor_HR,num_leituras_media,1);
  //calculo velocidade do ar
  Delta_T = Tquente - Tar;
  if (Delta_T <=  min_delta)   
    Delta_T =  min_delta;

  if (Delta_T >=  max_delta)   
    Delta_T =  max_delta;
    
  //DT = exp(1/Delta_T);
  DT = 1/Delta_T;
  Veloc = av*DT*DT*DT*DT + bv*DT*DT*DT + cv*DT*DT + dv*DT + ev;
  //Serial.println(Veloc);
  if (Veloc <= 0.0) 
    Veloc = 0.0;         
    //  Veloc = 0.5;

  update_sums();

  if (flagDatalog)
    datalog();
  if (flagOLED)
    oled(); 
}
//Função para coletar o horário
void getTime()
{
  int int_dia, int_mes, int_segundo, int_minuto, int_hora; 
  
  DateTime now = rtc.now();
  int_dia = (now.day());
  int_mes = (now.month());
  ano = String(now.year());
  int_hora = (now.hour());
  int_minuto = (now.minute());
  int_segundo = (now.second());
  

  if (int_dia < 10)    dia = ("0"+String(int_dia));
  else  dia = String(int_dia);
  
  if (int_mes < 10)  mes = ("0"+String(int_mes)); 
  else  mes = String(int_mes);
   
  if (int_hora < 10)  hora = ("0"+String(int_hora)); 
  else  hora = String(int_hora);
 
  if (int_minuto < 10)  minuto = ("0"+String(int_minuto)); 
  else  minuto = String(int_minuto);
 
  if (int_segundo < 10)  segundo = ("0"+String(int_segundo)); 
  else  segundo = String(int_segundo);
 
}

//Atualiza o display
void oled(){
  display.clearDisplay();
  Tar = soma_Tar_OLED/leituras_OLED; 
  Tglobo = soma_Tglobo_OLED/leituras_OLED;
  Veloc = soma_Veloc_OLED/leituras_OLED; 
  HR = soma_HR_OLED/leituras_OLED;

  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("Temp Ar: " + String(Tar)+"C");
  display.println("Temp Globo: " + String(Tglobo)+"C");
  display.println("Veloc Ar: " + String(Veloc)+"m/s");
  display.println("Umidade Ar: "+ String(HR)+"%");
  display.println("");
  display.println(hora + ":" + minuto + " "+ dia + "/" + mes + "/" + ano);
  display.println("");
  display.println("Log Status: "+logCheck);
  display.display();
  
  soma_Tar_OLED = 0;
  soma_Tglobo_OLED = 0;
  soma_Veloc_OLED = 0;
  soma_HR_OLED = 0;
  leituras_OLED = 0; 
  flagOLED = false; 
}

void datalog(){
  digitalWrite(2,HIGH); 
  Serial.println(cont);
  cont++; 
  Tar = soma_Tar_gravacao/leituras_gravacao; 
  Tglobo = soma_Tglobo_gravacao/leituras_gravacao;
  Veloc = soma_Veloc_gravacao/leituras_gravacao; 
  HR = soma_HR_gravacao/leituras_gravacao;
  getTime();
  dataString = dia +"/"+mes+"/"+ano+","+hora+":"+minuto+ ":" + segundo+ "," +String(Tar) + "," + String(Tglobo) + "," + String(Veloc)+ "," + String(HR) +"\n"; 
 
  if (firstsave){
    String titulo = "Data, Hora, Temperatura Ar, Temperatura Globo, Veloc Ar, Umidade Relativa \n";
    nome_arquivo = (dia+mes+hora+minuto+".CSV"); 
    for (int i = 0; i < (nome_arquivo.length()+1); i++) {
      adat2[i] = nome_arquivo[i];
    }
    for (int i = 0; i < (titulo.length()+1); i++) {
    adat[i] = titulo[i];
  }
  for (int i = 0; i < 50; i++) {
    if(flashDrive.checkIntMessage()){
		  if(flashDrive.getDeviceStatus()){
			  Serial.println(F("Flash drive attached!"));
		  } 
      else {
			  Serial.println(F("Flash drive detached!"));
	    }
    }}

    flashDrive.setFileName(adat2);  //set the file name
    flashDrive.openFile();                //open the file
    flashDrive.writeFile(adat, strlen(adat)); //string, string length
    flashDrive.closeFile();               //at the end, close the file
    firstsave = false; 

  }
  
  else{
    for (int i = 0; i < (dataString.length()+1); i++) {
    adat[i] = dataString[i];
    }
    flashDrive.setFileName(adat2);  //set the file name
        if(flashDrive.openFile() == ANSW_USB_INT_SUCCESS){               //open the file
        	flashDrive.moveCursor(CURSOREND); 
          flashDrive.writeFile(adat, strlen(adat)); 
          logCheck = "Ok";   //if the file exist, move the "virtual" cursor at end of the file, with CURSORBEGIN we actually rewrite our old file
         }
        else
          logCheck = "Failed";
    flashDrive.closeFile();
  }

soma_Tar_gravacao = 0;
soma_Tglobo_gravacao = 0;
soma_Veloc_gravacao = 0;
soma_HR_gravacao = 0; 
leituras_gravacao = 0;
digitalWrite(2,LOW); 
Serial.print(adat);
flagDatalog = false; 
}

void configure_constants()
{
  // 5.831754328128670 -48.71556076969560  160.6772385909790 -258.88667519170  230.41072706194 -82.324
  a[1] = 5.831754328128670; b[1] =-48.71556076969560; c[1] = 160.6772385909790; d[1] = -258.88667519170; e[1] = 230.41072706194;  f[1] = -82.324; 
  a[2] = 5.831754328128670; b[2] =-48.71556076969560; c[2] = 160.6772385909790; d[2] = -258.88667519170; e[2] = 230.41072706194;  f[2] = -82.324; 
  a[3] = 0; b[3] =0; c[3] = 0; d[3] = 0; e[3] = 47.646;  f[3] = -28.3; 
  a[4] = 5.831754328128670; b[4] =-48.71556076969560; c[4] = 160.6772385909790; d[4] = -258.88667519170; e[4] = 230.41072706194;  f[4] = -82.324; 
  a[5] = 0; b[5] =0; c[5] = 0; d[5] = 0; e[5] = 1.0;  f[5] = 0; 
  a[6] = 0; b[6] =0; c[6] = 0; d[6] = 0; e[6] = 1.0;  f[6] = 0;  
  //  Constantes anemometro
  //y = -1128472.76648x4 + 221193.12268x3 - 14374.35858x2 + 414.57681x - 4.46274
  av = -1128472.76648;
  bv = 221193.12268;
  cv = - 14374.35858;
  dv = 414.57681;
  ev = - 4.46274;
  min_delta = 14.0;
  max_delta = 37.5;
}

double Calibrar_tensao(double tensao, int canal) 
{
  double Valor_calibrado;
  const double beta = 3964.0;
  const double r0 = 15000.0;
  const double t0 = 273.0+ 25.0;
  const double rx = r0 * exp(-beta/t0);

  double rt = (r0*3.3-tensao*r0)/tensao;
  
  Valor_calibrado = (beta/(log(rt/rx))) - 273.15;
  //Valor_calibrado = (t-273.0);

  //Valor_calibrado = (1/(0.00091+(0.00024*log(rt))))-273.15;
  /*Valor_calibrado =   a[canal]*pow(tensao, 5) +
                       b[canal]*pow(tensao, 4) +
                       c[canal]*pow(tensao, 3) +
                       d[canal]*pow(tensao, 2) +
                       e[canal]*pow(tensao, 1) +
                       f[canal]*pow(tensao, 0);*/
                    
  return Valor_calibrado; 
}

double Leitura_Tensao_Canal(int canal, float n_amostras, int canal_calibracao) 
{
  
  float voltage = 0;
  double media_leituras_calibrado = 0;
  int i = 0;
  double media_leituras = 0;
  switch (canal)
  {
  case 32:
    for (i = 0; i < n_amostras; i++) 
  {    media_leituras += adc1_get_raw(ADC1_CHANNEL_4);  
  }
    media_leituras /= 100;
    media_leituras = esp_adc_cal_raw_to_voltage(media_leituras, &adc_cal);
    voltage = (media_leituras/1000);
    //Serial.println(voltage);
    media_leituras_calibrado = ((voltage)-(0.1515*3.3))/(3.3*0.00636);
    //media_leituras_calibrado = media_leituras_calibrado/(1.0546 - 0.00216*Tar);
    break;
  case 35:
     for (i = 0; i < n_amostras; i++) 
  {    media_leituras += adc1_get_raw(ADC1_CHANNEL_7);  
  }
    media_leituras /= 100;
    media_leituras = esp_adc_cal_raw_to_voltage(media_leituras, &adc_cal);
    voltage = (media_leituras/1000);
    media_leituras_calibrado = Calibrar_tensao(voltage, canal_calibracao);
    //Serial.println(media_leituras_calibrado);
    break;
  case 36:
    for (i = 0; i < n_amostras; i++) 
  {    media_leituras += adc1_get_raw(ADC1_CHANNEL_0);  
  }
    media_leituras /= 100;
    media_leituras = esp_adc_cal_raw_to_voltage(media_leituras, &adc_cal);
    voltage = (media_leituras/1000);
    media_leituras_calibrado = Calibrar_tensao(voltage, canal_calibracao);
    //Serial.println(media_leituras_calibrado);
    break;
  case 39:
     for (i = 0; i < n_amostras; i++) 
  {    media_leituras += adc1_get_raw(ADC1_CHANNEL_3);  
  }
    media_leituras /= 100;
    media_leituras = esp_adc_cal_raw_to_voltage(media_leituras, &adc_cal);
    voltage = (media_leituras/1000);
    media_leituras_calibrado = Calibrar_tensao(voltage, canal_calibracao);
    //Serial.println(media_leituras_calibrado);
  default:
    break;
  }
  
  return media_leituras_calibrado;
}


void update_sums(){
  soma_Tar_OLED += Tar;
  soma_Tglobo_OLED += Tglobo;
  soma_Veloc_OLED += Veloc;
  soma_HR_OLED += HR;
  soma_Tar_gravacao += Tar; 
  soma_Tglobo_gravacao += Tglobo;
  soma_Veloc_gravacao += Veloc; 
  soma_HR_gravacao += HR; 
}

void PostData()
{
  String url = ""; 
  if (firstPost){
    url = String("https://script.google.com") + "/macros/s/" + GOOGLE_SCRIPT_ID + "/exec?" + "value1=" + "Temperatura Ar" + 
                "&value2=" + "Temperatura Globo" + "&value3=" + "Velocidade do Ar" + "&value4=" + "Umidade Relativa";
    firstPost = false;
    }
  else{
    url = String("https://script.google.com") + "/macros/s/" + GOOGLE_SCRIPT_ID + "/exec?" + "value1=" + String(Tar) + "&value2=" + String (Tglobo)+ "&value3=" + String(Veloc)+ "&value4=" + String(HR) ;
  }
  
  Serial.print("Making a request");
  HTTPClient http;
  http.begin(url.c_str()); //Specify the URL and certificate
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  int httpCode = http.GET();
  String payload;

  if (httpCode > 0) { //Check for the returning code

    payload = http.getString();
    Serial.println("Payload: "+payload);

  }
  else
  {
    Serial.println("Error on HTTP request");
  }
  http.end();

}