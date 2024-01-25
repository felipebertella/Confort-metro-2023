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
#include <RTOS.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <WiFi.h>
#include "secrets_test.h" //https://script.google.com/macros/s/AKfycbwNyYF7dFzu1bpxnLxsNjKznJCYEQS8ii-UEcvL0s-bosbYbK3AIMEhmlw4EULSwzqI/exec
#include <esp_wpa2.h>
#include <esp_wifi.h>
#include <HTTPClient.h>

#define timeoutUntillResetSeconds 60

//----------------------- GOOGLE PLANILHAS ---------------------//
String GOOGLE_SCRIPT_ID = googleScriptID;
String GOOGLE_SHEET_NAME = sheetName;
const int httpsPort = 443;
WiFiClientSecure client;
#define button 27

//---------------------------- FREERTOS -----------------//
TaskHandle_t Task1 = NULL; //Leitura dos ADCs
TaskHandle_t Task2 = NULL; //Gravação PEN-DRIVE
TaskHandle_t Task3 = NULL; //Atualização OLED
TaskHandle_t Task4 = NULL; //Wifi

#define num_sensor_Tar 39 //temperatura do ar
#define num_sensor_Tglobo 35 //temperatura do globo
#define num_sensor_Tquente 36 //temperatura sensor quente
#define num_sensor_HR 32 //umidade do ar

//--------------------- ADC ESP -------------------//
int read_raw;
esp_adc_cal_characteristics_t adc1_chars; 
esp_adc_cal_characteristics_t adc_cal;

//----------------------------- DISPLAY ---------------------------//

/* Uncomment the initialize the I2C address , uncomment only one, If you get a totally blank screen try the other*/
#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
#define LED_pen_drive 13
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//----------------------- Pen drive --------------------------------------------//
Ch376msc flashDrive(Serial2);
int reset_count = 0;
int conseguiu_abrir_arquivo, estado_pen_drive_anterior, estado_pen_drive_agora;
int cont_gravacao = 0;
String nome_arquivo;
String dataString = "";
byte USB_Byte;               //used to store data coming from the USB stick
int pino_LED_pendrive = 13;                //the LED is connected to digital pin 
int timeOut = 2000;          //TimeOut is 2 seconds. This is the amount of time you wish to wait for a response from the CH376S module.
#define Debug_Pen_Drive 1 // 1 printa infos pen drive,   0 não printa nada

//------------------- FLAGS -------------------------------------------//     
bool flagDatalog = true; //Flag para liberar o armazenamento de dados no pendrive
bool flagOLED = false; //Flag para liberar a atualização do Display
bool flagPost = false; //Flag para liberar o post de dados nas planilhas do google
bool firstsave = true; //Flag para indicar o primeiro salvamento no pendrive (controle de nome de arquivo)
bool failed_save = false; //Flag para controle de funcionamento do pendrive
bool firstPost = true; //Flag para indicar o primeiro salvamento na planilha
String logCheck = ""; //String para indicar funcionamento do pendrive

//-------------------- RTC ------------------------------//
RTC_DS3231 rtc;
String dia,mes,ano,hora,minuto,segundo;

//------------------------ VARIÁVEIS GERAIS --------------------//
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

//--------------------- DECLARAÇÃO DE FUNÇÕES ------------------//
void setup_display();
double Calibrar_tensao(double tensao, int canal);
double Leitura_Tensao_Canal(int canal, float n_amostras, int canal_calibracao);
void update_sums();
void getTime();
void configure_constants();
void PostData();
void blinkLED();
byte getResponseFromUSB();
boolean waitForResponse(String errorMsg);
void fileClose(byte closeCmd);
void filePointer(boolean fileBeginning);
void fileDelete(String fileName);
boolean fileCreate();
boolean continueRead();
void fileRead();
int getFileSize();
boolean setByteRead(byte numBytes);
void fileOpen();
void USBdiskMount();
bool diskConnectionStatus();
void setFileName(String fileName);
byte fileWrite(String data);
void appendFile(String fileName, String data);
void writeFile(String fileName, String data);
void readFile(String fileName);
void resetALL();
void set_USB_Mode(byte value);
void checkConnection(byte value);
void printCommandHeader(String header);
void Atualiza_Nome_Arquivo();
void oled();

//---------------------Interrupções de tempo-------------------//

void inicialize_timers()  //timer for interruptions - esp32u 40Mhz
{
  void IRAM_ATTR data_log();
  void IRAM_ATTR oled_update(); 
  void IRAM_ATTR post_data();

  hw_timer_t *data_log_timer = NULL;
  data_log_timer = timerBegin(0, 8000, true);
  timerAttachInterrupt(data_log_timer, &data_log, true);
  timerAlarmWrite(data_log_timer, 1200000, true); //change middle number to change the interruption timer 10000 = 1s
  timerAlarmEnable(data_log_timer);

  hw_timer_t *oled_update_timer = NULL;
  oled_update_timer = timerBegin(3,8000,true);
  timerAttachInterrupt(oled_update_timer, &oled_update, true);
  timerAlarmWrite(oled_update_timer, 20000, true);
  timerAlarmEnable(oled_update_timer);


  hw_timer_t *status_timer = NULL; //every 7 minutes
  status_timer = timerBegin(2, 8000, true);
  timerAttachInterrupt(status_timer, &post_data, true);
  timerAlarmWrite(status_timer, 600000, true); //change middle number to change the interruption timer 10000 = 1s
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

//-------------------------TAREFAS FREERTOS----------------------------//

void Task1code( void * pvParameters )
{
  while (1)
  {
    Serial.println("task1");
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
    vTaskDelay(10/ portTICK_PERIOD_MS);
  }
}

void Task2code( void * pvParameters )
{
  while (1)
  {
    // O QUE ESTÁ COMENTADO FUNCIONA COM MAIS MÓDULOS MAS AS VEZES PERDE CONEXÃO E É NECESSÁRIO REINICIAR
    /*if (flagDatalog){
      Serial.println("task2"); 
      digitalWrite(2,HIGH); 
      Tar = soma_Tar_gravacao/leituras_gravacao; 
      Tglobo = soma_Tglobo_gravacao/leituras_gravacao;
      Veloc = soma_Veloc_gravacao/leituras_gravacao; 
      HR = soma_HR_gravacao/leituras_gravacao;
      getTime();
      dataString = dia +"/"+mes+"/"+ano+","+hora+":"+minuto+ ":" + segundo+ "," +String(Tar) + "," + String(Tglobo) + "," + String(Veloc)+ "," + String(HR) +"\n"; 
     if (logCheck == "Failed"){
        firstsave = true;
        Serial2.end();
        vTaskDelay(20 / portTICK_PERIOD_MS);
        Serial2.begin(9600);
        Ch376msc flashDrive(Serial2);
        reset_count +=1;
      }

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
          }
        }

        flashDrive.setFileName(adat2);  //set the file name
        flashDrive.openFile();         //open the file
        flashDrive.writeFile(adat, strlen(adat)); //string, string length
        flashDrive.closeFile();//at the end, close the file
        logCheck = "Checking";
        firstsave = false; 
      }
      
      else{
        for (int i = 0; i < (dataString.length()+1); i++) {
        adat[i] = dataString[i];
        }
        flashDrive.setFileName(adat2);  //set the file name
          if(flashDrive.openFile() == ANSW_USB_INT_SUCCESS){               //open the file
            flashDrive.moveCursor(CURSOREND); 
            flashDrive.writeFile(adat, strlen(adat)); //if the file exist, move the "virtual" cursor at end of the file, with CURSORBEGIN we actually rewrite our old file
            logCheck = "Ok"; 
            reset_count = 0; 
          }
          else
            logCheck = "Failed";
        flashDrive.closeFile();
      }

      soma_Tar_gravacao = 0; soma_Tglobo_gravacao = 0; soma_Veloc_gravacao = 0; soma_HR_gravacao = 0; leituras_gravacao = 0;
      digitalWrite(2,LOW); 
      flagDatalog = false; 
      if (reset_count == 4)
        esp_restart();
      vTaskDelay(500/ portTICK_PERIOD_MS);
    }
  else{
      vTaskDelay(10/ portTICK_PERIOD_MS);
    }*/

    if (failed_save){
        resetALL();                     //Reset the module
        set_USB_Mode(0x06);             //Set to USB Mode
        //diskConnectionStatus();         //Check that communication with the USB device is possible
        USBdiskMount();
        if (diskConnectionStatus()){
            resetALL();                     //Reset the module
            set_USB_Mode(0x06);             //Set to USB Mode
            //diskConnectionStatus();         //Check that communication with the USB device is possible
            USBdiskMount();
            nome_arquivo = (dia+mes+hora+minuto+".CSV");
            writeFile(nome_arquivo, "Data, Hora, Temperatura Ar, Temperatura Globo, Veloc Ar \n");
            failed_save = false;}
        reset_count +=1;
        vTaskDelay(1000);
        if (reset_count == 20) 
          esp_restart();
    }
    if (flagDatalog){
      digitalWrite(2, HIGH); //LED PARA CONTROLE
      Tar = soma_Tar_gravacao/leituras_gravacao; 
      Tglobo = soma_Tglobo_gravacao/leituras_gravacao;
      Veloc = soma_Veloc_gravacao/leituras_gravacao; 
      HR = soma_HR_gravacao/leituras_gravacao;
      getTime();

      if (firstsave){
        nome_arquivo = (dia+mes+hora+minuto+".CSV"); 
        Serial.print(nome_arquivo);
        //resetALL();                     //Reset the module

        set_USB_Mode(0x06);             //Set to USB Mode
        diskConnectionStatus();         //Check that communication with the USB device is possible
        USBdiskMount();              //Prepare the USB for reading/writing - you need to mount the USB disk for proper read/write operations.

        writeFile(nome_arquivo, "Data, Hora, Temperatura Ar, Temperatura Globo, Veloc Ar, Umidade Relativa \n");  
        firstsave = false;
        reset_count = 0;
        
      }

      if (diskConnectionStatus()){

        dataString = dia +"/"+mes+"/"+ano+","+hora+":"+minuto+ ":" + segundo+ "," +String(Tar) + "," + String(Tglobo) + "," + String(Veloc) + "," + String(HR) + "\n"; 

        appendFile(nome_arquivo, dataString);
        
        soma_Tar_gravacao = 0;
        soma_Tglobo_gravacao = 0;
        soma_Veloc_gravacao = 0;
        soma_HR_gravacao = 0;
        leituras_gravacao = 0;
        flagDatalog = false; 
        logCheck = "Ok";
        }
      else{
        failed_save = true; 
        logCheck = "Failed";}

      if (failed_save){
        resetALL();                     //Reset the module
        set_USB_Mode(0x06);             //Set to USB Mode
        //diskConnectionStatus();         //Check that communication with the USB device is possible
        USBdiskMount();
        if (diskConnectionStatus()){
            resetALL();                     //Reset the module
            set_USB_Mode(0x06);             //Set to USB Mode
            //diskConnectionStatus();         //Check that communication with the USB device is possible
            USBdiskMount();
            nome_arquivo = (dia+mes+hora+minuto+".CSV");
            writeFile(nome_arquivo, "Data, Hora, Temperatura Ar, Temperatura Globo, Veloc Ar \n");
            failed_save = false;}
      }
      flagDatalog = false;
      digitalWrite(2, LOW);
      vTaskDelay(500/ portTICK_PERIOD_MS); }

    else{
        vTaskDelay(10/ portTICK_PERIOD_MS);
    }
  }
}

void Task3code (void * pvParameters)
{
  while (1)
  {
    if (flagOLED){
      display.setCursor(0,0);
      Serial.println("task3"); 
      display.clearDisplay();
      //display.display();
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
      vTaskDelay(500/ portTICK_PERIOD_MS);
    }
    else{
      //Serial.println("task3");
      vTaskDelay(10/ portTICK_PERIOD_MS);
    }
  }
}

void Task4code (void * pvParameters)
{
  while (1)
  {
    if (flagPost){
      if (WiFi.status() == WL_CONNECTED)
      {
        Serial.print("Connected");
        PostData();
        
      }
      else
      {
        Serial.print("Disconnected");
        //WiFi.disconnect();
        //WiFi.reconnect();
      }
      flagPost = false;
      vTaskDelay(200/ portTICK_PERIOD_MS);
    }
    else{
      if (digitalRead(button) == HIGH) {
        WiFiManager wm;
        Serial.println("Resetando...");
        wm.resetSettings(); //apaga as credenciais salvas
        ESP.restart(); //reboota o código
      }
      vTaskDelay(50/ portTICK_PERIOD_MS);}
    }
}

void SetupTasks()
{
  xTaskCreatePinnedToCore
  (
                    Task1code,                /* Task function. */
                    "Task1",                  /* name of task. */
                    10000,                    /* Stack size of task */
                    NULL,                     /* parameter of the task */
                    0,                        /* priority of the task */
                    &Task1,                   /* Task handle to keep track of created task */
                    tskNO_AFFINITY);          /* lets the RTOS decide the core*/   

  xTaskCreatePinnedToCore
  (
                    Task2code,                /* Task function. */
                    "Task2",                  /* name of task. */
                    10000,                    /* Stack size of task */
                    NULL,                     /* parameter of the task */
                    0,                        /* priority of the task */
                    &Task2,                   /* Task handle to keep track of created task */
                    tskNO_AFFINITY);         /* lets the RTOS decide the core*/ 

  xTaskCreatePinnedToCore
  (
                    Task3code,                /* Task function. */
                    "Task3",                  /* name of task. */
                    10000,                    /* Stack size of task */
                    NULL,                     /* parameter of the task */
                    0,                        /* priority of the task */
                    &Task3,                   /* Task handle to keep track of created task */
                    tskNO_AFFINITY);          /* lets the RTOS decide the core*/ 
  
  xTaskCreatePinnedToCore
  (
                    Task4code,                /* Task function. */
                    "Task4",                  /* name of task. */
                    10000,                    /* Stack size of task */
                    NULL,                     /* parameter of the task */
                    0,                        /* priority of the task */
                    &Task4,                   /* Task handle to keep track of created task */
                    tskNO_AFFINITY);          /* lets the RTOS decide the core*/ 
}

//-------------------------- WIFI --------------------//
bool setupWifi(){
    WiFi.mode(WIFI_STA); //inicia o esp32 no modo estação
    WiFiManager wm; //chama o arquivo principal C++
    bool res;
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

//---------------------SETUP----------------------------//
void setup() {
  Serial.begin(9600); //Serial para monitor serial
  Serial2.begin(9600); //Serial para pendrive
  flashDrive.init(); //inicia o pendrive

  esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_cal);//Inicializa a estrutura de calibracao
  int read = analogRead(32); // necessário chamar pelo menos uma vez, não sei porque mas só funciona assim
   
  //pinMode(32, INPUT_PULLDOWN); 
  if(!display.begin(0x3C,true)) { // Address 0x3D for 128x64 INICIALIZAÇÃO DO DISPLAY
    Serial.println(F("SSD1306 allocation failed"));
  }
  setup_display();
  delay(40);
  pinMode(2, OUTPUT);
  pinMode(button, INPUT_PULLDOWN); // Botão para resetar as configurações de WIFI do WIFIMANAGER
  pinMode(LED_pen_drive, OUTPUT);  // Define digital pin 13 as an OUTPUT pin - so that we can use it with an LED
  digitalWrite(LED_pen_drive, LOW);  
  Wire.begin();//Inicia a biblioteca Wire responsável pelo SPI para o RTC
  rtc.begin(); //Inicia o RTC
  //rtc.adjust(DateTime(2023, 10, 30, 12, 00, 00)); //AJUSTE DE DATA (ano, mes, dia, hora, minuto, segundo)
  inicialize_timers();
  delay(15);
  configure_constants(); //configura as constantes de calibração feitas pelo Prof. Saulo
  setupWifi();
  delay(15);
  SetupTasks();
}

//-----------------INICIALIZAÇÃO DO DISPLAY------------------//
void setup_display(){
  display.clearDisplay();
  display.display(); 
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
  
  display.setCursor(0,0);
  display.clearDisplay();
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

//-----------------LEITURA DOS ADCs---------------------------//
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

    break;
  case 36:
    for (i = 0; i < n_amostras; i++) 
  {    media_leituras += adc1_get_raw(ADC1_CHANNEL_0);  
  }
    media_leituras /= 100;
    media_leituras = esp_adc_cal_raw_to_voltage(media_leituras, &adc_cal);
    voltage = (media_leituras/1000);
    media_leituras_calibrado = Calibrar_tensao(voltage, canal_calibracao);
    break;
  case 39:
     for (i = 0; i < n_amostras; i++) 
  {    media_leituras += adc1_get_raw(ADC1_CHANNEL_3);  
  }
    media_leituras /= 100;
    media_leituras = esp_adc_cal_raw_to_voltage(media_leituras, &adc_cal);
    voltage = (media_leituras/1000);
    media_leituras_calibrado = Calibrar_tensao(voltage, canal_calibracao);
  default:
    break;
  }
  
  return media_leituras_calibrado;
}

double Calibrar_tensao(double tensao, int canal) 
{
  double Valor_calibrado;

  //-----------------UTILIZANDO A CONVERSÃO PELA FORMULA---------------------//
  const double beta = 3964.0;
  const double r0 = 15000.0;
  const double t0 = 273.0+ 25.0;
  const double rx = r0 * exp(-beta/t0);

  double rt = (r0*3.3-tensao*r0)/tensao;
  
  Valor_calibrado = (beta/(log(rt/rx))) - 273.15;

  //----------------UTILIZANDO A CALIBRAÇÃO DO PROF. SAULO-------------------//
  //Valor_calibrado = (1/(0.00091+(0.00024*log(rt))))-273.15;
  /*Valor_calibrado =   a[canal]*pow(tensao, 5) +
                       b[canal]*pow(tensao, 4) +
                       c[canal]*pow(tensao, 3) +
                       d[canal]*pow(tensao, 2) +
                       e[canal]*pow(tensao, 1) +
                       f[canal]*pow(tensao, 0);*/
                    
  return Valor_calibrado; 
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

//----------------------LEITURA DO RTC----------------------//
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

//----------------------POSTAR DADOS NA PLANILHA DO SHEETS-----------------//
void PostData()
{
  String url = ""; 
  if (firstPost){
    url = String("https://script.google.com") + "/macros/s/" + GOOGLE_SCRIPT_ID + "/exec?" + "value1=" + "Temperatura Ar" + 
                "&value2=" + "Temperatura Globo" + "&value3=" + "Velocidade do Ar" + "&value4=" + "Umidade Relativa";
    firstPost = false;
    }
  else{
    url = String("https://script.google.com") + "/macros/s/" + GOOGLE_SCRIPT_ID + "/exec?" + "value1=" + String(Tar) + "&value2=" + String (Tglobo)+ "&value3=" + String(Veloc)+ "&value4=" + String(HR) + "&value5=" + logCheck ;
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

//------------------FUNÇÕES DO PENDRIVE PROF. SAULO -----------------------//
void Atualiza_Nome_Arquivo()
{      
       nome_arquivo = dia + "_" + hora + "-" + minuto + ".CSV";     
}
//print Command header
void printCommandHeader(String header) {
   // Serial.println("======================");
   // Serial.println("");
   // Serial.println(header);
   // Serial.println("----------------------");
}

//checkConnection==================================================================================
//This function is used to check for successful communication with the CH376S module. This is not dependant of the presence of a USB stick.
//Send any value between 0 to 255, and the CH376S module will return a number = 255 - value. 
void checkConnection(byte value) {

    Serial2.write(0x57);
    Serial2.write(0xAB);
    Serial2.write(0x06);
    Serial2.write(value);


    if (waitForResponse("checking connection")) {       //wait for a response from the CH376S. If CH376S responds, it will be true. If it times out, it will be false.
        if (getResponseFromUSB() == (255 - value)) {
            if (Debug_Pen_Drive == 1)   Serial.println(">Connection to CH376S was successful.");
            blinkLED();                               //blink the LED for 1 second if the connection was successful
        }
        else 
        {
            if (Debug_Pen_Drive == 1)     Serial.print(">Connection to CH376S - FAILED.");
        }
    }
}

//set_USB_Mode=====================================================================================
//Make sure that the USB is inserted when using 0x06 as the value in this specific code sequence
void set_USB_Mode(byte value) 
{
    Serial2.write(0x57);
    Serial2.write(0xAB);
    Serial2.write(0x15);
    Serial2.write(value);
    delay(20);
    if (Serial2.available()) {
        USB_Byte = Serial2.read();
        //Check to see if the command has been successfully transmitted and acknowledged.
        if (USB_Byte == 0x51) {                                   // If true - the CH376S has acknowledged the command.
            //Serial.println("set_USB_Mode command acknowledged"); //The CH376S will now check and monitor the USB port
            USB_Byte = Serial2.read();

            //Check to see if the USB stick is connected or not.
            if (USB_Byte == 0x15) 
            {                               // If true - there is a USB stick connected
                if (Debug_Pen_Drive == 1)   Serial.println("USB is present");
                blinkLED();                                     // If the process was successful, then turn the LED on for 1 second 
            }
            else 
            {
                 if (Debug_Pen_Drive == 1)   Serial.print("USB Not present. Error code:");   // If the USB is not connected - it should return an Error code = FFH
                  //Serial.print(USB_Byte, HEX);   Serial.println("H");
            }

        }
        else 
        {
            if (Debug_Pen_Drive == 1)  Serial.print("CH3765 error!   Error code:");
            //Serial.print(USB_Byte, HEX);     Serial.println("H");
        }
    }
    delay(20);
  
}

//resetALL=========================================================================================
//This will perform a hardware reset of the CH376S module - which usually takes about 35 msecs =====
void resetALL() 
{
    Serial2.write(0x57);
    Serial2.write(0xAB);
    Serial2.write(0x05);
 
    //Serial.println("The CH376S module has been reset !");
    delay(200);
}

//readFile=====================================================================================
//This will send a series of commands to read data from a specific file (defined by fileName)
void readFile(String fileName) {
    resetALL();                     //Reset the module
    set_USB_Mode(0x06);             //Set to USB Mode
    diskConnectionStatus();         //Check that communication with the USB device is possible
    USBdiskMount();                 //Prepare the USB for reading/writing - you need to mount the USB disk for proper read/write operations.
    setFileName(fileName);          //Set File name
    fileOpen();                     //Open the file for reading
    //int fs = getFileSize();         //Get the size of the file
    fileRead();                     //***** Send the command to read the file ***
    fileClose(0x00);                //Close the file
}

//writeFile========================================================================================
//is used to create a new file and then write data to that file. "fileName" is a variable used to hold the name of the file (e.g TEST.TXT). "data" should not be greater than 255 bytes long. 
void writeFile(String fileName, String data) {
    resetALL();                     //Reset the module
    set_USB_Mode(0x06);             //Set to USB Mode
    diskConnectionStatus();         //Check that communication with the USB device is possible
    USBdiskMount();                 //Prepare the USB for reading/writing - you need to mount the USB disk for proper read/write operations.
    setFileName(fileName);          //Set File name
    if (fileCreate()) {               //Try to create a new file. If file creation is successful
        fileWrite(data);              //write data to the file.
    }
    else 
    {
        if (Debug_Pen_Drive == 1)  Serial.println("File could not be created, or it already exists");
    }
    fileClose(0x01);
}

//appendFile()================== reparado   ==================================================================
//is used to write data to the end of the file, without erasing the contents of the file.
void appendFile(String fileName, String data){
    resetALL();                     //Reset the module
    set_USB_Mode(0x06);             //Set to USB Mode
    diskConnectionStatus();         //Check that communication with the USB device is possible
    USBdiskMount();                 //Prepare the USB for reading/writing - you need to mount the USB disk for proper read/write operations.
    setFileName(fileName);          //Set File name
    fileOpen();                     //Open the file
    filePointer(false);             //filePointer(false) is to set the pointer at the end of the file.  filePointer(true) will set the pointer to the beginning.
    byte numWr = fileWrite(data);   //Write data to the end of the file and return num bytes written
    fileClose(0x01);                //Close the file using 0x01 - which means to update the size of the file on close. 
    if (numWr < data.length()){//not all bytes were written, some 512 byte barrier broken...???
      //resend the lost data...
      delay(100);
      diskConnectionStatus();         //Check that communication with the USB device is possible
      //USBdiskMount();                 //Prepare the USB for reading/writing - you need to mount the USB disk for proper read/write operations.
      setFileName(fileName);          //Set File name
      fileOpen();                     //Open the file
      filePointer(false);             //filePointer(false) is to set the pointer at the end of the file.  filePointer(true) will set the pointer to the beginning.
      data = data.substring(numWr);    //remove the written data and write the truncated portion
      fileWrite(data);
      fileClose(0x01);
    }
   
}

//fileWrite========================  reparado ===============================================================
//are the commands used to write to the file
byte fileWrite(String data){
  byte numByteWr = 0;
 if (Debug_Pen_Drive == 1)  Serial.println("Writing to file:");
  byte dataLength = (byte) data.length();         // This variable holds the length of the data to be written (in bytes)
  //Serial.println(data);
  //Serial.print("Data Length:");
  //Serial.println(dataLength);
  delay(100);

 
  // This set of commands tells the CH376S module how many bytes to expect from the Arduino.  (defined by the "dataLength" variable)
  Serial2.write(0x57);
  Serial2.write(0xAB);
  Serial2.write(0x3C);
  Serial2.write((byte) dataLength);
  Serial2.write((byte) 0x00);
  if(waitForResponse("setting data Length"))
  {      // Wait for an acknowledgement from the CH376S module before trying to send data to it
    if(getResponseFromUSB()==0x1E)
    {                // 0x1E indicates that the USB device is in write mode.
      Serial2.write(0x57);
      Serial2.write(0xAB);
      Serial2.write(0x2D);
      Serial2.print(data);                             // write the data to the file
  
      if(waitForResponse("writing data to file"))   // wait for an acknowledgement from the CH376S module   // estranho, não tem nada dentro 
      numByteWr = Serial2.read();
      //Serial.print("Write code.. (number bytes written) and 14 : ");
      //Serial.print(numByteWr);                // code is number of bytes written to disk
      //Serial.print(",");
      Serial2.write(0x57);
      Serial2.write(0xAB);
      Serial2.write(0x3D);                             // This is used to update the file size. Not sure if this is necessary for successful writing.
      if(waitForResponse("updating file size"))   // wait for an acknowledgement from the CH376S module // estranho, não tem nada dentro     
      Serial2.read(); // mudado aqui: original era abaixo, mas se comentado gravava varios arq em sequencia. Então separado dessa forma
      //Serial.println(Serial2.read(),HEX);              //code is normally 0x14 // Deu prob se comentado... gravou varios arquivos sequenciais
    }
  }
  
  return(numByteWr);
}

//setFileName======================================================================================
//This sets the name of the file to work with
void setFileName(String fileName) {
 if (Debug_Pen_Drive == 1)  {  Serial.print("Setting filename to:");    Serial.println(fileName);}

    Serial2.write(0x57);
    Serial2.write(0xAB);
    Serial2.write(0x2F);
    Serial2.write(0x2F);         // Every filename must have this byte to indicate the start of the file name.
    Serial2.print(fileName);     // "fileName" is a variable that holds the name of the file.  eg. TEST.TXT
    Serial2.write((byte)0x00);   // you need to cast as a byte - otherwise it will not compile.  The null byte indicates the end of the file name.
    delay(20);
  

}

//diskConnectionStatus================================================================================
//Check the disk connection status
bool diskConnectionStatus() {
  // Serial.println("Checking USB disk connection status");

    Serial2.write(0x57);
    Serial2.write(0xAB);
    Serial2.write(0x30);

    if (waitForResponse("Connecting to USB disk")) {       //wait for a response from the CH376S. If CH376S responds, it will be true. If it times out, it will be false.
        if (getResponseFromUSB() == 0x14) 
        {               //CH376S will send 0x14 if this command was successful
             if (Debug_Pen_Drive == 1)  {Serial.println(">Connection to USB OK");
             return true;}
        }
        else {
             if (Debug_Pen_Drive == 1)   {Serial.print(">Connection to USB - FAILED.");
             return false;}
        }
    }
}

//USBdiskMount========================================================================================
//initialise the USB disk and check that it is ready - this process is required if you want to find the manufacturing information of the USB disk
void USBdiskMount() {
    //Serial.println("Mounting USB disk");

    Serial2.write(0x57);
    Serial2.write(0xAB);
    Serial2.write(0x31);

    if (waitForResponse("mounting USB disk")) {       //wait for a response from the CH376S. If CH376S responds, it will be true. If it times out, it will be false.
        if (getResponseFromUSB() == 0x14) {               //CH376S will send 0x14 if this command was successful
            if (Debug_Pen_Drive == 1)  Serial.println(">USB Mounted - OK");
            estado_pen_drive_agora = 1;            

        }
        else {
            if (Debug_Pen_Drive == 1)     Serial.print(">Failed to Mount USB disk.");
            estado_pen_drive_agora = 0; 
            estado_pen_drive_anterior = 0;

        }
    }
}

//fileOpen========================================================================================
//opens the file for reading or writing
void fileOpen() {
   // Serial.println("Opening file.");

    Serial2.write(0x57);
    Serial2.write(0xAB);
    Serial2.write(0x32);
    
    if (waitForResponse("file Open")) {                 //wait for a response from the CH376S. If CH376S responds, it will be true. If it times out, it will be false.
        if (getResponseFromUSB() == 0x14) 
        {                 //CH376S will send 0x14 if this command was successful  
            if (Debug_Pen_Drive == 1)        Serial.println(">File opened successfully.");
            conseguiu_abrir_arquivo = 1;
        }
        else {
            if (Debug_Pen_Drive == 1)        Serial.print(">Failed to open file.");
            conseguiu_abrir_arquivo = 0;
        }
    }
}
//setByteRead=====================================================================================
//This function is required if you want to read data from the file. 
boolean setByteRead(byte numBytes) {
    boolean bytesToRead = false;
    //int timeCounter = 0;

    Serial2.write(0x57);
    Serial2.write(0xAB);
    Serial2.write(0x3A);
    Serial2.write((byte)numBytes);   //tells the CH376S how many bytes to read at a time
    Serial2.write((byte)0x00);
   
    if (waitForResponse("setByteRead")) {       //wait for a response from the CH376S. If CH376S responds, it will be true. If it times out, it will be false.
        if (getResponseFromUSB() == 0x1D) {         //read the CH376S message. If equal to 0x1D, data is present, so return true. Will return 0x14 if no data is present.
            bytesToRead = true;
        }
    }
    return(bytesToRead);
}

//getFileSize()===================================================================================
//writes the file size to the serial Monitor.
int getFileSize() {
    int fileSize = 0;
    //Serial.println("Getting File Size");

    Serial2.write(0x57);
    Serial2.write(0xAB);
    Serial2.write(0x0C);
    Serial2.write(0x68);
    delay(100);
    //Serial.print("FileSize =");
    if (Serial2.available()) {
        fileSize = fileSize + Serial2.read();
    }
    if (Serial2.available()) {
        fileSize = fileSize + (Serial2.read() * 255);
    }
    if (Serial2.available()) {
        fileSize = fileSize + (Serial2.read() * 255 * 255);
    }
    if (Serial2.available()) {
        fileSize = fileSize + (Serial2.read() * 255 * 255 * 255);
    }
  
    //Serial.println(fileSize);
    delay(10);
    return(fileSize);
}


//fileRead========================================================================================
//read the contents of the file
void fileRead() {
    if (Debug_Pen_Drive == 1)  Serial.println("Reading file:");
    byte firstByte = 0x00;                     //Variable to hold the firstByte from every transmission.  Can be used as a checkSum if required.
    byte numBytes = 0x40;                      //The maximum value is 0x40  =  64 bytes

    while (setByteRead(numBytes)) {              //This tells the CH376S module how many bytes to read on the next reading step. In this example, we will read 0x10 bytes at a time. Returns true if there are bytes to read, false if there are no more bytes to read.
        Serial2.write(0x57);
        Serial2.write(0xAB);
        Serial2.write(0x27);                          //Command to read ALL of the bytes (allocated by setByteRead(x))
        if (waitForResponse("reading data")) {      //Wait for the CH376S module to return data. TimeOut will return false. If data is being transmitted, it will return true.
            firstByte = Serial2.read();                 //Read the first byte
            while (Serial2.available()) 
            {
                //Serial.write(Serial2.read());           //Send the data from the USB disk to the Serial monitor // ok, pode comentar
                delay(1);                           //This delay is necessary for successful Serial transmission
            }
        }
        if (!continueRead()) {                       //prepares the module for further reading. If false, stop reading.
            break;                                   //You need the continueRead() method if the data to be read from the USB device is greater than numBytes.
        }
    }
    
    //Serial.println();
    //Serial.println("NO MORE DATA");
}

//continueRead()==================================================================================
//continue to read the file : I could not get this function to work as intended.
boolean continueRead() {
    boolean readAgain = false;

    Serial2.write(0x57);
    Serial2.write(0xAB);
    Serial2.write(0x3B);
   
    if (waitForResponse("continueRead")) {       //wait for a response from the CH376S. If CH376S responds, it will be true. If it times out, it will be false.
        if (getResponseFromUSB() == 0x14) {         //CH376S will send 0x14 if this command was successful
            readAgain = true;
        }
    }
    return(readAgain);
}

//fileCreate()========================================================================================
//the command sequence to create a file
boolean fileCreate() {
    boolean createdFile = false;
    Serial2.write(0x57);
    Serial2.write(0xAB);
    Serial2.write(0x34);
    
    if (waitForResponse("creating file")) {       //wait for a response from the CH376S. If file has been created successfully, it will return true.
        if (getResponseFromUSB() == 0x14) {          //CH376S will send 0x14 if this command was successful
            createdFile = true;
        }
    }
    return(createdFile);
}

//fileDelete()========================================================================================
//the command sequence to delete a file
void fileDelete(String fileName) {
    setFileName(fileName);
    delay(20);

    Serial2.write(0x57);
    Serial2.write(0xAB);
    Serial2.write(0x35);
    
    if (waitForResponse("deleting file")) {       //wait for a response from the CH376S. If file has been created successfully, it will return true.
        if (getResponseFromUSB() == 0x14) {          //CH376S will send 0x14 if this command was successful
            if (Debug_Pen_Drive == 1)      Serial.println("Successfully deleted file");
        }
    }
}


//filePointer========================================================================================
//is used to set the file pointer position. true for beginning of file, false for the end of the file.
void filePointer(boolean fileBeginning) 
{
 
    Serial2.write(0x57);
    Serial2.write(0xAB);
    Serial2.write(0x39);
    if (fileBeginning) {
        Serial2.write((byte)0x00);             //beginning of file
        Serial2.write((byte)0x00);
        Serial2.write((byte)0x00);
        Serial2.write((byte)0x00);
    }
    else {
        Serial2.write((byte)0xFF);             //end of file
        Serial2.write((byte)0xFF);
        Serial2.write((byte)0xFF);
        Serial2.write((byte)0xFF);
    }

    
    if (waitForResponse("setting file pointer")) {       //wait for a response from the CH376S. 
        if (getResponseFromUSB() == 0x14) {                 //CH376S will send 0x14 if this command was successful
             if (Debug_Pen_Drive == 1)           Serial.println("Pointer successfully applied");
        }
    }
}

//fileClose=======================================================================================
//closes the file
void fileClose(byte closeCmd) {
    //Serial.println("Closing file:");

    Serial2.write(0x57);
    Serial2.write(0xAB);
    Serial2.write(0x36);
    Serial2.write((byte)closeCmd);                                // closeCmd = 0x00 = close without updating file Size, 0x01 = close and update file Size

 
    if (waitForResponse("closing file")) {                      // wait for a response from the CH376S. 
        byte resp = getResponseFromUSB();
        if (resp == 0x14) 
        {                                        // CH376S will send 0x14 if this command was successful
            if (Debug_Pen_Drive == 1)            Serial.println(">File closed successfully.");
        }
        else 
        {
            if (Debug_Pen_Drive == 1)            Serial.print(">Failed to close file. Error code:");
            //Serial.println(resp, HEX);
        }
    }
}


//waitForResponse===================================================================================
//is used to wait for a response from USB. Returns true when bytes become available, false if it times out.
boolean waitForResponse(String errorMsg) {
    boolean bytesAvailable = true;
    int counter = 0;

    while (!Serial2.available()) 
    {     //wait for CH376S to verify command
        delay(1);
        counter++;
        if (counter > timeOut) {
            if (Debug_Pen_Drive == 1)            Serial.print("TimeOut waiting for response: Error while: ");
            //Serial.println(errorMsg);
            bytesAvailable = false;
            break;
        }
    }

    delay(1);
    return(bytesAvailable);
}

//getResponseFromUSB================================================================================
//is used to get any error codes or messages from the CH376S module (in response to certain commands)
byte getResponseFromUSB() 
{
    byte response = byte(0x00);

    if (Serial2.available()) {
        response = Serial2.read();
    }
    return(response);

}

//blinkLED==========================================================================================
//Turn an LED on for 1 second
void blinkLED() {
    //digitalWrite(pino_LED_pendrive, HIGH);
    //delay(500);
    //digitalWrite(pino_LED_pendrive, LOW);
}

void loop() {
  vTaskDelay(100/ portTICK_PERIOD_MS);
} 

