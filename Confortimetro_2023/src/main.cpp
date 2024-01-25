#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <RTClib.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SH110X.h>
#include <Ch376msc.h>

Ch376msc flashDrive(Serial2);

#define Debug_Pen_Drive 1 // 1 printa infos pen drive,   0 não printa nada
String Print_Pen_Drive_Short = "nao";
#define LED_pen_drive 13

/* Uncomment the initialize the I2C address , uncomment only one, If you get a totally blank screen try the other*/
#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//----------------------- Pen drive --------------------------------------------
int conseguiu_abrir_arquivo, estado_pen_drive_anterior, estado_pen_drive_agora;
double tempo_anterior_teste_pendrive;
int cont_gravacao = 0;
String nome_arquivo;
String dataString = "";
byte computerByte;           //used to store data coming from the computer
byte USB_Byte;               //used to store data coming from the USB stick
int pino_LED_pendrive = 13;                //the LED is connected to digital pin 
int timeOut = 2000;          //TimeOut is 2 seconds. This is the amount of time you wish to wait for a response from the CH376S module.
    
//String Dado2; 
bool flagDatalog = false; 
bool flagOLED = false; 
bool firstsave = true; 
bool failed_save = false; 

String dia,mes,ano,hora,minuto,segundo;

RTC_DS3231 rtc;
#define num_sensor_Tar 39
#define num_sensor_Tglobo 35
#define num_sensor_UR 34
#define num_sensor_Tquente 39


int num_leituras_media = 100;
float a[10], b[10], c[10], d[10], e[10], f[10];
float av, bv, cv, dv, ev, min_delta, max_delta;
float V1,V2,V3,V4,V5,V6,V7,V8;
float Tar, Tglobo, Veloc, UR, Tquente, Delta_T, DT, UV1, UV2;
//float Tar_filtrado = 0, Tglobo_filtrado = 0, Tquente_filtrado = 0, UR_filtrado = 0;
float soma_Tar_OLED = 0, soma_Tglobo_OLED = 0,soma_Veloc_OLED = 0,soma_UR_OLED = 0, soma_UV1_OLED = 0 ,soma_UV2_OLED = 0;
float soma_Tar_gravacao = 0, soma_Tglobo_gravacao = 0,soma_Veloc_gravacao = 0,soma_UR_gravacao = 0, soma_UV1_gravacao = 0, soma_UV2_gravacao = 0;
int leituras_gravacao = 0, leituras_OLED=0;

//Declaração de funções que serão definidas depois
double Calibrar_tensao(double tensao, int canal);
double Leitura_Tensao_Canal(int canal, float n_amostras, int canal_calibracao);
void update_sums();
void datalog();
void getTime();
void configure_constants();
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

//Function to inicialize timers and configure interruptions
void inicialize_timers()  //timer for interruptions - esp32u 40Mhz
{
  void IRAM_ATTR data_log();
  void IRAM_ATTR oled_update(); 
  void IRAM_ATTR check_Status();

  hw_timer_t *data_log_timer = NULL;
  data_log_timer = timerBegin(0, 8000, true);
  timerAttachInterrupt(data_log_timer, &data_log, true);
  timerAlarmWrite(data_log_timer, 150000, true); //change middle number to change the interruption timer 10000 = 1s
  timerAlarmEnable(data_log_timer);

  hw_timer_t *oled_update_timer = NULL;
  oled_update_timer = timerBegin(1,8000,true);
  timerAttachInterrupt(oled_update_timer, &oled_update, true);
  timerAlarmWrite(oled_update_timer, 30000, true);
  timerAlarmEnable(oled_update_timer);

  hw_timer_t *status_timer = NULL; //every 7 minutes
  status_timer = timerBegin(2, 8000, true);
  timerAttachInterrupt(status_timer, &check_Status, true);
  timerAlarmWrite(status_timer, 4200000, true); //change middle number to change the interruption timer 10000 = 1s
  timerAlarmEnable(status_timer);
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  flashDrive.init();

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

  pinMode(LED_pen_drive, OUTPUT);                                // Define digital pin 13 as an OUTPUT pin - so that we can use it with an LED
  digitalWrite(LED_pen_drive, LOW);                              // Turn off the LED
  Wire.begin();
  rtc.begin();
  //rtc.adjust(DateTime(2023, 10, 30, 12, 00, 00));
  inicialize_timers();
  configure_constants();

  estado_pen_drive_anterior = 0; // desconectado
  estado_pen_drive_agora = 0;
  
}

void loop() {
  //leitura das entradas analógicas
  leituras_gravacao += 1;
  leituras_OLED +=1; 
  Tar = Leitura_Tensao_Canal(num_sensor_Tar, num_leituras_media, 1); 
  Tglobo = Leitura_Tensao_Canal(num_sensor_Tglobo, num_leituras_media, 2);
  Tquente = Leitura_Tensao_Canal(num_sensor_Tquente, num_leituras_media, 4); 
  
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

//Interrupções
void IRAM_ATTR data_log(){
  flagDatalog = true; 
}
void IRAM_ATTR oled_update(){
  flagOLED = true;  
}
void IRAM_ATTR check_Status(){
  
}

//Atualiza o display
void oled(){
  display.clearDisplay();
  Tar = soma_Tar_OLED/leituras_OLED; 
  Tglobo = soma_Tglobo_OLED/leituras_OLED;
  Veloc = soma_Veloc_OLED/leituras_OLED; 

  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("Temp Ar: " + String(Tar)+"C");
  display.println("Temp Globo: " + String(Tglobo)+"C");
  display.println("Veloc Ar: " + String(Veloc)+"m/s");
  display.println("");
  display.println("");
  display.println(hora + ":" + minuto + " "+ dia + "/" + mes + "/" + ano);
  display.display();
  
  soma_Tar_OLED = 0;
  soma_Tglobo_OLED = 0;
  soma_Veloc_OLED = 0;
  leituras_OLED = 0; 
  flagOLED = false; 
}

//Armazena os dados
void datalog(){
  Tar = soma_Tar_gravacao/leituras_gravacao; 
  Tglobo = soma_Tglobo_gravacao/leituras_gravacao;
  Veloc = soma_Veloc_gravacao/leituras_gravacao; 

  getTime();

  if (firstsave){
    nome_arquivo = (dia+mes+hora+minuto+".CSV"); 
    Serial.print(nome_arquivo);
    //resetALL();                     //Reset the module

    set_USB_Mode(0x06);             //Set to USB Mode
    diskConnectionStatus();         //Check that communication with the USB device is possible
    USBdiskMount();              //Prepare the USB for reading/writing - you need to mount the USB disk for proper read/write operations.

    writeFile(nome_arquivo, "Data, Hora, Temperatura Ar, Temperatura Globo, Veloc Ar \n");  
    firstsave = false; 
  }

  if (diskConnectionStatus()){

  dataString = dia +"/"+mes+"/"+ano+","+hora+":"+minuto+ ":" + segundo+ "," +String(Tar) + "," + String(Tglobo) + "," + String(Veloc) + "\n"; 

  appendFile(nome_arquivo, dataString);
  
  soma_Tar_gravacao = 0;
  soma_Tglobo_gravacao = 0;
  soma_Veloc_gravacao = 0;
  leituras_gravacao = 0;
  flagDatalog = false; 
}
  else
    failed_save = true; 

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
  const double r0 = 30000.0;
  const double t0 = 273.0+ 25.0;
  const double rx = r0 * exp(-beta/t0);

  double rt = (r0*3.3-tensao*r0)/tensao;
  
  Valor_calibrado = (beta/(log(rt/rx))) - 273.15;
  //Valor_calibrado = (t-273.0);

  /*Valor_calibrado = (1/(0.00091+(0.00024*log(rt))))-273.15;
  Valor_calibrado =   a[canal]*pow(tensao, 5) +
                       b[canal]*pow(tensao, 4) +
                       c[canal]*pow(tensao, 3) +
                       d[canal]*pow(tensao, 2) +
                       e[canal]*pow(tensao, 1) +
                       f[canal]*pow(tensao, 0);*/
                    
  return Valor_calibrado; 
}

double Leitura_Tensao_Canal(int canal, float n_amostras, int canal_calibracao) 
{
  double media_leituras_calibrado = 0;
  int i = 0;
  float media_leituras = 0;
  for (i = 0; i < n_amostras; i++) 
  {    media_leituras += analogRead(canal);
  }
  media_leituras = media_leituras / n_amostras * 3.3 / 4096.0;
  media_leituras_calibrado = Calibrar_tensao(media_leituras, canal_calibracao);
  return media_leituras_calibrado;
  //if (n >= 28) n = 23;
  //return n;
}

void update_sums()
{
  soma_Tar_OLED += Tar;
  soma_Tglobo_OLED += Tglobo;
  soma_Veloc_OLED += Veloc;
  soma_Tar_gravacao += Tar; 
  soma_Tglobo_gravacao += Tglobo;
  soma_Veloc_gravacao += Veloc; 
}

// ========================================================================================================================================
// ============================ P E N   D R I V E    reparado ===================================================

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
    digitalWrite(pino_LED_pendrive, HIGH);
    delay(500);
    digitalWrite(pino_LED_pendrive, LOW);
}


