#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <SPI.h>
#include <max6675.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <AccelStepper.h>

#include "button.h"
#include "storage.h"
#include "clientServer.h"
#include "wifiAP.h"
#include <PubSubClient.h>

const int thermoDO = 12; // SO (MISO)
const int thermoCS = 15; // CS
const int thermoCLK = 14; // SCK

const int stepPin = 19;
const int dirPin = 18;

const int ledPin = 2;
const int pemantik = 4;
#define BOOT_BUTTON GPIO_NUM_0

byte ATuneModeRemember=2; // Menyimpan mode PID sebelum autotune (0: MANUAL, 1: AUTOMATIC, 2: default)
double input, output, setpoint=52.50; // input: nilai proses (misal suhu), output: nilai keluaran ke aktuator, setpoint: target yang diinginkan

double kp,ki,kd; // kp: konstanta proporsional, ki: konstanta integral, kd: konstanta derivatif
// double kp=3.06,ki=0.03,kd=0.27;
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
const char* mqtt_topic = "esp32/pid/suhu"; // Topik untuk publikasi data suhu
unsigned long lastMsg = 0; // MQTT: Variabel untuk timer pengiriman data
unsigned long lastReconnectAttempt = 0; // Timer untuk reconnection
double kpmodel=1.5, taup=100, theta[50]; // kpmodel: konstanta proporsional model, taup: waktu tunda model, theta: array untuk simulasi model proses
double outputStart=0; // output awal untuk simulasi atau autotune
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=400; // aTuneStep: besaran langkah output autotune, aTuneNoise: toleransi noise autotune, aTuneStartValue: nilai awal output autotune
unsigned int aTuneLookBack=20; // durasi lookback autotune dalam detik
String ssid = "";
String password = "";
boolean tuning = false;
unsigned long  modelTime, serialTime;

double outputMin = 0;
double outputMax = 1600; 

storage *memory;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&input, &output);

button *tuningButton;

clientServer *server;
wifiAP *mywifi;

WiFiClient espClient;
PubSubClient client(espClient);

SemaphoreHandle_t inputMutex;

//set to false to connect to the real world
boolean useSimulation = false;

double readSensor(){
  double value = thermocouple.readCelsius();
  if (isnan(value))
  {
    value = 0;
    
  }
  return value;
  
}

bool reconnect() {
  // unsigned long now = millis();
  // Coba reconnect maksimal setiap 5 detik
  // if (now - lastReconnectAttempt > 5000) {
  //   lastReconnectAttempt = now;
    
    Serial.print("Attempting MQTT connection...");
    // Buat client ID acak
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    // Coba untuk terhubung
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      return true;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      return false;
    }
  // }
  return false;
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}

void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}
void SerialSend()
{
  // Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  // Serial.print("suhu: ");Serial.print(input); Serial.print(" ");
  // Serial.print("output: ");Serial.print(-output); Serial.print(" ");
  // Serial.print("current output: ");Serial.print(stepper.currentPosition());Serial.print(" ");
  // if(tuning){
  //   Serial.println("tuning mode");
  // } else {
  //   Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
  //   Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
  //   Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  // }
  delay(500);
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
  // if (tuningButton->getPress() == true) 
  // {
  //   changeAutoTune(); 
  //   tuningButton->setPress(false);
  // }
}

void DoModel()
{
  //cycle the dead time
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  //compute the input
  input = (kpmodel / taup) *(theta[0]-outputStart) + input*(1-1/taup) + ((float)random(-10,10))/100;

}

void applicationTask0(void *param);
// void loopCore0(void *param);
void applicationTask1(void *param);

void setup()
{
  Serial.begin(115200);
  memory = new storage();
  memory->init();
  memory->readWifi();


  // memory->savePID(kp, ki, kd);
  
  if(useSimulation)
  {
    for(byte i=0;i<50;i++)
    {
      theta[i]=outputStart;
    }
    modelTime = 0;
  }

  tuningButton = new button(BOOT_BUTTON);
  // Konfigurasi Motor Stepper
  stepper.setMaxSpeed(700);
  stepper.setAcceleration(300);
  stepper.setCurrentPosition(0);
  // stepper.moveTo(200);
  // delay(5000);
  // stepper.setCurrentPosition(0);
  myPID.SetOutputLimits(outputMin, outputMax);
  myPID.SetMode(AUTOMATIC);

  // kp = 8.06;
  // ki = 3.00;
  // kd = 1.27;
  kp = memory->getKp();
  ki = memory->getKi();
  kd = memory->getKd();
  myPID.SetTunings(kp,ki,kd);
  // memory->savePID(kp, ki, kd);

  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  serialTime = 0;
  tuningButton->begin();
  inputMutex = xSemaphoreCreateMutex();
  xSemaphoreGive(inputMutex);
  TaskHandle_t task0;
  xTaskCreatePinnedToCore(
    applicationTask0,
    "applicationTask0",
    10000,
    NULL,
    1,
    &task0,
    0
  );

  // xTaskCreatePinnedToCore(
  //   loopCore0,
  //   "loopCore0",
  //   4096,
  //   NULL,
  //   2,
  //   &task0,
  //   0
  // );

  TaskHandle_t task1;
  xTaskCreatePinnedToCore(
    applicationTask1,
    "applicationTask1",
    10000,
    NULL,
    1,
    &task1,
    1
  );

}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(50));
}

void applicationTask0(void *param){
// double currentTemp;
bool accessPoint = false;
bool finishConnectApp = false;
double currentTemp = 0;
server = new clientServer(memory);
mywifi = new wifiAP();
ssid = memory->getSsid();
password = memory->getPass();

// Serial print untuk ssid dan password
Serial.print("SSID: ");
Serial.println(ssid);
Serial.print("Password: ");
Serial.println(password);

if (ssid != "" && password != ""){
  Serial.println("Mencoba koneksi dengan kredensial tersimpan");
  mywifi->disconnectAP();
  mywifi->connectSTA();
  mywifi->connectToWiFi(ssid, password);
  if (mywifi->getMode()){
    client.setServer(mqtt_server, mqtt_port);
    if (!client.connected()) {
      reconnect();
    }
  }
}

if (!mywifi->getMode())
{
  Serial.println("Mengaktifkan mode Access Point dan Server");
  mywifi->connectAP();
  mywifi->setup();
  server->init();
  server->notFound();
  server->setup();
}
  for (;;)
  {
    if(xSemaphoreTake(inputMutex, portMAX_DELAY) == pdTRUE) {
      currentTemp = input;
    xSemaphoreGive(inputMutex);
    }
    if (server->getConneect() && server->getConneect() != finishConnectApp)
    {
      finishConnectApp = true;
      ssid = memory->getSsid();
      password = memory->getPass();
      Serial.println("Koneksi WiFi terputus");
      mywifi->connectSTA();
      mywifi->connectToWiFi(ssid, password);
      if (mywifi->getMode()){
        client.setServer(mqtt_server, mqtt_port);
        if (!client.connected()) {
          reconnect();
        }
        // else
        // {
        //   Serial.println("\nGagal terhubung ke MQTT");
        //   mywifi->connectAP();
        //   mywifi->setup();
        // }
      }else if(!mywifi->getMode()){
        // Jika gagal terhubung kembali, aktifkan mode AP
        Serial.println("Mengaktifkan mode Access Point");
        mywifi->setup();
        server->init();
        server->setup();
        server->notFound();
        finishConnectApp = false;
      }
    }

    if (WiFi.getMode() == WIFI_STA && WiFi.status() != WL_CONNECTED && !accessPoint) {
      Serial.println(WiFi.status());
      mywifi->connectToWiFi(ssid, password);
      if(!mywifi->getMode()){
        // Jika gagal terhubung kembali, aktifkan mode AP
        Serial.println("Mengaktifkan mode Access Point");
        mywifi->setup();
        server->init();
        server->setup();
        server->notFound();
        accessPoint = true;
      }
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      if (!client.connected()) {
        reconnect();
      }
      unsigned long now = millis();
      if (now - lastMsg > 5000) { // Interval pengiriman 5000 ms = 5 detik
          lastMsg = now;
          
          char tempString[8];
          // Ubah nilai float suhu menjadi string
          snprintf(tempString, 8, "%.2f", currentTemp);
          Serial.print("Publish message: ");
          Serial.println(tempString);
            
            
            client.publish(mqtt_topic, tempString);
      }
    }
    mywifi->loopDns();
    client.loop();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// void loopCore0(void *param){
//   for (;;)
//   {
//     mywifi->loopDns();
//     client.loop();
//     vTaskDelay(pdMS_TO_TICKS(50));
//   }
// }

void applicationTask1(void *param){
  for (;;)
  {
    if(xSemaphoreTake(inputMutex, pdMS_TO_TICKS(100)) == pdTRUE) { 
      // Baca suhu dari sensor
      if(!useSimulation) {
          input = readSensor();
      }
      // Lepaskan mutex setelah selesai
      xSemaphoreGive(inputMutex);
    }
    unsigned long now = millis();
  
    if(tuning)
    {
      byte val = (aTune.Runtime());
      if (val!=0)
      {
        tuning = false;
      }
      if(!tuning)
      { //we're done, set the tuning parameters
        kp = aTune.GetKp();
        ki = aTune.GetKi();
        kd = aTune.GetKd();
        myPID.SetTunings(kp,ki,kd);
        memory->savePID(kp, ki, kd);
        AutoTuneHelper(false);
      }
    }
    else myPID.Compute();
    
    if(useSimulation)
    {
      theta[30]=output;
      if(now>=modelTime)
      {
        modelTime +=100; 
        DoModel();
      }
    }
    else
    {
      stepper.moveTo(-output); 
      stepper.run(); 
      unsigned long nowSerial = millis();
      if (nowSerial - serialTime >= 500)
      {
        SerialReceive();
        SerialSend();
        serialTime = nowSerial;
      }
      if (stepper.currentPosition() >= -100)
      {
        digitalWrite(pemantik, HIGH);
        // Serial.println("pemantik nyala");
      }else
      {
        digitalWrite(pemantik, LOW);
      }
    }
  }
}