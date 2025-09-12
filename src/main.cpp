#include <Arduino.h>
#include <SPI.h>
#include <max6675.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <AccelStepper.h>

// MQTT: Include library WiFi dan PubSubClient
#include <WiFi.h>
#include <PubSubClient.h>

#include "button.h"
#include "storage.h"

// --- Konfigurasi Perangkat Keras ---
const int thermoDO = 12; // SO (MISO)
const int thermoCS = 15; // CS
const int thermoCLK = 14; // SCK
const int stepPin = 19;
const int dirPin = 18;
const int ledPin = 2;
const int pemantik = 4;
#define BOOT_BUTTON GPIO_NUM_0

// --- Konfigurasi WiFi ---
// MQTT: Ganti dengan kredensial WiFi Anda
const char* ssid = "wefee";
const char* password = "wepaywefee";

// --- Konfigurasi MQTT ---
// MQTT: Konfigurasi untuk broker EMQX publik
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
const char* mqtt_topic = "esp32/pid/suhu"; // Topik untuk publikasi data suhu

// MQTT: Inisialisasi client WiFi dan MQTT
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0; // MQTT: Variabel untuk timer pengiriman data
unsigned long lastReconnectAttempt = 0; // Timer untuk reconnection

// --- Variabel PID & Autotune ---
byte ATuneModeRemember=2;
double input, output, setpoint=52.50;
double kp,ki,kd;
double kpmodel=1.5, taup=100, theta[50];
double outputStart=0;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=1600;
unsigned int aTuneLookBack=20;
boolean tuning = false;
unsigned long modelTime, serialTime;
double outputMin = 0;
double outputMax = 1600; 

// --- Objek & Instance ---
storage *memory;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);
PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&input, &output);
button *tuningButton;

boolean useSimulation = false;

// MQTT: Fungsi untuk koneksi WiFi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Perbaikan fungsi reconnect untuk menghindari stack overflow
bool reconnect() {
  unsigned long now = millis();
  // Coba reconnect maksimal setiap 5 detik
  if (now - lastReconnectAttempt > 5000) {
    lastReconnectAttempt = now;
    
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
  }
  return false;
}

void AutoTuneHelper(boolean start)
{
  if(start)
  {
    ATuneModeRemember = myPID.GetMode();
    digitalWrite(ledPin, HIGH);
  }
  else
  {
    myPID.SetMode(ATuneModeRemember);
    digitalWrite(ledPin, LOW);
  }
}

void changeAutoTune()
{
 if(!tuning)
  {
    output=-aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  {
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
  if (stepper.currentPosition() >= -200)
  {
    digitalWrite(pemantik, HIGH);
  }else
  {
    digitalWrite(pemantik, LOW);
  }
  // if(tuning){
  //   Serial.println("tuning mode");
  // } else {
  //   Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
  //   Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
  //   Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  // }
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }

  if (tuningButton->getPress() == true) 
  {
    changeAutoTune(); 
    // Reset flag agar fungsi tidak dipanggil berulang kali
    tuningButton->setPress(false);
  }
}

void DoModel()
{
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  input = (kpmodel / taup) *(theta[0]-outputStart) + input*(1-1/taup) + ((float)random(-10,10))/100;
}

void applicationTask0(void *param);
void loopCore0(void *param);
void applicationTask1(void *param);

void setup()
{
  Serial.begin(115200);
  
  memory = new storage();
  memory->init();
  
  if(useSimulation)
  {
    for(byte i=0;i<50;i++)
    {
      theta[i]=outputStart;
    }
    modelTime = 0;
  }

  tuningButton = new button(BOOT_BUTTON);
  pinMode(ledPin, OUTPUT);
  pinMode(pemantik, OUTPUT);
  stepper.setMaxSpeed(700);
  stepper.setAcceleration(300);
  stepper.setCurrentPosition(0);

  myPID.SetOutputLimits(outputMin, outputMax);
  myPID.SetMode(AUTOMATIC);

  // kp = 8.06;
  // ki = 3.00;
  // kd = 1.27;
  kp = memory->getKp();
  ki = memory->getKi();
  kd = memory->getKd();
  myPID.SetTunings(kp,ki,kd);

  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  serialTime = 0;
  tuningButton->begin();

  TaskHandle_t task0;
  xTaskCreatePinnedToCore(
    applicationTask0,
    "applicationTask0",
    8192,  // Ditingkatkan dari 10000 ke 8192 (lebih aman)
    NULL,
    1,
    &task0,
    0
  );
  xTaskCreatePinnedToCore(
    loopCore0,
    "loopCore0",
    4096,  // Ditingkatkan dari 1000 ke 4096 untuk MQTT operations
    NULL,
    2,
    &task0,
    0
  );

  TaskHandle_t task1;
  xTaskCreatePinnedToCore(
    applicationTask1,
    "applicationTask1",
    8192,  // Ditingkatkan dari 10000 ke 8192
    NULL,
    1,
    &task1,
    1
  );
} // Menambahkan kurung kurawal yang hilang

void loop()
{
  tuningButton->tick();
  vTaskDelay(pdMS_TO_TICKS(50));
}

void applicationTask0(void *param){
  // MQTT: Setup WiFi dan koneksi ke MQTT Broker
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  for (;;)
  {
    input = thermocouple.readCelsius();
    if(!useSimulation)
    {
      if (isnan(input)) {
        Serial.println("Gagal membaca suhu dari termokopel!");
        delay(500); 
        continue; // Lanjut ke iterasi berikutnya jika bacaan gagal
      }
    }
  
    unsigned long nowSerial = millis();
    if (nowSerial - serialTime >= 500)
    {
      SerialReceive();
      SerialSend();
      serialTime = nowSerial;
    }

    unsigned long now = millis();
    if (now - lastMsg > 5000) { // Interval pengiriman 5000 ms = 5 detik
        lastMsg = now;
        
        // Cek jika bacaan suhu valid sebelum mengirim
        if (!isnan(input)) {
          char tempString[8];
          // Ubah nilai float suhu menjadi string
          snprintf(tempString, 8, "%.2f", input);
          Serial.print("Publish message: ");
          Serial.println(tempString);
          
          // Publikasikan pesan ke topik MQTT
          client.publish(mqtt_topic, tempString);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void loopCore0(void *param){
  for (;;)
  {
    if (!client.connected()) {
      reconnect();
    }
    
    client.loop();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void applicationTask1(void *param){
  for (;;)
  {
    unsigned long now = millis();
  
    if(tuning)
    {
      byte val = (aTune.Runtime());
      if (val!=0)
      {
        tuning = false;
      }
      if(!tuning)
      {
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
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Jeda singkat
  }
}