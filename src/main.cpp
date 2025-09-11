#include <Arduino.h>
#include <SPI.h>
#include <max6675.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <AccelStepper.h>

#include "button.h"
#include "storage.h"

const int thermoDO = 12; // SO (MISO)
const int thermoCS = 15; // CS
const int thermoCLK = 14; // SCK

const int stepPin = 19;
const int dirPin = 18;

#define BOOT_BUTTON GPIO_NUM_0

byte ATuneModeRemember=2; // Menyimpan mode PID sebelum autotune (0: MANUAL, 1: AUTOMATIC, 2: default)
double input, output, setpoint=52.50; // input: nilai proses (misal suhu), output: nilai keluaran ke aktuator, setpoint: target yang diinginkan

double kp,ki,kd; // kp: konstanta proporsional, ki: konstanta integral, kd: konstanta derivatif
// double kp=3.06,ki=0.03,kd=0.27;

double kpmodel=1.5, taup=100, theta[50]; // kpmodel: konstanta proporsional model, taup: waktu tunda model, theta: array untuk simulasi model proses
double outputStart=0; // output awal untuk simulasi atau autotune
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=400; // aTuneStep: besaran langkah output autotune, aTuneNoise: toleransi noise autotune, aTuneStartValue: nilai awal output autotune
unsigned int aTuneLookBack=20; // durasi lookback autotune dalam detik

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


//set to false to connect to the real world
boolean useSimulation = false;

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
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("suhu: ");Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(-output); Serial.print(" ");
  Serial.print("current output: ");Serial.print(stepper.currentPosition());Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  }
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
void applicationTask1(void *param);

void setup()
{
  Serial.begin(115200);
  memory = new storage();
  memory->init();


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
  for (;;)
  {
    input = thermocouple.readCelsius();
    if(!useSimulation)
    { //pull the input in from the real world
      if (isnan(input)) {
        Serial.println("Gagal membaca suhu dari termokopel!");
        // Kita bisa menambahkan jeda singkat di sini untuk memberi sensor waktu
        // sebelum loop berikutnya mencoba membaca lagi.
        delay(500); 
        return; // Keluar dari iterasi loop ini jika bacaan gagal
      }
    }
  
    //send-receive with processing if it's time
    if(millis()>serialTime)
    {
      SerialReceive();
      SerialSend();
      serialTime+=500;
    }
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
    }
  }
}