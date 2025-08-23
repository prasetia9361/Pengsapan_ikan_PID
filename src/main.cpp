#include <Arduino.h>
#include <SPI.h>
#include <max6675.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <AccelStepper.h>

#include "button.h"
// === Pin Konfigurasi ===
// Pin untuk Sensor Suhu MAX6675
const int thermoDO = 12; // SO (MISO)
const int thermoCS = 15; // CS
const int thermoCLK = 14; // SCK

// Pin untuk Driver Motor Stepper A4988
const int stepPin = 19;
const int dirPin = 18;

#define BOOT_BUTTON GPIO_NUM_0

long serialTime;
double Input;

// === Inisialisasi Komponen ===
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// Inisialisasi dengan AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

// === Konfigurasi PID & Autotune ===
double Setpoint, Output; // Variabel untuk PID
double Kp = 2, Ki = 0.05, Kd = 0.5; // Nilai awal, akan di-update oleh autotune

double outputMin = 0;
double outputMax = 250; 

// Inisialisasi PID Controller
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Inisialisasi PID Autotune
PID_ATune aTune(&Input, &Output);

button *tuningButton;

// Variabel untuk proses tuning
bool tuning = false;

void startTuning() {
  tuning = true;
  Serial.println("\nMemulai Proses Autotune PID...");
  Serial.println("Proses ini akan memakan waktu beberapa menit.");
  Serial.println("Sistem akan berosilasi di sekitar setpoint.");
}

void applicationTask0(void *param);
void applicationTask1(void *param);

void setup() {
  Serial.begin(115200);
  Serial.println("Sistem Pengasap Ikan Otomatis");
  Serial.println("Kirim 't' atau 'T' melalui Serial Monitor untuk memulai Autotune PID.");
  tuningButton = new button(BOOT_BUTTON); 
  // Konfigurasi Motor Stepper
  stepper.setMaxSpeed(200);      // Kecepatan maksimum (langkah/detik)
  stepper.setAcceleration(7000);  // Akselerasi (langkah/detik^2)
  stepper.setCurrentPosition(0); // Asumsikan motor mulai dari posisi 0 (valve tertutup)

  // Tentukan Setpoint suhu (target di antara 50-55 C)
  Setpoint = 52.5;

  // Konfigurasi Autotune
  aTune.SetOutputStep(100);
  aTune.SetControlType(1);
  aTune.SetNoiseBand(0.5);
  aTune.SetLookbackSec(60);

  // Aktifkan PID
  myPID.SetOutputLimits(outputMin, outputMax);
  myPID.SetMode(AUTOMATIC);

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

void loop() {
  tuningButton->tick();
  vTaskDelay(pdMS_TO_TICKS(50));
}

void applicationTask0(void *param){
  serialTime = millis();
  for (;;)
  {
    Input = thermocouple.readCelsius();

    if (isnan(Input)) {
      Serial.println("Gagal membaca suhu dari termokopel!");
      // Kita bisa menambahkan jeda singkat di sini untuk memberi sensor waktu
      // sebelum loop berikutnya mencoba membaca lagi.
      delay(500); 
      return; // Keluar dari iterasi loop ini jika bacaan gagal
    }
  
    // Cetak informasi ke Serial Monitor setiap 2 detik
    if (millis() - serialTime > 2000) {
      serialTime = millis();
      Serial.print("Setpoint: "); Serial.print(Setpoint);
      Serial.print(" C | Suhu: "); Serial.print(Input);
      Serial.print(" C | PID Output (Target Pos): "); Serial.print(Output);
      Serial.print(" | Posisi Motor Aktual: "); Serial.println(stepper.currentPosition());
      delay(500);
    }
    delay(50);
  }
}

void applicationTask1(void *param){
  tuningButton->begin();
  for (;;)
  {
    if (tuningButton->getPress() == true) 
    {
      startTuning(); 
      tuningButton->setPress(false);
    }
  
    if (tuning) {
      byte val = aTune.Runtime();
      Serial.println(val);
      if (val != 0) {
        tuning = false;
        Kp = aTune.GetKp();
        Ki = aTune.GetKi();
        Kd = aTune.GetKd();
        myPID.SetTunings(Kp, Ki, Kd);
        
        Serial.println("\nProses Autotune Selesai!");
        Serial.println("Nilai PID baru telah diterapkan:");
        Serial.print("Kp: "); Serial.println(Kp);
        Serial.print("Ki: "); Serial.println(Ki);
        Serial.print("Kd: "); Serial.println(Kd);
      }
    } else {
      myPID.Compute(); // Jalankan mode PID normal
    }
    
    // Kontrol Motor dengan AccelStepper
    stepper.moveTo(Output); 
    stepper.run(); 
    delay(50);
  }
}