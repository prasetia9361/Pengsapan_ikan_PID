#include <Arduino.h>
#include <SPI.h>
#include <max6675.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <AccelStepper.h>

#include "button.h"
// === Pin Konfigurasi ===
// Pin untuk Sensor Suhu MAX6675
const int thermoDO = 12; // SO
const int thermoCS = 15;
const int thermoCLK = 14; // SCK

// Pin untuk Driver Motor Stepper A4988
const int stepPin = 19;//19
const int dirPin = 18;//18
// Pin enable tidak selalu dibutuhkan oleh AccelStepper, namun kita tetap bisa menggunakannya
// const int enablePin = 5;
#define BOOT_BUTTON GPIO_NUM_0
long serialTime;
double Input;
double celcius;
// === Inisialisasi Komponen ===
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// <<< PERUBAHAN DIMULAI DI SINI: Inisialisasi dengan AccelStepper >>>
// Tipe driver (1), pin STEP, pin DIR
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);
// <<< PERUBAHAN SELESAI >>>

// === Konfigurasi PID & Autotune ===
double Setpoint, Output; // Variabel untuk PID

double Kp = 2, Ki = 0.05, Kd = 0.5; // Nilai awal, akan di-update oleh autotune

// Rentang output disesuaikan dengan langkah motor yang dibutuhkan
// Misal, dari valve tertutup penuh (0) hingga terbuka penuh (400 langkah)
// Sesuaikan nilai '400' ini dengan mekanisme Anda
double outputMin = 0;
double outputMax = 250; // Pastikan ini sesuai dengan kalibrasi fisik Anda

// Inisialisasi PID Controller
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Inisialisasi PID Autotune
PID_ATune aTune(&Input, &Output);

button *tuningButton;

// Variabel untuk proses tuning
bool tuning = false; // Status apakah sedang tuning atau tidak

void startTuning() {
  tuning = true;
  Serial.println("\nMemulai Proses Autotune PID...");
  Serial.println("Proses ini akan memakan waktu beberapa menit.");
  Serial.println("Sistem akan berosilasi di sekitar setpoint.");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Sistem Pengasap Ikan Otomatis");
  Serial.println("teBOOT button ESP32 untuk memulai proses Autotune PID.");
  tuningButton = new button(BOOT_BUTTON); 
  tuningButton->begin();

  // <<< PERUBAHAN DIMULAI DI SINI: Konfigurasi Motor Stepper >>>
  // pinMode(enablePin, OUTPUT);
  // digitalWrite(enablePin, LOW); // Aktifkan driver A4988

  stepper.setMaxSpeed(200);      // Kecepatan maksimum (langkah/detik)
  stepper.setAcceleration(100);  // Akselerasi (langkah/detik^2)
  stepper.setCurrentPosition(0); // PENTING: Asumsikan motor mulai dari posisi 0 (valve tertutup)
  // <<< PERUBAHAN SELESAI >>>

  // Tentukan Setpoint suhu (target di antara 50-55 C)
  Setpoint = 52.5;

  // Konfigurasi Autotune
  aTune.SetOutputStep(100);    // Seberapa besar perubahan output saat tuning, sesuaikan
  aTune.SetControlType(1);     // Tipe PID
  aTune.SetNoiseBand(0.5);     // Batas noise suhu (misal 0.5 C)
  aTune.SetLookbackSec(60);    // Waktu histori (detik) untuk analisis

  // Aktifkan PID
  myPID.SetOutputLimits(outputMin, outputMax);
  myPID.SetMode(AUTOMATIC);

  serialTime = millis();
}
bool lastbutton = false;
void loop() {
  // tuningButton->tick();
  // if (tuningButton->getPress() == true) 
  // {
  //   startTuning(); 
  //   tuningButton->setPress(false);
  // }
  if (Serial.available() > 0) {
    char val = Serial.read();
    if (val == 't' || val == 'T') {
      startTuning();
    }
  }

  // Baca suhu dari sensor
  celcius = thermocouple.readCelsius();

  if (isnan(thermocouple.readCelsius())) {
    Serial.println("Gagal membaca suhu dari termokopel!");
    return; // Keluar dari loop jika bacaan gagal
  }

  if (tuning) {
    byte val = aTune.Runtime();
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
  
  // <<< PERUBAHAN DIMULAI DI SINI: Kontrol Motor dengan AccelStepper >>>
  // Output PID menjadi target posisi motor
  stepper.moveTo(Output); 
  // Fungsi ini harus dipanggil sesering mungkin agar motor bergerak dengan mulus
  stepper.run(); 
  // <<< PERUBAHAN SELESAI >>>

  // Cetak informasi ke Serial Monitor setiap 2 detik
  if (millis() - serialTime > 2000) {
    serialTime = millis();
    Serial.print("Setpoint: "); Serial.print(Setpoint);
    Serial.print(" | Suhu: "); Serial.print(celcius);
    Serial.print(" | PID Output (Target Pos): "); Serial.print(Output);
    Serial.print(" | Posisi Motor Aktual: "); Serial.println(stepper.currentPosition());
  }
}