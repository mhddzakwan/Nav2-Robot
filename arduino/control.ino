// ============================================================
//  robot_mino_firmware.ino
//  2WD Differential Drive — Arduino Nano 33 BLE Sense Rev1
//  Didesain untuk dipakai dengan ros2_control (diff_drive_controller)
//
//  PERBEDAAN UTAMA dari versi serial_bridge manual sebelumnya:
//  [1] Kinematika invers DIHAPUS dari Arduino — dipindah ke
//      diff_drive_controller di sisi ROS 2.
//  [2] Protokol RX berubah: terima target RAD/S PER RODA langsung
//      (bukan linear_x,angular_z lagi).
//  [3] CMD_TIMEOUT_MS diperbaiki jadi 500ms (sebelumnya salah ketik 10000).
//  [4] Struktur tetap: PID per-roda di Arduino, ROS hanya kirim
//      target velocity & terima ticks+rpm+imu.
// ============================================================

#include <Arduino.h>
#include <Arduino_LSM9DS1.h>

// ============================================================
//  PARAMETER ROBOT — WAJIB diukur/kalibrasi ulang untuk robot_mino
//  Jangan pakai angka lama dari project sebelumnya tanpa verifikasi,
//  karena WHEEL_DIAMETER & WHEELBASE robot_mino kemungkinan beda fisik.
// ============================================================
const float COUNTS_PER_REV = 1012.0;   // [GANTI] Ticks per 1 putaran roda penuh
                                         // Cara ukur: tandai roda dgn spidol, putar
                                         // manual 1x penuh sambil baca posisiEnc di
                                         // Serial Monitor, catat selisihnya.

const float MAX_RPM         = 100.0;    // [GANTI] RPM maksimum motor pada tegangan suplai kamu
const float MAX_PWM         = 255.0;    // Tetap (PWM 8-bit Arduino)

// CATATAN: WHEEL_DIAMETER dan WHEELBASE TIDAK ada di firmware ini.
// Kinematika invers (linear/angular -> rad/s per roda) sudah dipindah
// ke diff_drive_controller di sisi ROS. Dimensi fisik robot didefinisikan
// di controllers.yaml sebagai:
//   wheel_radius     (= WHEEL_DIAMETER lama / 2)
//   wheel_separation (= WHEELBASE lama)
// Arduino hanya perlu tahu COUNTS_PER_REV (buat hitung RPM dari ticks)
// dan MAX_RPM (buat limit & normalisasi PWM).

// ============================================================
//  PARAMETER PID — JANGAN tuning dulu sebelum hardware test lolos
//  Mulai dari nilai konservatif ini, naikkan bertahap (lihat panduan tuning di bawah)
// ============================================================
float Kp = 1.0;     // [TUNING] mulai rendah
float Ki = 0.3;     // [TUNING]
float Kd = 0.05;    // [TUNING]

const float INTEGRAL_LIMIT = 150.0;  // Anti-windup, sesuaikan jika PWM sering saturasi

// ============================================================
//  TRIM RODA — kompensasi asimetri fisik (diameter roda, slip, dll)
//  Bisa diubah runtime via serial (lihat bacaSerialKomunikasi),
//  TAPI reset ke 1.0 tiap kali Arduino reboot/reflash.
//  Setelah ketemu nilai final dari kalibrasi, HARDCODE di sini permanen.
// ============================================================
float TRIM_KIRI  = 1.000;
float TRIM_KANAN = 1.000;

// ============================================================
//  WATCHDOG — [FIX] dari 10000 (salah) jadi 500ms (benar)
//  Harus selaras dengan cmd_vel_timeout di controllers.yaml nanti
// ============================================================
const unsigned long CMD_TIMEOUT_MS = 500;

// ============================================================
//  PIN DEFINITION — TB6612FNG Motor Driver
//  [GANTI sesuai wiring fisik robot_mino, ini cuma contoh]
// ============================================================
const int pinPWMA = 5;   // Motor KIRI - Channel A
const int pinAIN1 = 6;
const int pinAIN2 = 7;

const int pinPWMB = 10;  // Motor KANAN - Channel B
const int pinBIN1 = 8;
const int pinBIN2 = 9;

const int pinSTBY = 4;

const int encKiriA  = 3;   // WAJIB pin interrupt-capable
const int encKiriB  = 2;
const int encKananA = 12;  // WAJIB pin interrupt-capable
const int encKananB = 11;

// ============================================================
//  VARIABEL ENCODER (volatile — diakses dari ISR)
// ============================================================
volatile long posisiEncKiri  = 0;
volatile long posisiEncKanan = 0;

// ============================================================
//  VARIABEL LOOP KONTROL
// ============================================================
long prevEncKiri  = 0;
long prevEncKanan = 0;

unsigned long prevPIDTime = 0;
unsigned long lastCmdTime = 0;

// Target RPM per roda (dikonversi dari rad/s yang diterima via serial)
float targetRPMKiri  = 0.0;
float targetRPMKanan = 0.0;
float aktualRPMKiri  = 0.0;
float aktualRPMKanan = 0.0;

// State PID
float errIntKiri  = 0.0, prevErrKiri  = 0.0;
float errIntKanan = 0.0, prevErrKanan = 0.0;

int pwmOutputKiri  = 0;
int pwmOutputKanan = 0;

// ============================================================
//  BUFFER SERIAL
// ============================================================
String inputString    = "";
bool   stringComplete  = false;

// ============================================================
//  DEKLARASI FUNGSI
// ============================================================
void hitungRPMUpdate(float dt);
void updatePID();
int  rpmKePWM(float rpm, float maxRPM);
void setMotor(int pwmKiri, int pwmKanan);
void stopMotor();
void bacaSerialKomunikasi();
void kirimDataKeRaspy();
void bacaEncoderKiri();
void bacaEncoderKanan();

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(pinPWMA, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);
  pinMode(pinPWMB, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  pinMode(pinSTBY, OUTPUT);
  digitalWrite(pinSTBY, HIGH);

  pinMode(encKiriA,  INPUT_PULLUP);
  pinMode(encKiriB,  INPUT_PULLUP);
  pinMode(encKananA, INPUT_PULLUP);
  pinMode(encKananB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encKiriA),  bacaEncoderKiri,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(encKananA), bacaEncoderKanan, CHANGE);

  if (!IMU.begin()) {
    // IMU gagal — data IMU akan terkirim sebagai 0.0
    // Robot tetap bisa gerak, tapi EKF nanti kehilangan satu sumber data
  }

  inputString.reserve(64);
  stopMotor();

  Serial.println(F("#ROBOT_READY"));
}

// ============================================================
//  LOOP UTAMA
// ============================================================
void loop() {
  bacaSerialKomunikasi();

  if (millis() - lastCmdTime > CMD_TIMEOUT_MS) {
    if (targetRPMKiri != 0.0 || targetRPMKanan != 0.0) {
      targetRPMKiri  = 0.0;
      targetRPMKanan = 0.0;
    }
  }

  unsigned long now = millis();
  if (now - prevPIDTime >= 20) {   // Loop PID 50Hz
    float dt = (now - prevPIDTime) / 1000.0f;
    prevPIDTime = now;

    hitungRPMUpdate(dt);
    updatePID();
    kirimDataKeRaspy();
  }
}

// ============================================================
//  TERIMA PERINTAH DARI ROS
//  Format velocity: "wL_radps,wR_radps"           contoh: "2.50,-2.50"
//  Format trim     : "T,trimKiri,trimKanan"        contoh: "T,1.00,1.02"
//  (trim runtime untuk kalibrasi cepat, TIDAK persisten setelah reboot)
// ============================================================
void bacaSerialKomunikasi() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      stringComplete = true;
    } else if (c != '\r') {
      inputString += c;
    }
  }

  if (stringComplete) {
    if (inputString.startsWith("T,")) {
      String rest = inputString.substring(2);
      int commaIdx = rest.indexOf(',');
      if (commaIdx > 0) {
        TRIM_KIRI  = rest.substring(0, commaIdx).toFloat();
        TRIM_KANAN = rest.substring(commaIdx + 1).toFloat();
      }
    } else {
      int commaIdx = inputString.indexOf(',');
      if (commaIdx > 0) {
        float wL = inputString.substring(0, commaIdx).toFloat();
        float wR = inputString.substring(commaIdx + 1).toFloat();

        const float RADPS_TO_RPM = 60.0f / (2.0f * PI);
        targetRPMKiri  = constrain(wL * RADPS_TO_RPM * TRIM_KIRI,  -MAX_RPM, MAX_RPM);
        targetRPMKanan = constrain(wR * RADPS_TO_RPM * TRIM_KANAN, -MAX_RPM, MAX_RPM);

        lastCmdTime = millis();
      }
    }
    inputString    = "";
    stringComplete = false;
  }
}

// ============================================================
//  KIRIM TELEMETRI KE ROS
//  Format TX (tidak berubah dari versi sebelumnya):
//  #timestamp_ms,ticksL,ticksR,rpmL,rpmR,ax,ay,az,gx,gy,gz
// ============================================================
void kirimDataKeRaspy() {
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    ax *= 9.80665f; ay *= 9.80665f; az *= 9.80665f;
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    const float DEG2RAD = PI / 180.0f;
    gx *= DEG2RAD; gy *= DEG2RAD; gz *= DEG2RAD;
  }

  noInterrupts();
  long ticksL = posisiEncKiri;
  long ticksR = posisiEncKanan;
  interrupts();

  Serial.print('#');
  Serial.print(millis());         Serial.print(',');
  Serial.print(ticksL);           Serial.print(',');
  Serial.print(ticksR);           Serial.print(',');
  Serial.print(aktualRPMKiri, 2); Serial.print(',');
  Serial.print(aktualRPMKanan,2); Serial.print(',');
  Serial.print(ax, 4);            Serial.print(',');
  Serial.print(ay, 4);            Serial.print(',');
  Serial.print(az, 4);            Serial.print(',');
  Serial.print(gx, 4);            Serial.print(',');
  Serial.print(gy, 4);            Serial.print(',');
  Serial.println(gz, 4);
}

// ============================================================
//  HITUNG RPM AKTUAL dari encoder
// ============================================================
void hitungRPMUpdate(float dt) {
  if (dt <= 0) return;

  noInterrupts();
  long curL = posisiEncKiri;
  long curR = posisiEncKanan;
  interrupts();

  long deltaL = curL - prevEncKiri;
  long deltaR = curR - prevEncKanan;

  prevEncKiri  = curL;
  prevEncKanan = curR;

  aktualRPMKiri  = ((float)deltaL / COUNTS_PER_REV) * (60.0f / dt);
  aktualRPMKanan = ((float)deltaR / COUNTS_PER_REV) * (60.0f / dt);
}

// ============================================================
//  UPDATE PID — dipanggil tiap 20ms
// ============================================================
void updatePID() {
  float errorKiri = targetRPMKiri - aktualRPMKiri;
  if (targetRPMKiri == 0.0f && fabsf(aktualRPMKiri) < 1.0f) {
    errIntKiri = 0.0f;
  } else {
    errIntKiri += errorKiri;
    errIntKiri  = constrain(errIntKiri, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  }
  float errDerKiri = errorKiri - prevErrKiri;
  float pidOutKiri = (Kp * errorKiri) + (Ki * errIntKiri) + (Kd * errDerKiri);
  prevErrKiri      = errorKiri;

  float errorKanan = targetRPMKanan - aktualRPMKanan;
  if (targetRPMKanan == 0.0f && fabsf(aktualRPMKanan) < 1.0f) {
    errIntKanan = 0.0f;
  } else {
    errIntKanan += errorKanan;
    errIntKanan  = constrain(errIntKanan, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  }
  float errDerKanan = errorKanan - prevErrKanan;
  float pidOutKanan = (Kp * errorKanan) + (Ki * errIntKanan) + (Kd * errDerKanan);
  prevErrKanan      = errorKanan;

  pwmOutputKiri  = rpmKePWM(pidOutKiri,  MAX_RPM);
  pwmOutputKanan = rpmKePWM(pidOutKanan, MAX_RPM);

  setMotor(pwmOutputKiri, pwmOutputKanan);
}

int rpmKePWM(float rpmValue, float maxRPM) {
  float clamped = constrain(rpmValue, -maxRPM, maxRPM);
  return (int)((clamped / maxRPM) * MAX_PWM);
}

void setMotor(int pwmKiri, int pwmKanan) {
  pwmKiri  = constrain(pwmKiri,  -255, 255);
  pwmKanan = constrain(pwmKanan, -255, 255);

  if (pwmKiri > 0)      { digitalWrite(pinAIN1, LOW);  digitalWrite(pinAIN2, HIGH); }
  else if (pwmKiri < 0) { digitalWrite(pinAIN1, HIGH); digitalWrite(pinAIN2, LOW);  }
  else                  { digitalWrite(pinAIN1, LOW);  digitalWrite(pinAIN2, LOW);  }
  analogWrite(pinPWMA, abs(pwmKiri));

  if (pwmKanan > 0)      { digitalWrite(pinBIN1, HIGH); digitalWrite(pinBIN2, LOW);  }
  else if (pwmKanan < 0) { digitalWrite(pinBIN1, LOW);  digitalWrite(pinBIN2, HIGH); }
  else                   { digitalWrite(pinBIN1, LOW);  digitalWrite(pinBIN2, LOW);  }
  analogWrite(pinPWMB, abs(pwmKanan));
}

void stopMotor() {
  digitalWrite(pinAIN1, LOW); digitalWrite(pinAIN2, LOW);
  digitalWrite(pinBIN1, LOW); digitalWrite(pinBIN2, LOW);
  analogWrite(pinPWMA, 0);
  analogWrite(pinPWMB, 0);

  targetRPMKiri  = 0;
  targetRPMKanan = 0;
  errIntKiri     = 0;
  errIntKanan    = 0;
}

// ============================================================
//  ISR ENCODER — quadrature, bandingkan fase A vs B untuk arah
// ============================================================
void bacaEncoderKiri() {
  if (digitalRead(encKiriA) == digitalRead(encKiriB)) posisiEncKiri++;
  else posisiEncKiri--;
}

void bacaEncoderKanan() {
  // Kanan sering terpasang terbalik secara mekanik -> arah dinegasikan
  if (digitalRead(encKananA) == digitalRead(encKananB)) posisiEncKanan--;
  else posisiEncKanan++;
}
