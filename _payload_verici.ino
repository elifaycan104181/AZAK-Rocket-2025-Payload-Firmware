// --- KÜTÜPHANE TANIMLAMALARI ---
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include <Adafruit_LSM6DSOX.h>
#include <math.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// --- DONANIM PİN TANIMLAMALARI ---
#define LSM_CS 7
#define M0 19
#define M1 20
// --- SERİ PORT TANIMALAMLARI ---
HardwareSerial FixSerial(2);
HardwareSerial GpsSerial(1);
// --- SENSÖR NESNE TANIMLAMALARI ---
TinyGPSPlus gps;
Adafruit_LSM6DSOX sox;
// --- HABERLEŞME DEĞİŞKENLERİ ---
uint8_t sent[42];
// --- GPS ---
float gps_irtifa;
float gps_enlem;
float gps_boylam;
bool gps_valid;
// --- IMU ve Euler ---
float pitch_deg = 0.0, roll_deg = 0.0;
float rocket_angle_to_normal_deg = 0.0;
bool aciKosuluSaglandi;
// --- Mahony filtre değişkenleri ---
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
#define Kp 2.0f
#define Ki 0.005f
float eInt[3] = { 0.0f, 0.0f, 0.0f };

// --- Dönüştürülmüş Roket Eksen İvmesi ---
float ax_roket = 0.0, ay_roket = 0.0, az_roket = 0.0;

// --- Zamanlama ---
unsigned long lastIMUUpdateTime = 0;
// --- Alçak Geçiren Filtre Değişkenleri ---
float alpha = 0.9f;  // LPF katsayısı (0-1 arasında, 1'e yakın daha ağır filtre)  TESTE GÖRE DEĞİŞTİR !!!!!!!!!!

// Filtrelenmiş ivme ve jiroskop değerleri için global değişkenler
float filtered_ax = 0, filtered_ay = 0, filtered_az = 0;
float filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;

// --- Fonksiyonlar ---
float kalmanFilter(float input, float *kalman_old, float *cov_old) {
  const float Q = 0.001f, R = 0.1f;
  *cov_old += Q;
  float K = *cov_old / (*cov_old + R);
  float result = *kalman_old + K * (input - *kalman_old);
  *cov_old *= (1 - K);
  *kalman_old = result;
  return result;
}
void Mahony_update_6dof(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float recipNorm, halfvx, halfvy, halfvz, halfex, halfey, halfez, qa, qb, qc;

  recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;

  halfvx = q1 * q3 - q0 * q2;
  halfvy = q0 * q1 + q2 * q3;
  halfvz = q0 * q0 - 0.5f + q3 * q3;

  halfex = (ay * halfvz - az * halfvy);
  halfey = (az * halfvx - ax * halfvz);
  halfez = (ax * halfvy - ay * halfvx);

  if (Ki > 0.0f) {
    eInt[0] += halfex * dt;
    eInt[1] += halfey * dt;
    eInt[2] += halfez * dt;
  } else {
    eInt[0] = eInt[1] = eInt[2] = 0.0f;
  }

  gx += Kp * halfex + Ki * eInt[0];
  gy += Kp * halfey + Ki * eInt[1];
  gz += Kp * halfez + Ki * eInt[2];

  dt *= 0.5f;
  gx *= dt;
  gy *= dt;
  gz *= dt;
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}
void initializeQuaternion() {
  sensors_event_t accel, gyro, temp;
  sox.getEvent(&accel, &gyro, &temp);
  float norm = sqrt(accel.acceleration.x * accel.acceleration.x + accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z);

  float ax = accel.acceleration.x / norm;
  float ay = accel.acceleration.y / norm;
  float az = accel.acceleration.z / norm;

  float initial_pitch_rad = atan2(ax, sqrt(ay * ay + az * az));
  float initial_roll_rad = atan2(ay, az);

  float cy = cos(0 * 0.5f), sy = sin(0 * 0.5f);
  float cp = cos(initial_pitch_rad * 0.5f), sp = sin(initial_pitch_rad * 0.5f);
  float cr = cos(initial_roll_rad * 0.5f), sr = sin(initial_roll_rad * 0.5f);

  q0 = cr * cp * cy + sr * sp * sy;
  q1 = sr * cp * cy - cr * sp * sy;
  q2 = cr * sp * cy + sr * cp * sy;
  q3 = cr * cp * sy - sr * sp * cy;

  float norm_q = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= norm_q;
  q1 /= norm_q;
  q2 /= norm_q;
  q3 /= norm_q;
}
// --- Haberleşmedeki checksum fonksiyonu ---
uint8_t checksumHesaplama(const uint8_t *data, size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < length; ++i) {
    checksum += data[i];
  }
  return checksum;
}
void GPSTask(void *pv) {
  (void)pv;
  for (;;) {
    while (GpsSerial.available()) { gps.encode(GpsSerial.read()); }
    if (gps.altitude.isValid() && gps.altitude.isUpdated()) { gps_irtifa = gps.altitude.meters(); }
    if (gps.location.isValid() && gps.location.isUpdated()) {
      gps_enlem = gps.location.lat();
      gps_enlem = gps_enlem * 1000000;
      gps_boylam = gps.location.lng();
      gps_boylam = gps_boylam * 1000000;
      gps_valid = true;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
void imuTask(void *pv) {
  // İlk filtrelenmiş değerleri başlat
  sensors_event_t accel, gyro, temp;
  sox.getEvent(&accel, &gyro, &temp);
  filtered_ax = accel.acceleration.y;
  filtered_ay = accel.acceleration.z;
  filtered_az = accel.acceleration.x;
  filtered_gx = gyro.gyro.y;
  filtered_gy = gyro.gyro.z;
  filtered_gz = gyro.gyro.x;

  for (;;) {
    unsigned long now = millis();
    float dt = (now - lastIMUUpdateTime) / 1000.0f;
    if (dt > 0.05f) dt = 0.05f;
    lastIMUUpdateTime = now;

    sox.getEvent(&accel, &gyro, &temp);

    // Ham veriler roket eksenine dönüştürülmüş halde
    float raw_ax_roket = accel.acceleration.y;
    float raw_ay_roket = accel.acceleration.z;
    float raw_az_roket = accel.acceleration.x;
    float raw_gx = gyro.gyro.y;
    float raw_gy = gyro.gyro.z;
    float raw_gz = gyro.gyro.x;

    // Alçak geçiren filtre (Low-Pass Filter) uygulaması
    filtered_ax = alpha * filtered_ax + (1.0f - alpha) * raw_ax_roket;
    filtered_ay = alpha * filtered_ay + (1.0f - alpha) * raw_ay_roket;
    filtered_az = alpha * filtered_az + (1.0f - alpha) * raw_az_roket;

    filtered_gx = alpha * filtered_gx + (1.0f - alpha) * raw_gx;
    filtered_gy = alpha * filtered_gy + (1.0f - alpha) * raw_gy;
    filtered_gz = alpha * filtered_gz + (1.0f - alpha) * raw_gz;

    // Filtrelenmiş verileri Mahony filtresine gönder
    Mahony_update_6dof(filtered_gx, filtered_gy, filtered_gz,
                       filtered_ax, filtered_ay, filtered_az, dt);

    roll_deg = atan2f(2.0f * (q0 * q1 + q2 * q3),
                      1.0f - 2.0f * (q1 * q1 + q2 * q2))
               * 180.0f / PI;
    pitch_deg = asinf(2.0f * (q0 * q2 - q1 * q3)) * 180.0f / PI;

    float rz = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    rocket_angle_to_normal_deg = acosf(constrain(rz, -1.0f, 1.0f)) * 180.0f / PI;

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
void tobytesSendTask(void *pv) {
  for (;;) {
    sent[0] = 'B';
    memcpy(&sent[1], &gps_irtifa, sizeof(float));  //gps irtifa
   // sent[5] = 'C';
    memcpy(&sent[5], &gps_enlem, sizeof(float));  //gps enlem
   // sent[10] = 'D';
    memcpy(&sent[9], &gps_boylam, sizeof(float));  //gps boylam
   // sent[15] = 'E';
    memcpy(&sent[13], &filtered_gx, sizeof(float));  //filtreli gyroX
    //sent[20] = 'F';
    memcpy(&sent[17], &filtered_gy, sizeof(float));  //filrteli gyroY
   // sent[25] = 'G';
    memcpy(&sent[21], &filtered_gz, sizeof(float));  //filtreli gyroZ
    //sent[30] = 'H';
    memcpy(&sent[25], &filtered_ax, sizeof(float));  //filtreli ivmeX
   // sent[35] = 'I';
    memcpy(&sent[29], &filtered_ay, sizeof(float));                 //filtreli ivmeY
    //sent[40] = 'J';
    memcpy(&sent[33], &filtered_az, sizeof(float));                 //filtreli ivmeZ
   // sent[45] = 'K';
    memcpy(&sent[37], &rocket_angle_to_normal_deg, sizeof(float));  //roket açısı
    //sent[50] = 'L';
    sent[41] = checksumHesaplama(sent, sizeof(sent) - 1);
    FixSerial.write((byte)0x00);
    FixSerial.write(41);
    FixSerial.write(50);
    FixSerial.write(sent, 42);
    vTaskDelay(pdMS_TO_TICKS(150));
  }
}
void printTask(void *pv) {
  for (;;) {
    if (gps.satellites.isValid()) {
      Serial.print("UYDU SAYİSİ: ");
      // Eğer uydu verisi geçerliyse, değeri yazdır.
      Serial.print(gps.satellites.value());
    }
    Serial.print("  | Roket Açısı: ");
    Serial.print(rocket_angle_to_normal_deg, 2);
    Serial.print("  | GPS irtifa: ");
    Serial.print(gps_irtifa);
    Serial.print(" m | Enlem: ");
    Serial.print(gps_enlem);
    Serial.print("  | Boylam: ");
    Serial.print(gps_boylam);
    Serial.print("  | GX: ");
    Serial.print(filtered_gx, 2);
    Serial.print("  | GY: ");
    Serial.print(filtered_gy, 2);
    Serial.print("  | GZ: ");
    Serial.print(filtered_gz, 2);
    Serial.print(" | AX: ");
    Serial.print(filtered_ax, 2);
    Serial.print(" | AY: ");
    Serial.print(filtered_ay, 2);
    Serial.print(" | AZ: ");
    Serial.println(filtered_az, 2);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
void setup() {
  Serial.begin(9600);
  FixSerial.begin(9600, SERIAL_8N1, A5, A4);
  GpsSerial.begin(9600, SERIAL_8N1, 2, 3);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  SPI.begin();
  if (!sox.begin_SPI(LSM_CS)) {
    Serial.println("LSM6DSOX bulunamadı.");
    while (1)
      ;
  }
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  initializeQuaternion();  // Kuaterniyonları başlat
  lastIMUUpdateTime = millis();

  xTaskCreate(GPSTask, "GPS", 4096, NULL, 2, NULL);
  xTaskCreate(imuTask, "IMU", 4096, NULL, 2, NULL);
  xTaskCreate(printTask, "Control", 4096, NULL, 1, NULL);
  xTaskCreate(tobytesSendTask, "Haberleşme", 8192, NULL, 2, NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
}
