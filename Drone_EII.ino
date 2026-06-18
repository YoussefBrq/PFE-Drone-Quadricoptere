#include <Wire.h>
#include <Servo.h>
#include <math.h>

// axes
#define ROLL   0
#define PITCH  1
#define YAW    2

// periode boucle 250Hz
const unsigned long PERIODE_US = 4000UL;
const float DT = 0.004f;

// adresse MPU6050 et registres
#define MPU_ADRESSE    0x68
#define REG_PWR_MGMT   0x6B
#define REG_DLPF       0x1A
#define REG_GYRO_CFG   0x1B
#define REG_ACCEL_CFG  0x1C
#define REG_ACCEL_XOUT 0x3B
#define REG_GYRO_XOUT  0x43

// limites ESC en microsecodes
const int ESC_ARRET = 1000;
const int ESC_MIN   = 1120;
const int ESC_MAX   = 1300;
const int THROTTLE  = 1160;

Servo M1, M2, M3, M4;

// variables accelerometre
float accX, accY, accZ;
float accAngleRoll  = 0.0f;
float accAnglePitch = 0.0f;

// variables gyroscope
float vitRoll, vitPitch, vitYaw;
float biaisGyro[3] = {0.0f, 0.0f, 0.0f};

// angles filtres (filtre complementaire)
float angleFiltre[2] = {0.0f, 0.0f};

// gains PID
float Kp[2] = {0.20f, 0.20f};
float Ki[2] = {0.01f, 0.01f};
float Kd[2] = {0.005f, 0.005f};

// variables internes PID
float erreur[2]         = {0.0f, 0.0f};
float erreurPrev[2]     = {0.0f, 0.0f};
float erreurIntegral[2] = {0.0f, 0.0f};
float sortie[2]         = {0.0f, 0.0f};
float consigne[2]       = {0.0f, 0.0f};

// commandes moteurs
int cmdMoteur[4] = {ESC_ARRET, ESC_ARRET, ESC_ARRET, ESC_ARRET};

unsigned long timerBoucle = 0;

// prototypes
void  initMPU();
void  calibrationGyro();
void  lectureIMU();
void  filtreComplementaire();
void  calculPID();
void  mixageMoteurs();
void  arreterMoteurs();
void  monteeProg();
float limiter(float val, float minVal, float maxVal);


void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  initMPU();

  M1.attach(4, 1000, 2000);
  M2.attach(5, 1000, 2000);
  M3.attach(6, 1000, 2000);
  M4.attach(7, 1000, 2000);

  // armement ESC
  arreterMoteurs();
  delay(6000);

  Serial.println("calibration gyro... ne pas bouger");
  calibrationGyro();
  Serial.println("calibration ok");

  monteeProg();
  delay(2000);

  Serial.println("PID demarre");
  timerBoucle = micros();
}


void loop() {
  lectureIMU();
  filtreComplementaire();
  calculPID();
  mixageMoteurs();

  Serial.print("Roll: ");  Serial.print(angleFiltre[ROLL], 2);
  Serial.print("  Pitch: "); Serial.println(angleFiltre[PITCH], 2);

  while (micros() - timerBoucle < PERIODE_US);
  timerBoucle = micros();
}


// initialisation MPU6050 via registres I2C directs
// on active le DLPF a 44Hz pour filtrer les vibrations moteurs
// plage gyro : +/-500 deg/s  |  plage accel : +/-8g
void initMPU() {
  Wire.beginTransmission(MPU_ADRESSE);
  Wire.write(REG_PWR_MGMT);
  Wire.write(0x00);  // sortie du mode veille
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADRESSE);
  Wire.write(REG_DLPF);
  Wire.write(0x03);  // filtre passe-bas 44Hz
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADRESSE);
  Wire.write(REG_ACCEL_CFG);
  Wire.write(0x10);  // +/-8g
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADRESSE);
  Wire.write(REG_GYRO_CFG);
  Wire.write(0x08);  // +/-500 deg/s
  Wire.endTransmission();

  delay(250);
}


// calcul du biais gyroscope sur 3000 echantillons drone immobile
void calibrationGyro() {
  const int N = 3000;

  for (int i = 0; i < N; i++) {
    Wire.beginTransmission(MPU_ADRESSE);
    Wire.write(REG_GYRO_XOUT);
    Wire.endTransmission();
    Wire.requestFrom(MPU_ADRESSE, 6);

    int16_t gxB = Wire.read() << 8 | Wire.read();
    int16_t gyB = Wire.read() << 8 | Wire.read();
    int16_t gzB = Wire.read() << 8 | Wire.read();

    biaisGyro[ROLL]  += (float)gxB / 65.5f;
    biaisGyro[PITCH] += (float)gyB / 65.5f;
    biaisGyro[YAW]   += (float)gzB / 65.5f;

    delay(1);
  }

  biaisGyro[ROLL]  /= N;
  biaisGyro[PITCH] /= N;
  biaisGyro[YAW]   /= N;
}


// lecture accelerometre et gyroscope par acces direct aux registres
// formule angle accel a deux axes pour eviter singularite (amelioration vs promotion precedente)
void lectureIMU() {

  // accelerometre
  Wire.beginTransmission(MPU_ADRESSE);
  Wire.write(REG_ACCEL_XOUT);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADRESSE, 6);

  int16_t axB = Wire.read() << 8 | Wire.read();
  int16_t ayB = Wire.read() << 8 | Wire.read();
  int16_t azB = Wire.read() << 8 | Wire.read();

  accX = (float)axB / 4096.0f;
  accY = (float)ayB / 4096.0f;
  accZ = (float)azB / 4096.0f;

  accAngleRoll  = atan2f(accY, sqrtf(accX*accX + accZ*accZ)) * (180.0f / PI);
  accAnglePitch = atan2f(accX, sqrtf(accY*accY + accZ*accZ)) * (180.0f / PI);

  // gyroscope
  Wire.beginTransmission(MPU_ADRESSE);
  Wire.write(REG_GYRO_XOUT);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADRESSE, 6);

  int16_t gxB = Wire.read() << 8 | Wire.read();
  int16_t gyB = Wire.read() << 8 | Wire.read();
  int16_t gzB = Wire.read() << 8 | Wire.read();

  vitRoll  = (float)gxB / 65.5f - biaisGyro[ROLL];
  vitPitch = (float)gyB / 65.5f - biaisGyro[PITCH];
  vitYaw   = (float)gzB / 65.5f - biaisGyro[YAW];
}


// filtre complementaire : 98% gyro + 2% accelerometre
// le gyro est reactif mais derive, l accel est stable mais bruite
void filtreComplementaire() {
  angleFiltre[ROLL]  = 0.98f * (angleFiltre[ROLL]  + vitRoll  * DT) + 0.02f * accAngleRoll;
  angleFiltre[PITCH] = 0.98f * (angleFiltre[PITCH] + vitPitch * DT) + 0.02f * accAnglePitch;
}


// correcteur PID sur Roll et Pitch
// correction bug promotion precedente : integral = += erreur*DT (pas += integral)
void calculPID() {
  for (int axe = ROLL; axe <= PITCH; axe++) {
    erreur[axe] = consigne[axe] - angleFiltre[axe];

    erreurIntegral[axe] += erreur[axe] * DT;
    erreurIntegral[axe]  = limiter(erreurIntegral[axe], -50.0f, 50.0f);  // anti-windup

    float deltaErr = (erreur[axe] - erreurPrev[axe]) / DT;
    erreurPrev[axe] = erreur[axe];

    sortie[axe] = Kp[axe] * erreur[axe]
                + Ki[axe] * erreurIntegral[axe]
                + Kd[axe] * deltaErr;

    sortie[axe] = limiter(sortie[axe], -100.0f, 100.0f);
  }
}


// mixage configuration X
// M1/M2 ont des offsets pour compenser le desequilibre de poussee mesure
void mixageMoteurs() {
  cmdMoteur[0] = (int)(THROTTLE - sortie[ROLL] + sortie[PITCH] + 25);
  cmdMoteur[1] = (int)(THROTTLE - sortie[ROLL] - sortie[PITCH] + 10);
  cmdMoteur[2] = (int)(THROTTLE + sortie[ROLL] + sortie[PITCH]);
  cmdMoteur[3] = (int)(THROTTLE + sortie[ROLL] - sortie[PITCH]);

  for (int i = 0; i < 4; i++)
    cmdMoteur[i] = (int)limiter((float)cmdMoteur[i], (float)ESC_MIN, (float)ESC_MAX);

  M1.writeMicroseconds(cmdMoteur[0]);
  M2.writeMicroseconds(cmdMoteur[1]);
  M3.writeMicroseconds(cmdMoteur[2]);
  M4.writeMicroseconds(cmdMoteur[3]);
}


void arreterMoteurs() {
  M1.writeMicroseconds(ESC_ARRET);
  M2.writeMicroseconds(ESC_ARRET);
  M3.writeMicroseconds(ESC_ARRET);
  M4.writeMicroseconds(ESC_ARRET);
}


// montee progressive avant de lancer le PID
void monteeProg() {
  for (int v = ESC_ARRET; v <= ESC_MIN; v += 5) {
    M1.writeMicroseconds(v + 25);
    M2.writeMicroseconds(v + 10);
    M3.writeMicroseconds(v);
    M4.writeMicroseconds(v);
    delay(100);
  }
}


float limiter(float val, float minVal, float maxVal) {
  if (val < minVal) return minVal;
  if (val > maxVal) return maxVal;
  return val;
}
