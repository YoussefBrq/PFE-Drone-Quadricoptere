// La capteur MPU6050 contient à la fois un accéléromètre 3 axes et un gyroscope 3 axes qui communique via i2c
// il faut donc installer les bibliothèque "i2c", "MPU6050" et "math.h" pour pouvoir utiliser la fonction "arctan"
#include "Wire.h"  // Arduino Wire library
#include "I2Cdev.h"  //Installer ces 2 librairies
#include "MPU6050.h"
#include "math.h"

MPU6050 accelgyro;

int16_t ax, ay, az;  //mesures brutes du accélorémètre
int16_t gx, gy, gz;   //mesures brutes du gyroscope

#define Yaw      0
#define Pitch    1
#define Roll     2

int CommandeESC1,CommandeESC2,CommandeESC3,CommandeESC4;
int throttle=1600;
int Temps,t0;
int PeriodeControle=4000;  // Fs = 250Hz : on envoie les impulsions toutes les 4000µs


float  kp[3]={2,2,2}, ki[3]={0.1,0.1,0.1}, kd[3]={1,1,1}, AngleMesure[3]={}, Consigne[3]={0,0,0}, CommandeCalculee[3]={};
float  Erreur[3]={}, ErreurIntegral[3]={}, DeltaErreur[3]={}, ErreurPrecedent[3]={};

void setup() {
  Wire.begin();  // bus I2C
  accelgyro.initialize();  // initialisation du composant
  DDRD=0xFF;// initialisation du port D en mode sortie
  Serial.begin(9600);
    for (int i=0; i<1000;i++)
  InitialisationControle(800);
    for (int i=0; i<1000;i++)
  InitialisationControle(1200);
}

void loop() {   
  while ((Temps= micros()) - t0 < PeriodeControle);

    // enregistre le temps de commencement
   t0 = Temps;

    // On active les broches 4, 5, 6 et 7 (état HIGH)
   PORTD |= B11110000;

    // On boucle tant que toutes les sorties ne sont pas retournées à l'état LOW
   while (PORTD >= 16) {
        Temps = micros();
        if ((Temps-t0) >= CommandeESC1) PORTD &= B11101111; // on met la broche 4 à LOW
        if ((Temps-t0) >= CommandeESC2) PORTD &= B11011111; // on met la broche 5 à LOW
        if ((Temps-t0) >= CommandeESC3) PORTD &= B10111111; // on met la broche 6 à LOW
        if ((Temps-t0) >= CommandeESC4) PORTD &= B01111111; // on met la broche 7 à LOW
    }
  AnglesMesuees();
  CalculCommandePID();
  Serial.println(CommandeESC1);
  
}

void AnglesMesuees() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  AngleMesure[Yaw]=0.98*(AngleMesure[Yaw]+float(gx)*0.01/131) + 0.02*atan2((double)ay,(double)az)*180/PI;
  AngleMesure[Pitch]=0.98*(AngleMesure[Pitch]+float(gy)*0.01/131) + 0.02*atan2((double)ax,(double)az)*180/PI;
  AngleMesure[Roll]=0.98*(AngleMesure[Roll]+float(gz)*0.01/131) + 0.02*atan2((double)ay,(double)ax)*180/PI;
}

void InitialisationControle(int Rcy) {
   // Fs = 250Hz : on envoie les impulsions toutes les 4000µs

    // enregistre le temps de commencement
    t0 = Temps;

    // On active les broches 4, 5, 6 et 7 (état HIGH)
    PORTD |= B11110000;

    // On boucle tant que toutes les sorties ne sont pas retournées à l'état LOW
    while (PORTD >= 16) {
        Temps = micros();
        if ((Temps-t0) >= Rcy) PORTD &= B11101111; // on met la broche 4 à LOW
        if ((Temps-t0) >= Rcy) PORTD &= B11011111; // on met la broche 5 à LOW
        if ((Temps-t0) >= Rcy) PORTD &= B10111111; // on met la broche 6 à LOW
        if ((Temps-t0) >= Rcy) PORTD &= B01111111; // on met la broche 7 à LOW
    }
    while ((Temps= micros()) - t0 < PeriodeControle);
    
}
/*
 * CalculConsigne: c'est la fonction qui calcule la consigne d'un axe en degré/sec.
 * elle a deux paramètres:(Angle,LargeImpuls)
 * Angle mesuré en degré sur l'axe (X, Y, Z)
 * LargeImpuls : Durée en µs de l'impulsion reçue pour l'axe  (comprise entre 1000µs et 2000µs)
 */
float CalculConsigne(float Angle, int LargeImpuls) {
    float Niveau = Angle * 15;
    float Consigne= 0;

    // On s'accorde une marge 16µs pour de meilleurs résultats
    if (LargeImpuls > 1508) {
        Consigne = LargeImpuls - 1508;
    } else if (LargeImpuls <  1492) {
        Consigne = LargeImpuls - 1492;
    }
    Consigne -= Niveau;
    Consigne /= 3;
    return Consigne;
}

float minmax(float x,float min, float max)
{
  if (x > max) {
    x = max;
  }
  if (x < min) {
    x = min;
  }
  return x;
}
 // c'est la fonction qui calcule les erreurs des trois axes et en déduire les commandes des moteurs.
void CalculCommandePID() {

    // Calcul des erreurs courantes
    Erreur[Yaw]   = Consigne[Yaw]- AngleMesure[Yaw];
    Erreur[Pitch] = Consigne[Pitch] - AngleMesure[Pitch];
    Erreur[Roll]  = Consigne[Roll] - AngleMesure[Roll] ;

    // Calcul des sommes d'erreurs : composante intégrale
    ErreurIntegral[Yaw]   += ErreurIntegral[Yaw] ;
    ErreurIntegral[Pitch] += ErreurIntegral[Pitch];
    ErreurIntegral[Roll] += ErreurIntegral[Roll];

    // on s'assure que ∫e.Ki ne dépasse pas 400
    ErreurIntegral[Yaw]   = minmax(ErreurIntegral[Yaw],   -400/ki[Yaw],   400/ki[Yaw]);
    ErreurIntegral[Pitch] = minmax(ErreurIntegral[Pitch], -400/ki[Pitch], 400/ki[Pitch]);
    ErreurIntegral[Roll]  = minmax(ErreurIntegral[Roll],  -400/ki[Roll],  400/ki[Roll]);

    // Calcul de la dérivée d'erreur : composante dérivée
    DeltaErreur[Yaw]   = Erreur[Yaw]  - ErreurPrecedent[Yaw] ;
    DeltaErreur[Pitch] = Erreur[Pitch] - ErreurPrecedent[Pitch];
    DeltaErreur[Roll] = Erreur[Roll] - ErreurPrecedent[Roll];

    // L'erreur courante devient l'erreur précédente pour le prochain tour de boucle
    ErreurPrecedent[Yaw]   = Erreur[Yaw];
    ErreurPrecedent[Pitch] = Erreur[Pitch];
    ErreurPrecedent[Roll]  = Erreur[Roll];

    // Calcul de la commande du PID (e.Kp + ∫e.Ki + Δe.Kd) pour chacun des axes 
    CommandeCalculee[Yaw]   = (kp[Yaw] * Erreur[Yaw]) + (ki[Yaw]* ErreurIntegral[Yaw]) + (kd[Yaw]* DeltaErreur[Yaw]);
    CommandeCalculee[Pitch] = (kp[Pitch] * Erreur[Pitch]) + (ki[Pitch]* ErreurIntegral[Pitch] ) + (kd[Pitch]* DeltaErreur[Pitch]);
    CommandeCalculee[Roll]  = (kp[Roll] * Erreur[Roll]) + (ki[Roll]* ErreurIntegral[Roll]) + (kd[Roll]* DeltaErreur[Roll]);

    // Calcul de la largeur de l'impulsion à envoyer pour chaque ESC
    CommandeESC1 = throttle - CommandeCalculee[Yaw]  - CommandeCalculee[Pitch] + CommandeCalculee[Roll];
    CommandeESC2 = throttle + CommandeCalculee[Yaw]  - CommandeCalculee[Pitch] - CommandeCalculee[Roll];
    CommandeESC3 = throttle - CommandeCalculee[Yaw]  + CommandeCalculee[Pitch] - CommandeCalculee[Roll];
    CommandeESC4 = throttle + CommandeCalculee[Yaw]  + CommandeCalculee[Pitch] + CommandeCalculee[Roll];
}