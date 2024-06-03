#include <Arduino.h>
#include <math.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include "DFRobot_GDL.h"
#include "DFRobot_UI.h"
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define CMPS12_ADDRESS 0x60 // Addresse du CMPS12 shifté à droite de un bit pour la libraire wire
#define ANGLE_8 1           // Registre pour lire l'angle de 8bit

TinyGPSPlus gps;

// Batterie
#define VOLTAGE_PIN 34 // Broche ADC utilisée pour mesurer la tension de la batterie
#define VOLTAGE_DIVIDER_RATIO 2 // Ratio du diviseur de tension (R2 / (R1 + R2))
int percentage;

// Ecran define
#define TFT_DC 15
#define TFT_CS 21
#define TFT_RST 2

/**
 * @brief Constructor Constructor of hardware SPI communication
 * @param dc Command/data line pin for SPI communication
 * @param cs Chip select pin for SPI communication
 * @param rst reset pin of the screen
 */
DFRobot_ST7789_240x320_HW_SPI screen(/*dc=*/TFT_DC, /*cs=*/TFT_CS, /*rst=*/TFT_RST);
DFRobot_UI ui(&screen, NULL);

// Encodeur define
#define CLK 32 // A
#define DT 33 // B
#define SW 39 // SW

//Variables Encodeur
int counter = 1;
int currentStateCLK;
int lastStateCLK;
unsigned long lastButtonPress = 0;

// Variables CMPS12
unsigned char high_byte, low_byte, angle8;
unsigned int angle16;
uint16_t compass;

// Variables Acceleromètre
const int x_out = 35; // A0
const int y_out = 39; // A1
const int z_out = 36; // A2
int pitch;
int pitch_values[5];
int pitch_index = 0;
int pitch_moyen = 0;

// Variables Servo Motor Inclinaison
Servo servo_I; // Créer objet servo pour controler le servomoteur 
int servoPin_I = 14;
int val_I; // variable pour envoyer position au servomoteur

// Variables Servo Motor Orientation
Servo servo_O; // Créer objet servo pour controler le servomoteur
int servoPin_O = 12;
int val_O; // variable pour envoyer position au servomoteur

// Variables Inclinaison
float effect_I_Incl = 0;
unsigned long temps_precedent_I = 0;
unsigned long temps_precedent_accelerometre = 0;
unsigned long temps_actuel_I;
unsigned long diff_temps_I = 0;
signed int diff_erreur_I = 0;

const float KI_I = 0;//0.0000015;
const float KP_I = 10;

// Variable Orientation
float effect_I_O = 0;
unsigned long temps_precedent_O = 0;
unsigned long temps_actuel_O;
unsigned long diff_temps_O = 0;
signed int diff_erreur_O = 0;

const float KI_O = 0.0000015;
const float KP_O = 1;

const int OFFSET = 1500;

//Variables GPS
//uint8_t heure;
uint8_t minute;
uint8_t jour;
uint8_t mois;
uint8_t annee;

//Connexion WIFI
/* const char* ssid = "FAMEGA";
const char* password = "Fam3g@$1"; */
const char* ssid = "iPhone Santy";
const char* password = "Santy1910";

//URL pour API
String URL = "https://api.ipgeolocation.io/astronomy?";
String ApiKey = "07b72fbbfcb541ac81ef5a2915bd3368";

//Variable API
String lat = "45.508888";
String lng = "-73.561668";
String date;
String heure;
String coucher_sol;
String lever_sol;
float sun_azi;
float sun_alti;
float alti_aff;

//************ Fonctions ************//
void ecran(int menu)
{
  switch(menu){

    // Heure
    case 1:
      screen.setTextWrap(false);
      screen.setTextColor(COLOR_RGB565_WHITE);
      screen.setTextSize(1);
      screen.setFont(&FreeSans12pt7b);
      screen.setCursor(55, 140);
      screen.print(heure); 
      //screen.print(heure);
      /* screen.print(heure - 4); 
      screen.print(":"); 
      screen.print(minute); */
      screen.setTextSize(1);
      screen.setCursor(60, 190);
      screen.println(date);
      delay(50);
      break;

    // Coucher Lever
    case 2:
      screen.setTextWrap(false);
      screen.setTextColor(COLOR_RGB565_WHITE);
      screen.setTextSize(2);
      screen.setFont(&FreeSans12pt7b);
      screen.setCursor(30, 100);
      screen.println("Lever:"); 
      screen.setCursor(30, 140);
      screen.println(lever_sol); 
      screen.setCursor(30, 190);
      screen.println("Coucher:"); 
      screen.setTextSize(2);
      screen.setCursor(25, 230);
      screen.println(coucher_sol);
      delay(50);
      break;

    // Info Soleil
    case 3:
      screen.setTextWrap(false);
      screen.setTextColor(COLOR_RGB565_WHITE);
      screen.setTextSize(2);
      screen.setFont(&FreeSans12pt7b);
      screen.setCursor(30, 100);
      screen.println("Azimuth:"); 
      screen.setCursor(30, 140);
      screen.println(sun_azi); 
      screen.setCursor(30, 190);
      screen.println("Altitude:"); 
      screen.setTextSize(2);
      screen.setCursor(25, 230);
      screen.println(alti_aff);
      delay(50);
      break;
    
    // Battérie
    case 4:
      screen.setTextWrap(false);
      screen.setTextColor(COLOR_RGB565_WHITE);
      screen.setTextSize(3);
      screen.setFont(&FreeSans12pt7b);
      screen.setCursor(30, 140);
      screen.print(percentage); 
      screen.println("%");
      screen.setTextSize(2);
      screen.setCursor(30, 190);
      screen.println("Batterie");
      delay(50);
      break;

    // Mode Test
    case 5:
      screen.setTextWrap(false);
      screen.setTextColor(COLOR_RGB565_WHITE);
      screen.setTextSize(2);
      screen.setFont(&FreeSans12pt7b);
      screen.setCursor(6, 140);
      screen.println("Mode Test");
      screen.setTextSize(1);
      screen.setCursor(10, 190);
      screen.println("Appuyez pour activer");
      delay(50);
      break;
  }
  
}

float voltageToPercentage(float voltage) {
    // Table de correspondance tension-pourcentage
    float voltagePoints[] = {4.2, 4.1, 4.0, 3.9, 3.8, 3.7, 3.6};
    int percentagePoints[] = {100, 90, 80, 60, 40, 20, 0};

    // Trouver les deux points entre lesquels se situe la tension de la batterie
    int lowerIndex = -1;
    int upperIndex = -1;
    for (int i = 0; i < sizeof(voltagePoints) / sizeof(float); i++) {
        if (voltage >= voltagePoints[i]) {
            lowerIndex = i;
            break;
        }
    }
    if (lowerIndex == -1) {
        lowerIndex = 0;
        upperIndex = 1;
    } else if (lowerIndex == 0) {
        lowerIndex = sizeof(voltagePoints) / sizeof(float) - 2;
        upperIndex = sizeof(voltagePoints) / sizeof(float) - 1;
    } else {
        upperIndex = lowerIndex - 1;
    }

    // Interpolation linéaire entre les deux points
    float lowerVoltage = voltagePoints[lowerIndex];
    float upperVoltage = voltagePoints[upperIndex];
    int lowerPercentage = percentagePoints[lowerIndex];
    int upperPercentage = percentagePoints[upperIndex];

    // Calculer le pourcentage en utilisant l'interpolation linéaire
    int percentage = lowerPercentage + (upperPercentage - lowerPercentage) * (voltage - lowerVoltage) / (upperVoltage - lowerVoltage);
    return percentage;
}

void appelAPI_Soleil(){
  if (WiFi.status() == WL_CONNECTED) {

    HTTPClient http;

    //Set HTTP Request Final URL with Location and API key information
    http.begin(URL + "apiKey=" + ApiKey + "&lat=" + lat + "&long=" + lng);

    // start connection and send HTTP Request
    int httpCode = http.GET();

    // httpCode will be negative on error
    if (httpCode > 0) {

      //Read Data as a JSON string
      String JSON_Data = http.getString();
      Serial.println(JSON_Data);

      //Retrieve some information about the weather from the JSON format
      DynamicJsonDocument doc(2048);
      deserializeJson(doc, JSON_Data);
      JsonObject obj = doc.as<JsonObject>();

      Serial.println(obj);

      date = obj["date"].as<String>();
      sun_azi = obj["sun_azimuth"].as<float>();
      sun_alti = obj["sun_altitude"].as<float>();
      coucher_sol = obj["sunset"].as<String>();
      lever_sol = obj["sunrise"].as<String>();
      heure = obj["current_time"].as<String>();

      alti_aff = sun_alti;

      Serial.print("Azimuth: ");
      Serial.print(sun_azi);
      Serial.print("    Altitude: ");
      Serial.println(sun_alti);

      // Fonction conversion Altitude
      if (sun_alti < 70){
        sun_alti = 70;
      }
      if (sun_alti > 115){
        sun_alti > 115;
      }

    } else {
      Serial.println("Error!");
    }

    http.end();

  }
}

/* void displayInfo()
{
  //Serial.print(F("Location: "));
  if (gps.location.isValid()){
    //Serial.print(gps.location.lat(), 6);
    //Serial.print(F(","));
    //Serial.print(gps.location.lng(), 6);
    heure = gps.time.hour();
    minute = gps.time.minute();
    jour = gps.date.day();
    mois = gps.date.month();
    annee = gps.date.year();

    if(heure < 4){
      heure = 24 + heure;
    }
    if(heure == 4){
      Serial.print(F("0"));
    }
    Serial.print(heure - 4);
    Serial.print(F(":"));
    if (minute < 10){
      Serial.print(F("0"));
    }
    Serial.println(minute);
    Serial.print(jour);
    Serial.print(mois);
    Serial.println(annee);
  }
  else
  {
    Serial.print(F("INVALID"));
  }
} */


void setup(void){
  Serial.begin(9600);                 /*Set the baudrate to 9600*/
  Serial2.begin(9600);
  Wire.begin();
	// Allocation de tous les timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	servo_I.setPeriodHertz(50);			  // standard 50 hz servo
	servo_I.attach(servoPin_I, 1300, 1700); // attache le servo à la pin 13
  servo_O.setPeriodHertz(50);			  // standard 50 hz servo
	servo_O.attach(servoPin_O, 1300, 1700); // attache le servo à la pin 13

  screen.begin();
  screen.fillScreen(COLOR_RGB565_BLACK); // Clear screen
  // Cadre Écran
  screen.drawRoundRect(/*x0=*/0, /*y0=*/0, /*w=*/screen.width() - 3, /*h=*/screen.height() - 3, /*radius=*/20, /*color=*/COLOR_RGB565_DGRAY);
  screen.fillRect(/*x=*/0, /*y=*/270, /*w=*/screen.width() - 3, /*h=*/screen.height() - 270, /*color=*/COLOR_RGB565_DGRAY);
  screen.setFont(&FreeMonoBoldOblique12pt7b);
  screen.setTextColor(COLOR_RGB565_GREEN);
  screen.setTextSize(1);
  screen.setCursor(13, 150);
  screen.print("WIFI Connection");
  screen.setTextSize(1);
  screen.setCursor(13, 300);
  screen.print("Tracker Solaire");
  // Encodeur Pinout
	pinMode(CLK,INPUT);
	pinMode(DT,INPUT);
	pinMode(SW, INPUT_PULLUP);
  // Lire état CLK
	lastStateCLK = digitalRead(CLK);

  // We start by connecting to a WiFi network
  WiFi.begin(ssid, password);
  screen.setCursor(100, 170);
  int i = 0;
  // Tant que le WIFI n'est pas connecté
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    screen.print(".");
    i++;
    if (i > 3){
      screen.fillRect(/*x=*/100, /*y=*/160, /*w=*/55, /*h=*/25, /*color=*/COLOR_RGB565_BLACK);
      screen.setCursor(100, 170);
      i = 0;
    }
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  screen.fillRect(/*x=*/10, /*y=*/130, /*w=*/215, /*h=*/70, /*color=*/COLOR_RGB565_BLACK);
  screen.setCursor(13, 150);
  screen.println("WIFI Connected!");
  // wait for WiFi connection
  appelAPI_Soleil();
  
  temps_actuel_I = micros(); //Compter les microsecondes pour le calcul de différence de temps
  temps_precedent_I = temps_actuel_I; //Temps précedent prends valeur de temps actuel
  temps_precedent_accelerometre = temps_actuel_I;
}

void loop() {
  int x_adc_value, y_adc_value, z_adc_value; 
  double x_g_value, y_g_value, z_g_value;
  // Tant que GPS connecté
  /* while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      displayInfo();
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  } */

  // Lire la tension de la batterie à partir de l'ADC
  int rawValue = analogRead(VOLTAGE_PIN);
  float voltage = rawValue * (3.6 / 4095) * VOLTAGE_DIVIDER_RATIO;
  // Convertir la tension en pourcentage
  percentage = voltageToPercentage(voltage);

  temps_actuel_I = micros(); //Compter les microsecondes pour le calcul de différence de temps

  // Fonction pour moyenne glissante de l'accelerometre
  if (temps_actuel_I - temps_precedent_accelerometre > 10000) {
    temps_precedent_accelerometre = temps_actuel_I; //Temps précedent prends valeur de temps actuel

    x_adc_value = analogRead(x_out); /* Digital value of voltage on x_out pin */ 
    y_adc_value = analogRead(y_out); /* Digital value of voltage on y_out pin */ 
    z_adc_value = analogRead(z_out); /* Digital value of voltage on z_out pin */ 

    x_g_value = ( ( ( (double)(x_adc_value * 3.6)/4096) - 1.53 ) / 0.330 ); /* Acceleration in x-direction in g units */ 
    y_g_value = ( ( ( (double)(y_adc_value * 3.6)/4096) - 1.53 ) / 0.330 ); /* Acceleration in y-direction in g units */ 
    z_g_value = ( ( ( (double)(z_adc_value * 3.6)/4096) - 1.53 ) / 0.330 ); /* Acceleration in z-direction in g units */ 
    
    pitch = ( (atan2(z_g_value,x_g_value) * 180) / 3.14 ); /* Formula for pitch */

    pitch_values[pitch_index] = pitch;
    pitch_index++;
    if (pitch_index >= 5) {
      pitch_index = 0;
    }
  }

  if (temps_actuel_I - temps_precedent_I > 50000) {
    diff_temps_I = temps_actuel_I - temps_precedent_I; //Calcul et affichage sur terminal de la différence de temps
    temps_precedent_I = temps_actuel_I; //Temps précedent prends valeur de temps actuel

    pitch_moyen = 0;
    for (int i = 0; i < 5 ; i++) {
      pitch_moyen = pitch_moyen + pitch_values[i];
    }
    pitch_moyen = pitch_moyen / 5;

    signed int erreur_I = sun_alti - pitch_moyen; //Calcul de l'erreur et afficher sur terminal

    diff_erreur_I = diff_temps_I * erreur_I; //Calcul de la différence d'erreur en multipliant la diff de temps par l'erreur
    effect_I_Incl = effect_I_Incl + diff_erreur_I; //Calcul et affichage sur terminal de l'effet I

    signed int effet_p_I = (erreur_I * KP_I); //Calcul de l'effet P
    unsigned int input_I = effet_p_I + KI_I * effect_I_Incl + OFFSET; //Calcul de la valeur qu'on envoie au servomoteur
    val_I = input_I;

    servo_I.write(val_I); // Position du servomoteur selon la valeur qu'on lui input

    /* Serial.print("Pitch: ");
    Serial.print(pitch_moyen);
    Serial.print("  Alti: ");
    Serial.println(sun_alti);
    
    Serial.print("    Erreur: "); 
    Serial.print(erreur_I, DEC);

    Serial.print("    Diff Temps: "); 
    Serial.print(diff_temps_I, DEC);
    
    Serial.print("    Diff Erreur: "); 
    Serial.print(diff_erreur_I, DEC); // Affichage sur terminal

    Serial.print("    Effect_I: "); 
    Serial.print(effect_I_Incl);

    Serial.print("    KI * effect_I: "); //Affichage sur terminal de l'effet I fois KI
    Serial.print(KI_I * effect_I_Incl, DEC);

    Serial.print("    Effect P: "); //Affichage sur terminal de l'effet P
    Serial.print(effet_p_I, DEC);

    Serial.print("    input: "); //Affichage sur terminal de la valeur qu'on envoie au servomoteur
    Serial.println(input_I, DEC); */
    
  }

  Wire.beginTransmission(CMPS12_ADDRESS); // Commence la communication avec le CMPS12
	Wire.write(ANGLE_8);					// Envoie le registre qu'on veut lire
	Wire.endTransmission();
	Wire.requestFrom(CMPS12_ADDRESS, 5); //Nous donnes les données qu'on veut lire du CMPS12

	while (Wire.available() < 5); // Attend pour que tout les bits soient recus

  temps_actuel_O = micros(); //Compter les microsecondes pour le calcul de différence de temps

	angle8 = Wire.read(); // Lire les 5 bits
	high_byte = Wire.read();
	low_byte = Wire.read();

	angle16 = high_byte; // Calculer l'angle de 16 bit
	angle16 <<= 8;
	angle16 += low_byte;

	compass = angle16 / 10; //Calcul de l'angle sur 360 et afficher sur terminal
  signed int erreur_O = compass - sun_azi; //Calcul de l'erreur et afficher sur terminal
  
  diff_temps_O = temps_actuel_O - temps_precedent_O; //Calcul et affichage sur terminal de la différence de temps
  diff_erreur_O = diff_temps_O * erreur_O; //Calcul de la différence d'erreur en multipliant la diff de temps par l'erreur
  effect_I_O = effect_I_O + diff_erreur_O; //Calcul et affichage sur terminal de l'effet I

  if (effect_I_O > (100 / KI_O)){ //Si effet I est plus grand que 100
		effect_I_O = (100/ KI_O); //Effet I égal 100
	}
	else if (effect_I_O < (-100 / KI_O)){ //Si effet I est plus grand que -100
		effect_I_O = (-100 / KI_O); //Effet I égal -100
	}

	signed int effet_p = (erreur_O * KP_O); //Calcul de l'effet P
	unsigned int input_O = effet_p + KI_O * effect_I_O + OFFSET; //Calcul de la valeur qu'on envoie au servomoteur
	val_O = input_O;
  servo_O.write(val_O); // Position du servomoteur selon la valeur qu'on lui input

  /* Serial.print("Compass: "); 
  Serial.print(compass, DEC);
  Serial.print("  Azi: ");
  Serial.println(sun_azi); */

  temps_precedent_O = temps_actuel_O; //Temps précedent prends valeur de temps actuel

  // Read the current state of CLK
	currentStateCLK = digitalRead(CLK);
	// If last and current state of CLK are different, then pulse occurred
	// React to only 1 state change to avoid double count
	if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
		// If the DT state is different than the CLK state then
		if (digitalRead(DT) != currentStateCLK) {
			counter ++;
			if (counter > 5){
				counter = 1;
			}
		} else {
			counter --;
      if (counter < 1){
				counter = 5;
			}
		}
    screen.fillRect(/*x=*/2, /*y=*/20, /*w=*/screen.width() - 7, /*h=*/screen.height() - 80, /*color=*/COLOR_RGB565_BLACK);
    ecran(counter);
	}
	// Remember last CLK state
	lastStateCLK = currentStateCLK;
	delay(1);
}