/*
 * Arduino StegerBräu 0.1
 * DIGITAL:
 * 1  - Speaker
 * 2  - Rotary Dreh
 * 3  - Rotary Dreh
 * 4  - Rotary Button
 * 5  - Feuer Sensor
 * 6  - Magnetventil
 * 7  - Temperatur I
 * 8  - GFA
 * 10 - Motor 100%
 * 11 - Motor 50%
 * 12 - 
 * 13 - 
 * 
 * ANALOG:
 * A0 - Gas Sensor
 * A4 - Display
 * A5 - Display
 * 
 * Temperaturfühler 
 * PINS: D7
 * 
 * Rotary Encoder
 * PINS: D2,D3 - D4
 * 
 * Rührwerk
 * PINS: D10,D11
 * 
 * Display 16x2 oder 20x4
 * PINS: GND, 5V, A5, A4
 * 
 * REALIS FÜR GAS
 * PINS: GND, 5V, D6
 *  
 * Servo f. Flammen Regulierung
 * PINS: D8
 * 
 */
//#include <rotary.h>             // RotaryEncoder
#include <OneWire.h>            // OneWire-Bibliothek einbinden TEMP
#include <DallasTemperature.h>  // DS18B20-Bibliothek einbinden TMEP
#include <LiquidCrystal_I2C.h>  // Display
// Drehknopf
#define DIR_CCW 0x10
#define DIR_CW 0x20
//Speaker
#include "pitches.h";
//Display:
#define I2C_ADDR    0x27  // Define I2C Address where the PCF8574A is
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7
LiquidCrystal_I2C  lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
String myTxt;
/*
 * 0 = Hauptmenü
 * 1 = Setup
 * 2 = Programm
 * 3 = Manuell
 */
int state = 0;                // Aktueller Programmteil
int stateAlt = -1;            // Alter Status

int Speaker = 1;              // Speaker

int Rotary1 = 2;              // Rotary Drehknopf
int Rotary2 = 3;              // Rotary Drehknopf
int RotaryButton = 4;         // Rotary Button
int rAlt = 0;                 // Rotary PressStatus Alt
int rWert = 0;                // Rotary DrehWert

int FeuerSensor = 5;          // Feuer Sensor

int Magnetventil = 6;         // Magnetventil

int GFA = 8;                  // Gas Feuerungs Automat GFA

int Motor1 = 10;              // Motor 1
int Motor2 = 11;              // Motor 2

int GasSensor = A0;           // GAS Sensor

unsigned long minute = 60000;             // 1 Minute

/*
 *  Rezepte: array{zeit, grad, modus}
 *  Modus: 1: Heizen, 2: Rasten, 3: Farbmalz zugeben 4: Hopfen zugeben
 */
String arten[4] = {"Heizen","Rasten","Farbmalz zugeben","Hopfen zugeben"};
int programmSetup[20][3];
int setupAnz = 0;
int programmDunkles[6][3] = {{0,38,1},{40,50,2},{30,64,2},{20,72,2},{10,72,3},{0,78,2}};
int programmHelles[3][3] = {{0,38,1},{40,50,2},{30,64,2}};

#define DS18B20_PIN 7           // TemperaturPIN D7
OneWire oneWire(DS18B20_PIN);          // OneWire Referenz setzen TEMP
DallasTemperature sensors(&oneWire);   // DS18B20 initialisieren TEMP

void setup() {
  lcd.begin(20,4);
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home();
  lcd.print("Brauhelfer v2.0");
  lcd.setCursor(0,1);
  lcd.print("Initialisiere...");
  lcd.setCursor(0,3);
  lcd.print("[15.05.16] by  Hansi");
  
  Serial.begin(9600);
  pinMode(Rotary1, INPUT);  // Drehknopf initialisieren
  pinMode(Rotary2, INPUT);  // Drehknopf initialisieren
  digitalWrite(Rotary1, HIGH);
  digitalWrite(Rotary2, HIGH);

  pinMode(FeuerSensor, INPUT);  // Feuer Sensor

  pinMode(Magnetventil, OUTPUT); // Magnetventil initalisieren
  digitalWrite(Magnetventil,HIGH);
  
  //Motor über Relais
  pinMode(Motor1, OUTPUT);    // Motor 100% initialisieren
  digitalWrite(Motor1,HIGH);
  pinMode(Motor2, OUTPUT);    // Motor 50% initialisieren
  digitalWrite(Motor2,HIGH);
  
  pinMode(GFA, OUTPUT);    // GFA initialisieren
  digitalWrite(GFA,HIGH);
  
  pinMode(RotaryButton, INPUT);      // Button initialisieren
  digitalWrite(RotaryButton, HIGH);  // Button aktivieren

  pinMode(Speaker,OUTPUT);   // Speaker initialisieren
  digitalWrite(Speaker, HIGH);
  delay(500);
  digitalWrite(Speaker, LOW);

  sensors.begin();  // DS18B20 Temp starten
}

int menuAkt = 0;
int menu[] = {0,1,2,3};
int menuSize = sizeof(menu) / sizeof(int);

void loop() {
  if(state != stateAlt){
    stateAlt = state;
    switch(state){
      case 1: state = loadSetup(); break;
      case 2: state = loadProgramm(1); break;
      case 3: state = loadManuell(); break;
      case 4: state = showTemp(); break;
      default: state = loadMenu(); break;
    }
  }
  delay(1000);
}

/*
 * Hauptmenü
 */
boolean menuTrigger = false;
int loadMenu(){
  // 0: Setup starten
  // 1: Rezepte
  // 2: Manueller Modus
  // 3: Temperatur zeigen
  int menuAlt = 0;
  changeMenu(0);
  while(1){
    int rotary = getRotary();
    if(rotary){
      changeMenu(rotary);
    }

    if(pushed() && menuTrigger){
      menuAkt++;
      return menuAkt;
    }
    menuTrigger = true;
  }
  return 0;
}

void changeMenu(int s){
  if(!s)
    showMenu(0);
  else{
    if(s < 0)
      menuAkt--;
    else
      menuAkt++;
    
    if(menuAkt > menuSize)
      menuAkt = 0;
    else if(menuAkt < 0)
      menuAkt = menuSize;
    showMenu(menuAkt);
  }
}


byte arrow[8] = {
  B00000,
  B11000,
  B01100,
  B00110,
  B01100,
  B11000,
  B00000,
};
void showMenu(int s){
  lcd.createChar(0, arrow);
  lcd.clear();
  lcd.setCursor(2,0); lcd.print("Setup starten");
  lcd.setCursor(2,1); lcd.print("Rezepte");
  lcd.setCursor(2,2); lcd.print("Manueller Modus");
  lcd.setCursor(2,3); lcd.print("Temperatur");  
  
  switch(s){
    case 1: lcd.setCursor(0,1); lcd.write(byte(0)); break;
    case 2: lcd.setCursor(0,2); lcd.write(byte(0)); break;
    case 3: lcd.setCursor(0,3); lcd.write(byte(0)); break;
    default: lcd.setCursor(0,0); lcd.write(byte(0)); break;
  }
  menuAkt = s;
}
 
/*
 * Setup Starten
 */
int loadSetup(){
  menuTrigger = false;
  for(int j=0;j < 20; j++){
    lcd.clear();
    myTxt = String("Setup - Schritt: ") + (setupAnz+1);
    lcd.print(myTxt);
    for(int i=0;i < 4; i++){
      switch(i){
        case 0: programmSetup[setupAnz][i] = setTime(); break;
        case 1: programmSetup[setupAnz][i] = setTemp(); break;
        case 2: programmSetup[setupAnz][i] = setArt(); break;
        case 3: ((setFertig())?loadProgramm(0):1); break;
      }
    }
    setupAnz++;
  }
  loadProgramm(0);
  return 0;
}

int artAktuell = 0;
int artSize = 3;
int setArt(){
  boolean dome = true;
  showArt(0);
  while(dome){
    int r = getRotary();
    if(r)
      showArt(r);
    if(pushed())
      dome = false; 
  }
  return (artAktuell+1);
}

void showArt(int r){
//  1: Heizen, 2: Rasten, 3: Farbmalz zugeben 4: Hopfen zugeben
  if(r > 0){
    artAktuell++;
  }else if(r < 0){
    artAktuell--;
  }else{
    artAktuell = r;
  }
  if(artAktuell < 0)
    artAktuell = 0;
  else if(artAktuell > artSize)
    artAktuell = artSize;
  //Serial.println(artAktuell);
  lcd.createChar(0, arrow);
  lcd.clear();
  lcd.setCursor(0,artAktuell); 
  lcd.write(byte(0));
  lcd.setCursor(2,0); lcd.print("Heizen");
  lcd.setCursor(2,1); lcd.print("Rasten");
  lcd.setCursor(2,2); lcd.print("Malz zugeben");
  lcd.setCursor(2,3); lcd.print("Hopfen zugeben");
  return;
}

int setTemp(){
  lcd.clear();
  lcd.print(myTxt);
  lcd.setCursor(0,1); lcd.print("Temperatur in Grad:");
  lcd.setCursor(0,2); lcd.print("0 Grad");
  lcd.setCursor(0,3); lcd.print("Drucken fur OK!");
  return showInputSetup("Grad");
}

int setTime(){
  lcd.clear();
  lcd.print(myTxt);
  lcd.setCursor(0,1); lcd.print("Zeit in Minuten:");
  lcd.setCursor(0,2); lcd.print("0 Minuten");
  lcd.setCursor(0,3); lcd.print("Drucken fur OK!");
  int r = -1;
  return showInputSetup("Minuten");
}

int showInputSetup(String was){
  int t = 0;
  int r = false;
  while(1){
    r = getRotary();
    if(r){
      if(r > 0)
        t++;
      else if(r < 0)
        t--;
      else {}
      if(t < 0)
        t = 0;
      clearLine(2);
      lcd.setCursor(0,2); lcd.print(t + String(" ") + String(was));
    }
    if(pushed())
      return t;
  }
  return t;
}

boolean setFertig(){
  lcd.createChar(0, arrow);
  lcd.clear();
  lcd.print("Fertig?");
  lcd.setCursor(0,1); lcd.write(byte(0));
  lcd.setCursor(2,1); lcd.print("Nein!");
  lcd.setCursor(2,2); lcd.print("Ja!");
  lcd.setCursor(0,3); lcd.print("Drucken fur OK!");
  int r = false;
  boolean g = false;
  boolean dome = true;
  while(dome){
    r = getRotary();
    if(r){
      if(r > 0){
        lcd.setCursor(0,1); lcd.write(byte(0));
        lcd.setCursor(0,2); lcd.write(" ");
        g = false;      
      }else if(r < 0){
        lcd.setCursor(0,2); lcd.write(byte(0));
        lcd.setCursor(0,1); lcd.write(" ");
        g = true;
      }
    }
    if(pushed())
      dome = false;
  }
  return g;
}
/*
 * Programm starten
 * Aufheizen => Schnell drehen
 * Hize erreicht => Timer starten & Hitze halten & Langsam drehen
 */
int loadProgramm(int i){
  menuTrigger = false;
  switch(i){
    case 0: doBrauen(programmSetup,sizeof(programmSetup)); break;
    case 1: doBrauen(programmHelles,sizeof(programmHelles)); break;
    case 2: doBrauen(programmDunkles,sizeof(programmDunkles)); break;
    case 99: return 0; break;
  }
  return 0;
}

int doBrauen(int brewProgramm[][3],int n){
  int brewProgrammSize = n / sizeof(int);
  float myTemp = -1;
  for(int i=0;i<brewProgrammSize;i++){
    if(!brewProgramm[i][0])
      i = 0;
    lcd.clear();
    myTxt = String("Durchlauf: ") + (i+1);
    lcd.setCursor(0,0);
    lcd.print(myTxt);
    // Erste Runde -> Nur aufheitzen
    if(brewProgramm[i][2] == 1 && !i){
      clearLine(1);
      myTxt = String("Heizen bis ") + brewProgramm[i][1] + String("C");
      lcd.setCursor(0,1);
      lcd.print(myTxt);
      /*
      Serial.print("Aufheizen auf");
      Serial.print(brewProgramm[i][1]);
      Serial.write(176);
      Serial.println("C");
      */
      setGfa(1);
      myTemp = 0;
      while(myTemp < brewProgramm[i][1]){myTemp = displayTemp(); delay(1000);}
      //Motor auf 75% Last
      setMotor(50);
      
      displayTemp();
      
      setGfa(0);
      
      clearLine(1);
      lcd.setCursor(0,1);
      lcd.print("Einmaischen");
      
      clearLine(2);
      lcd.setCursor(0,2);
      lcd.print("Weiter mit Button.");
      
      //Serial.println("Bitte Einmaischen. Mit Button bestätigen...");
      while(!pushed()){}
    }
    //Serial.println(brewProgramm[i][2]);
    if(brewProgramm[i][2] == 2){

      clearLine(1);
      lcd.setCursor(0,1);
      myTxt = String("Heizen bis ") + brewProgramm[i][1] + String(" C");
      lcd.print(myTxt);

      setGfa(1);

      clearLine(2);
      lcd.setCursor(0,2);
      lcd.print("Motor auf Vollgas");

      //Motor auf 100% Last
      setMotor(100);
      myTemp = 0;
      while(myTemp < brewProgramm[i][1]){myTemp = displayTemp();delay(1000);}
      
      setGfa(0);

      clearLine(2);
      lcd.setCursor(0,2);
      lcd.print("Motor drosseln");
      //Motor auf 50% Last
      setMotor(50);
      
      myTxt = String("Rast: ") + brewProgramm[i][0] + String(" Minuten");
      clearLine(1);
      lcd.setCursor(0,1);
      lcd.print(myTxt);

      unsigned long rast = brewProgramm[i][0] * minute + millis();
      while(timer(rast)){
        warnCriticalTemp(brewProgramm[i][1]);
      }
    }
  }
  setMotor(0);
  setGfa(0);
  lcd.clear();
  lcd.print("Maischen fertig.");
  lcd.setCursor(0,1);
  lcd.print("Maische laeutern...");
  lcd.setCursor(0,2);
  lcd.print("Nachguss aufheizen.");
  lcd.setCursor(0,3);
  lcd.print("Danach Knopf drucken");
  
  boolean dome = true;
  while(dome){
    if(pushed())
      dome = false;
    delay(100);
  }

  lcd.clear();
  lcd.print("Weurze kochen min 1h");
  lcd.setCursor(0,1);
  lcd.print("Hopfen zugeben");

  setGfa(1);
  lcd.setCursor(0,2);
  lcd.print("Auf 100 Grad heizen");
  myTemp = 0;
  while(myTemp < 90){myTemp = displayTemp();delay(1000);}  // 90 Min
  
  while(timer(0)){displayTemp();delay(1000);} // 60 Min
  
  lcd.clear();
  lcd.print("Wuerze gekocht...");
  lcd.setCursor(0,1);
  lcd.print("Zum Beenden bitte");
  lcd.setCursor(0,2);
  lcd.print("Knopf drucken.");

  unsigned long timeNow = millis();
  setTimer(timeNow,3,0);
  
  dome = true;
  while(dome){
    if(pushed())
      dome = false;
    setTimer(timeNow,3,0);
    delay(1000);
  }
  setGfa(0);
  return 99;
}

// Motor GFA
int motorStatus = 0;
boolean setMotor(int i){
  if(i == 1){
    if(motorStatus == 0)
      i = 50;
    else if(motorStatus == 50)
      i = 100;
    else
      i = 0;
  }
 Serial.println(motorStatus);
 Serial.println(": Motor ausschalten");
  motorStatus = i;
  if(motorStatus == 50){
    digitalWrite(Motor1, LOW);
    digitalWrite(Motor2, HIGH);
  }else if(motorStatus == 100){
    digitalWrite(Motor1, HIGH);
    digitalWrite(Motor2, LOW);
  }else{
    digitalWrite(Motor1, HIGH);
    digitalWrite(Motor2, HIGH);
  }
}

boolean gfaStatus = false;
void setGfa(int i){
  if(i && !gfaStatus){
    clearLine(2);
    lcd.setCursor(0,2);
    lcd.print("Zundung ein...");
    digitalWrite(GFA, LOW);
    delay(500);
    lcd.setCursor(0,2);
    lcd.print("Gas oeffnen...");
    digitalWrite(Magnetventil,LOW);
    delay(500);
    int z = 0;
    while(!getFeuer() && z < 10){
      lcd.setCursor(0,2);
      myTxt = String("Versuch ") + (int)z + String("/10");
      lcd.print(myTxt);
      delay(500); 
      z++;
    }
    digitalWrite(GFA,HIGH);
    clearLine(2);
    lcd.setCursor(0,2);
    if(!getFeuer()){
      digitalWrite(Magnetventil,HIGH);
      lcd.print("Feuer fehlgeschlagen");
    }else if(getFeuer()){
      lcd.print("Feuer brennt...");
    }else{
      lcd.print("YOLO? Unmoeglich!");
    }
    delay(500);
    gfaStatus = true;
  }else if(!i && gfaStatus){
    clearLine(2);
    lcd.setCursor(0,2);
    lcd.print("Magnetventil schliessen.");
    digitalWrite(GFA, HIGH);
    delay(1000);
    clearLine(2);
    lcd.setCursor(0,2);
    lcd.print("Brenner ist aus...");
    gfaStatus = false;
  }else{}
}

void toggleBrenner(){
  setGfa(gfaStatus);
}

boolean timer(unsigned long e){
  if(millis() < e){
    e = e - millis();
    float h,m,s,ms;
    h=int(e/3600000);
    e=e%3600000;
    m=int(e/60000);
    e=e%60000;
    s=int(e/1000);
    clearLine(2);
    myTxt = String("Noch: ") + int(h) + String("h ") + int(m) + String("m ") + int(s) + String("s");
    lcd.setCursor(0,2);
    lcd.print(myTxt);
    return true;
  }
  return false;
}

void warnCriticalTemp(int temp){
  float aktTemp = getTemp();
  char buffer[10];
  clearLine(3);
  lcd.setCursor(0,3);
  if(aktTemp <= temp *0.95){
    myTxt = String("Heizen: ") + dtostrf(aktTemp, 5, 2,buffer) + String(" < ") + (int) temp + String("Grad");
    lcd.print(myTxt);
    setGfa(1);
    setMotor(100);
  }else if(aktTemp >= temp * 1.05){
    myTxt = String("Kuhlen: ") + dtostrf(aktTemp, 5, 2,buffer) + String(" > ") + (int) temp + String("Grad");
    lcd.print(myTxt);
    setGfa(0);
    setMotor(50);
  }else{
    myTxt = String("Temperatur: ") + dtostrf(temp, 5, 2,buffer) + String("C Grad");
    lcd.print(myTxt);
    setGfa(0);
    setMotor(0);
  }
  delay(100);
}

/*
* Manueller Modus starten
* 
* Zeitanzeigen / zurücksetzen
* 
* Brenner an/aus
* 
* Motor an/aus
* 
* Zurück
*/
int loadManuell(){
  menuTrigger = false;
  lcd.clear();
  lcd.setCursor(2,3);
  lcd.print("Zuruck");
  unsigned long timeNow = millis();
  setTimer(timeNow,0,2);
  lcd.setCursor(0,0);
  lcd.write(byte(0));
  lcd.setCursor(2,1);
  lcd.print("Brenner an/ein");
  lcd.setCursor(2,2);
  lcd.print("Motor an/aus");
  int r = false;
  int menAkt = 0;
  boolean dome = true;
  while(dome){
    r = getRotary();
    if(r){
      if(r > 0){
        menAkt++;
      }else if(r < 0){
        menAkt--;
      }
      if(menAkt < 0)
        menAkt = 3;
      else if(menAkt > 3)
        menAkt = 0;
      
      lcd.clear();
      lcd.setCursor(2,1);
      lcd.print("Brenner an/ein");
      lcd.setCursor(2,2);
      lcd.print("Motor an/aus");
      lcd.setCursor(2,3);
      lcd.print("Zuruck");
      setTimer(timeNow,0,2);
      lcd.setCursor(0,menAkt);
      lcd.write(byte(0));
    }
    if(pushed()){
      switch(menAkt){
        case 0: timeNow = millis(); setTimer(timeNow,0,2); break;
        case 1: toggleBrenner(); break;
        case 2: setMotor(1); break;
        case 3: return 0; break;
      }
    }
  }
  return 0;
}

boolean setTimer(unsigned long timeNow,int line, int row){
    timeNow = millis() - timeNow;
    float h,m,s,ms;
    h=int(timeNow/3600000);
    timeNow=timeNow%3600000;
    m=int(timeNow/60000);
    timeNow=timeNow%60000;
    s=int(timeNow/1000);

    clearLine(line);
    myTxt = String("Timer: ") + int(h) + String("h ") + int(m) + String("m ") + int(s) + String("s");
    lcd.setCursor(row,line);
    lcd.print(myTxt);
    return true;
}
 
/*
* Temperatur ausgeben
*/
int showTemp(){
  menuTrigger = false;
  float tempAlt = 0;
  int gasAlt = 0;
  boolean feuerAlt = false;
  sensors.setResolution(TEMP_12_BIT);
  boolean dome = true;
  lcd.clear();
  while(dome){
    float temp = displayTemp();
    Serial.println(temp);
    if(temp != tempAlt){
      //clearLine(3);
      tempAlt = temp;
    }
    delay(100);
    
    int gas = displayGas();
    if(gas != gasAlt){
      //clearLine(2);
      gasAlt = gas;
    }
    Serial.println(gas);
    delay(100);
    boolean feuer = displayFeuer();
    if(feuer != feuerAlt){
      //clearLine(1);
      feuerAlt = feuer;
    }
    Serial.println(feuer);
    delay(800);
    if(pushed())
      dome = false;
  }
  //Serial.println("raus!");
  return 0;
}

float getTemp(){
  sensors.requestTemperatures(); // Temperatursensor(en) auslesen
  float temp, temp1, temp2 = 0;
   temp1 = sensors.getTempCByIndex(0);
   temp2 = sensors.getTempCByIndex(1);
   if(temp1 < -50)
    temp1 = 0;
   if(temp2 < -50)
    temp2 = 0;

   temp = temp1 + temp2;
   if(temp == 0)
    return -50;
   return (temp1 + temp2) / 2;
}

float displayTemp(){
  float temp = getTemp();
  char buffer[10];
  myTxt = String("Temperatur: ") + dtostrf(temp, 5, 2, buffer) + String("C");
  lcd.setCursor(0,3);
  lcd.print(myTxt);
  return temp;
}

/*
 * Gasdichte anzeigen
 */
int getGas(){
  int gas = analogRead(GasSensor);
  return gas;
}

int displayGas(){
  int gas = getGas();
  myTxt = String("Gas: ") + gas + String("ppm");
  lcd.setCursor(0,2);
  lcd.print(myTxt);
  return gas;
}

/*
 * Feuer brennt?
 */

boolean getFeuer(){
  boolean isFlame = digitalRead(FeuerSensor);
  if(isFlame == LOW)
    return true;
   else
    return false;
}

boolean displayFeuer(){
  boolean feuer = getFeuer();
  if(feuer)
    myTxt = String("Feuer: An");
   else
    myTxt = String("Feuer: Aus");
  lcd.setCursor(0,1);
  lcd.print(myTxt);
  return feuer;
}

/*
 * Knopfdruck abfragen
 */
boolean pushed(){
  int rStatus = digitalRead(RotaryButton);
  if (rStatus != rAlt) {
    rAlt = rStatus;
    if(rStatus == HIGH)
      return true;
  }
  return false;
}

/*
 * Drehimpuls abfragen
 */
 
const unsigned char ttable[7][4] = {
  {0x0, 0x2, 0x4,  0x0}, {0x3, 0x0, 0x1, 0x10},
  {0x3, 0x2, 0x0,  0x0}, {0x3, 0x2, 0x1,  0x0},
  {0x6, 0x0, 0x4,  0x0}, {0x6, 0x5, 0x0, 0x20},
  {0x6, 0x5, 0x4,  0x0},
};

volatile unsigned char stateR = 0;

int getRotary() {
  unsigned char pinstate = (digitalRead(Rotary2) << 1) | digitalRead(Rotary1);
  stateR = ttable[stateR & 0xf][pinstate];
  unsigned char result = (stateR & 0x30);

  if(result)
    return (result == DIR_CCW ? -1 : 1);
  return false;
}



// notes in the melody:
int melody[] = {
  c4, g4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8
};

void playAlarm(){
  for (int thisNote = 0; thisNote < 2; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(Speaker, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(Speaker);
  }
}

void clearLine(int i){
  lcd.setCursor(0,i);
  lcd.print("                    ");
  return;
}

