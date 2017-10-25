
  
// ArduiStepFocus
// ---------------
// Version 0.7.5TOR
//
// Basé sur la version DRV8825_HW203_F Ver1.51 du projet Arduino Focuser MyFocuser
//
// COMPATIBLE avec le driver ASCOM de ce projet, avec MyFocuserWinApp, avec le logiciel
// Moonlite
//
// PRESENTATION AFFICHEUR:
//  ____________________
//
// GESTION DES BOUTONS
// Normale:
// NOSWITCH  = 0 = SW2 OFF et SW1 OFF : aucune action
// SWEXTRA   = 1 = SW2 OFF et SW1 ON  : "+", Focus Out (déplacement vers extrafocale pour un crayford, MAP vers distances courtes pour un objectif)
// SWINTRA   = 2 = SW2 ON  et SW1 OFF : "-", Focus In (déplacement vers intra-focale pour un crayford, MAP vers infini pour un objectif)
// ALLSWITCH = 3 = SW2 ON and SW1 ON  : entre dans le menu
//
// Menu:
// NOSWITCH  = 0 = SW2 OFF et SW1 OFF : aucune action
// SWPLUS    = 1 = SW2 OFF et SW1 ON  : Change le menu/choix
// SWVALID   = 2 = SW2 ON  et SW1 OFF : Valide le menu/choix
// ALLSWITCH = 3 = SW2 ON and SW1 ON  : aucune action
//
// COMMANDES PORT SERIE
// Cmd  Signification           MyFocuser   INDY (Moonlite)   Objet
// ************************************************************************************************************************************
// ** REQUETES
// GP   Get Position                                          Retourne la position courante du moteur
// GI   Get full In position                                  Le moteur tourne-t-il?
// GN   Get New target position                               Retourne la position cible
// GH   Get Half step                                         Rotation par 1/2 pas selectionnée?
// GS   Get Scale                                             Renvoie le mode micro-pas utilisé
// GT   Get Temperature                                       Renvoie la température corrigée de l'offset
// GZ   Get temp. offset Zero                                 Renvoie la température sans offset
// GV   Get Version                                           Renvoie la version du programme
// GF   Get Firmware                                          Renvoie la version du firmware (figé)
// GD   Get current step Delay                                Non utilisé
// GC   Get temp. Coefficient                                 Non utilisé
// GM   Get MaxSteps                                          Renvoie la position limite du moteur
// GY   Get MaxIncrement                                      Renvoie l'incrément maximum autorisé pour le changement de position
// GO   Get the CoilPwr setting                               Renvoie si les bobines du moteur restent alimentée entre 2pas
// GR   Get Reverse direction                                 Renvoie si l'inversion du sens de rotation est selectionné
// DG   Get Display status                                    Renvoie si l'afficheur est Eteint/Allumé
// GE   Get TempWaitDelay setting                             Renvoie le delai minimum entre 2 acquisitions successive de température
// GB   Get LED Backlight value                               Renvoie toujours "00"
// XY   Mode débugage                                         Récupères les variables pour le monitorer
// ** COMMANDES
// FG   Focuser GOTO                                          Démarre le moteur vers la position cible
// FQ   Focuser Quiet                                         Stop la rotation du moteur, la position courante devient la position cible
// PH   Position Home                                         Retour du focuser à sa position "Home"
// +                                                          Active la compensation automatique de température
// -                                                          Désactive la compensation automatique de température
// ** PARAMETRAGES
// SF   Set motor Full step                                   Rotation par pas entiers
// SH   Set motor Half step                                   Rotation par 1/2 pas
// SS   Set motor step Scale                                  Paramétrage du mode micro-pas
// SP   Set Position                                          Paramètre la position courante du moteur à une valeur arbitraire
// SN   Set New target position                               Paramétrage de la position cible (ne démarre pas le moteur)
// SD   Set step Delay                                        Non utilisé
// SC   Set temp. Coefficient                                 Non utilisé
// PO   temp. Offset Parameter                                Paramètre l'offset de empérature par pas de 0.5° entre -3°..+3°
// SM   Set new MaxSteps position                             Définit une nouvelle limite supérieure du moteur
// SY   Set MaxIncrement                                      Paramètre l'incrément max autorisé pour les changements de position, Ignoré
// SO   Set the CoilPwr setting                               Paramètre si les bobines du moteur restent alimentée entre 2pas
// SR   Set Reverse direction                                 Règle l'inversion du sens de rotation
// MS   Set MotorSpeed                                        Delais entre 2 impulsions successives en ms
// DS   Display Status                                        Eteint/Allume l'afficheur
// SU   Set display Update                                    Autorise ou pas la mise à jour de l'afficheur quand le moteur tourne
// SE   Set TempWaitDelay setting                             Règle le delai minimum entre 2 acquisitions successive de température
// DM   Display temp. mesurement                              Règle l'affichage en °C ou °F
// XZ   reset valeur usine

//*************************************************************************//
//*************************************************************************//
//***                     DECLARATION DES LIBRAIRIES                    ***//
//*************************************************************************//
//*************************************************************************//
#include <Arduino.h>
#include <OneWire.h>                // Necessaire pour DS18B20 sonde de temperature
#include <DallasTemperature.h>      // Necessaire pour DS18B20 sonde de temperature
#include <Wire.h>                   // Necessaire pour I2C
#include <LiquidCrystal_I2C.h>      // Nécessaire pour piloter le LCD en I2C
#include <EEPROM.h>                 // Necessaire pour EEPROM
#include <TimerOne.h>
#include "eepromanything.h"         // Necessaire pour EEPROM

//*************************************************************************//
//*************************************************************************//
//***              DECLARATION DES CONSTANTES GLOBALES                  ***//
//*************************************************************************//
//*************************************************************************//

// Affiché au BOOT sur le LCD
const String ProgramName = "ArduiStepFocus";// Nom du programme
const String ProgramVersion = "Ver 0.7.5.TOR";// Version du programme

//*************************************************************************//
//*************************************************************************//
//***                     DECLARATION DES ALIAS                         ***//
//*************************************************************************//
//*************************************************************************//

//Utilisé pour l'EEPROM
// #define EEPROMSIZE 512              // ATMEGA168 512 EEPROM
#define EEPROMSIZE 1024             // ATMEGA328P 1024 EEPROM
#define EEPROMWRITETEMPO 10000      // Temporisation en ms pour l'écriture en EEPROM
#define DATAOK 99                   // Structure valide en Eeprom
#define DATAWRONG 0                 // Structure invalide en Eeprom

// Utilisé pour la sonde de température
#define MAXPROBES 2                 // Maximum de capteur par canal, en realite un seul supporte
#define TEMP_PRECISION 10           // Regle la precision du DS18B20s à 0.25 degree 
#define TEMPACQUISITIONDELAY 800    // Intervalle entre une requête de mesure de température et le résultat (ms)
#define MAXTEMPWAITDELAY 10000      // Tempo max entre 2 acquisition de température (ms)
#define MINTEMPWAITDELAY 2000       // Tempo min entre 2 acquisition de température (ms)

// Utilisé pour la gestion du moteur
#define MOTORSTEP 200               // Type de moteur (200pas, 400pas...)
#define MOTORDIV 1                  // Démultiplication éventuelle
#define PUSHCOUNTSLOW 20            // Delai avant accélération lors de l'appuie des boutons en vitesse lente
#define PUSHCOUNTMED 12             // Delai avant accélération lors de l'appuie des boutons en vitesse moyenne
#define PUSHCOUNTFAST 8             // Delai avant accélération lors de l'appuie des boutons en vitesse rapide

#define STEPSTARTSTOPSLOW 8         // Nbre de pas pour accélérer et ralentir, pour vitesse lente
#define STEPSTARTSTOPMED 14         // Nbre de pas pour accélérer et ralentir, pour vitesse moyenne
#define STEPSTARTSTOPFAST 35        // Nbre de pas pour accélérer et ralentir, pour vitesse rapide

#define MOTORSPEEDSLOWmS 60         // (ms) Vitesse lente
#define MOTORSPEEDMEDmS 30          // (ms) Vitesse moyenne
#define MOTORSPEEDFASTmS 6          // (ms) Vitesse rapide

#define MOTORSPEEDAUTOCALuS1 32000  // (µs) Vitesse de recherche en autocalibration pas 1/1
#define MOTORSPEEDAUTOCALuS2 16000  // (µs) Vitesse de recherche en autocalibration pas 1/2
#define MOTORSPEEDAUTOCALuS4 8000   // (µs) Vitesse de recherche en autocalibration pas 1/4
#define MOTORSPEEDAUTOCALuS8 4000   // (µs) Vitesse de recherche en autocalibration pas 1/8
#define MOTORSPEEDAUTOCALuS16 2000  // (µs) Vitesse de recherche en autocalibration pas 1/16
#define MOTORSPEEDAUTOCALuS32 1000  // (µs) Vitesse de recherche en autocalibration pas 1/32
#define BACKLASHOFFSET 35           // Offset (en pas) pour gestion du backlash < MOTORSPEEDSTARTSTOPmS
#define MAXFOCUSERLIMIT 65000       // Limite Arbitraire supérieure de la limite supérieure du focuser
#define MINFOCUSERLIMIT 1000        // Limite Arbitraire inférieure de la limite supérieure du focuser
#define BYPASSFOCUSERLIMIT 500      // Nbre de pas en autocal, après inversion, avec fins de course inhibés
#define STEPONTIMEuS 5              // (µs) Durée de l'impulsion d'horloge moteur
#define AUTOCAL true                // Autocalibration
#define MANUALCAL false             // Calibration manuel
#define STEPLIMITDRIVER 32          // Limite des u-pas du driver, 32 pour un DRV8825
#define MAXBYPASSPULSE 3            // Nbre de fausse détection de fin de course admise (érratique)
// Pour la Calibration
#define STEPCAL1 1                  // Recherche de la 1ère butée
#define STEPCAL2 2                  // Recherche de la seconde butée
#define STEPCAL3 3                  // Confirmation du sens du dernier déplacement
#define STEPCALERR1 4               // 1ère butée non trouvée
#define STEPCALERR2 5               // Amplitude du déplacement inférieur à la limite basse (1000)
#define STEPCALERR3 6               // 2nde butée non trouvée
#define STEPCALERR4 7               // Incohérence entre la buté détectée
#define STEPCALEXIT 8               // Sortie de la calibration
// Pour la rotation
#define STEPMOTORINIT 0             // Phase d'initialisation du moteur
#define STEPMOTORINTRA 1            // Rotation en Intra focale
#define STEPMOTORPAUSE 2            // Pause d'inversion de sens de rotation
#define STEPMOTOREXTRA 3            // Rotation en extra focale
#define STEPMOTOREND 4              // Sortie de la procédure de rotation

// Utilisé pour le LCD
#define LCDLENGTH 20                // Nbre caractères/lignes, LCD2004=20 ou LCD1602=16
#define LCDLINE 4                   // Nbre de lignes du LCD, LCD2004=4 ou LCD1602=2
#define LCDUPDATERATETEMP 10000     // Rythme rafraichissement Température sur LCD
#define LCDUPDATEPOS 300            // Rythme rafraichissement position
#define LCDBEFOREOFF 20000          // Tempo en ms avant extinction écran
#define LCDON true                  // Activation LCD
#define LCDOFF false                // Désactivation LCD

// Mise en service du buzzer
#define BUZZERSTATE 1023            // Buzzer actif
//#define BUZZERSTATE 0               // Buzzer inactif

// Pour les boutons
#define NOSWITCH 0                  // Aucun bouton enfoncé
#define SWEXTRA 1                   // Valeur du switch "déplacement Extra focal" = "+"
#define SWPLUS 1                    // Valeur du switch "+" = "Navigation"
#define SWINTRA 2                   // Valeur du switch "déplacement Intra focal" = "-"
#define SWVALID 2                   // Valeur du switch "Validation"
#define ALLSWITCH 3                 // Les 2 boutons enfoncés
#define PBCHECKRATE 300             // Boutons vérifié toutes les 300ms

// Fin de course
#define LIMITSWITCHDISABLE 1        // Fin de course Absent
#define LIMITSWITCHACTIVLOW 2       // Fin de course actif à l'état bas
#define LIMITSWITCHACTIVHIGH 3      // Fin de course actif à l'état haut

// Backlash
#define BACKLASHOFF 1               // Pas de traitement du Backlash
#define BACKLASHLOW 2               // Backlash faible
#define BACKLASHMED 3               // Backlash moyen
#define BACKLASHHIGH 4              // Backlash fort

// Valeurs usines MyFocuser
#define FACTORYMOTORSPEED MOTORSPEEDSLOWmS
#define FACTORYREVERSEDIR false
#define FACTORYBACKLASH BACKLASHOFF
#define FACTORYMAXSTEP 10000
#define FACTORYFOCUSPOS 5000
#define FACTORYSTEPMODE 1
#define FACTORYCOILPWR true
#define FACTORYSWITCH LIMITSWITCHDISABLE
#define FACTORYTEMPWAITDELAY 5000

// Pour les Menus
#define NBREMENU1 11
#define NBREMENU2 6

//*************************************************************************//
//*************************************************************************//
//***             AFFECTATION DES BROCHES DU µCONTROLEUR                ***//
//*************************************************************************//
//*************************************************************************//

// Sonde température
#define Ch1Temp 2                   // Sonde de temperature sur D2, utilise 4.7k de pullup
// Driver Moteur pas à pas,  DRV8825
#define MyDir     3                 // Direction, D3
#define MyStep    4                 // Horloge, D4
// Version ERIC
//#define MyM2      7                 // Choix du mode de fonctionnement (pas entiers...) / Poids Fort, D5
//#define MyM1      6                 // Choix du mode de fonctionnement (pas entiers...), D6
//#define MyM0      5                 // Choix du mode de fonctionnement (pas entiers...) / Poids faibles, D7
// Version originale
#define MyM2      5                 // Choix du mode de fonctionnement (pas entiers...) / Poids Fort, D5
#define MyM1      6                 // Choix du mode de fonctionnement (pas entiers...), D6
#define MyM0      7                 // Choix du mode de fonctionnement (pas entiers...) / Poids faibles, D7
                                    // M2 M1 M0 Mode    Pas par revolution
                                    // 0  0  0  Entier  200
                                    // 0  0  1  1/2     400
                                    // 0  1  0  1/4     800
                                    // 0  1  1  1/8     1600
                                    // 1  0  0  1/16    3200
                                    // 1  0  1  1/32    6400
                                    // 1  1  0  1/64    12800, non supporté par le DRV8825 (1/32)
                                    // 1  1  1  1/128   25600, non supporté par le DRV8825 (1/32)
                                    // La limitation en courant doit-etre réglée pour le bon fonctionnement des micro-pas
#define MyEnable  8                 // /ENABLE du DRV8825, valide ou pas l'alimentation des bobines moteur
// Switch Fin de course (VERSION ERIC)          
#define LimitIntra 9                // Interupteur fin de course Intra focale
#define LimitExtra 10               // Interupteur fin de course Extra Focale
// NOTE: Utilise les broches analogiques
// Declaration des LEDS, associe aux boutons poussoir et mouvements du moteur
#define LED_ToIntra A1              // Indique moteur actif +direction rotation, Analogique A1 (LED Bleue)
#define LED_ToExtra A2              // Indique moteur actif +direction rotation, Analogique A2 (LED Verte)
// Declaration Buzzer
#define Buzzer A3                   // Buzzer, Analogique A3
// Déclaration Boutons
#define PBIntra 14                  // Bouton Déplacement Intra
#define PBExtra 15                  // Bouton Déplacement Extra
// LCD CONNECTE SUR LES BROCHES A4/A5 de DE L'ARDUINO

//*************************************************************************//
//*************************************************************************//
//***                         VARIABLES GENERALES                       ***//
//*************************************************************************//
//*************************************************************************//

unsigned long CurrentMillis;        // Variable tampon

//  Pour la Gestion de l'EEPROM
// Stocké en EEPROM - Toutes les variables contenus dans une structure
struct ConfigType 
{
  unsigned char ValidData;          // Si à 99 alors les donnees sont valides
  unsigned char MotorSpeed;         // Vitesse de rotation moteur
  boolean ReverseDirection;         // Commandes de rotation moteur inversé
  unsigned char BacklashLevel;      // Gestion du Backlash
  long MaxStep;                     // Limite haute du nbre de pas, la basse à 0
  long FocuserPosition;             // Derniere position du Focuser
  unsigned char StepMode;           // Type de conctionnement (Pas entiers, 1/2 Pas, 1/4, 1/8. 1/16. 1/32 [1.2.4.8.16.32]
  boolean CoilPwr;                  // Maintien de l'alimentation des bobines du moteur
  unsigned char LimitSwitch;        // Présence & état actif des capteurs fin de course 
  long TempWaitDelay;               // Intervalle en milliseconds entre 2 acquisition de temperature (5s par défaut), configurable
} MyFocuser;

unsigned char DataSize;             // Contiendra la taille de la structure MyFocuser
int NumberOfLocations;              // Nombre d'emplacements de stockage disponible en EEPROM
int CurrentAddr;                    // Contiendra l'adresse en EEPROM de la donnees stockee
boolean WriteEepromNow = false;     // Doit-on mettre a jour la donnees en eeprom?
boolean DataEepromFound;            // A-t-on trouve une valeur valide stockée?
unsigned long PreviousMillisEeprom = 0;// Moment de la dernière sauvegarde en EEPROM

// Pour la gestion du moteur
long TargetPosition = 5000;         // Position de consigne
long TargetPositionStore = 5000;    // Variable temporaire mémo Consigne
long MaxIncrement = 1000;           // Nombre maximum de pas autorisés pour un déplacement
boolean GotoNewPosition = false;    // Utilisé par Moonlite après une commande SN suivi d'un FG
unsigned char StepMotorNumber = STEPMOTORINIT;
int StepDelay = 2;                  // Pour les focuser moonlite
// Entrée ENABLE du DRV8825, détermine l'alimentation des bobines moteur
// NOTE: Mode µ-pas, alimentation moteur toujours ON = on ne désactive pas le DRV8825
long DeltaPosition;                 // Ecart entre la position courante et la position cible
unsigned char DeltaSpeed;           // Ecart entre position courante et cible pour ralentir
boolean GotoExtra;                  // Le crayford se déplace en ExtraFocale (indépendant du backlash)
boolean IsMoving = false;           // Le moteur est à l'arrêt
long BacklashOffset;                // Nbre de pas pour le traitement du Backlash
unsigned char CountWait = 0;        // Pour la gestion de la vitesse
unsigned char CurrentWait;          // Pour la gestion de la vitesse
boolean MotorStopping;              // Le moteur ralenti
boolean EnableLSwitch;              // Active les fins de course
int SpeedCal;                       // Programmation du Timer1 en calibration
unsigned char FilterCount;          // pour le filtrage des fausses détections de fin de course
// Pour la gestion de la sonde température
boolean ProbeTemp1Present = false;  // Indique si le probe 1 est présent (défaut = non)
float Probe1TempVal = 20.0;         // Temperature courante du capteur (20°C si pas de probe)
long  TempPreviousMillis = 0;       // Moment de la dernière acquisition de température

// Pour le Gestion de température (ASCOM)
float TempOffsetVal = 0;            // Calibration de l'offset de temperature (moonlite)
int TempComp = 0;                   // Compensation mesure température (moonlite)

// Pour la gestion de l'affichage
unsigned long LastPosDisplay = 0;   // Moment du dernier affichage Position
unsigned long LastTempDisplay = 0;  // Moment du dernier affichage Température
unsigned long LastWakeeUpLCD;       // Moment du dernier réveille du LCD 
boolean UpdateLCD_WhileMoving = true;// Permet de déclencher LCD_PosMoteur(), mise à jour affichage position moteur
boolean FlashLCD = false;           // Permet de faire clignonter l'afficheur (µ-pas non supportés)
boolean LCD_OnBySerial = false;     // LCD forcé à 1 par le port série
boolean TempAcqProcessing = false;  // Acquisition de température en cours
boolean DisplayEnabled;             // Indique l'état du LCD allumé/éteint

// Pour les boutons
unsigned long LastPB_Check = 0;     // Moment de la derniere vérif bouton
unsigned char PushCountInit;
unsigned char PushCount;            // Nbre de passage  ds bcle appuie bouton sans relachement bouton
boolean WaitMotorStop = false;      // Permet d'attendre l'arret complet du moteur avant inversé sens consigne
boolean SlowSpeedForced = false;    // Force la vitesse lente
boolean IsMovingByButton = false;   // Le moteur bouge par les boutons

// Pour les menus
const String  MenuRow1[NBREMENU1] = {"Vites.","Inver.","R.Jeu","PosMin","PosMax","M.Mode","M.Alim","F.Cour","Usine","Calib.","EXIT"};
String  MenuRow2[NBREMENU2];

// Pour la calibration
boolean AutoCal;                    // Variable indiquant le choix de la calibration Auto/Manuelle
boolean AutoCalEnd;                 // Variable indiquant la fin de la calibration dans un sens

//*************************************************************************//
//*************************************************************************//
//***                              GENERALE                             ***//
//*************************************************************************//
//*************************************************************************//

// ............................................................... //
// .. Nom: TestPause200()                                       .. //
// .. Role: Marque une pause de 200ms                           .. //
// .. Entrées: -                                                .. //
// .. Sortie: -                                                 .. //
// ................................................................//
void TestPause200()
{
  delay(200);
} // void TestPause200()

// ............................................................... //
// .. Nom: TestPause1000()                                      .. //
// .. Role: Marque une pause de 1s                              .. //
// .. Entrées: -                                                .. //
// .. Sortie: -                                                 .. //
// ................................................................//
void TestPause1000()
{
  delay(1000);
} // void TestPause1000()

//*************************************************************************//
//*************************************************************************//
//***                                   LCD                             ***//
//*************************************************************************//
//*************************************************************************//

//*** DECLARATIONS
// Declaration de l'objet LCD connecte sur les broches A4/A5 de l'arduino
LiquidCrystal_I2C lcd(0x27, LCDLENGTH, LCDLINE);

// ............................................................... //
// .. Nom: LCD_Status(boolean State)                            .. //
// .. Role: Allume/Eteint l'écran (en Bipant)                   .. //
// .. Entrées: State, DisplayEnabled, Buzzer, BUZZERSTATE       .. //
// .. Sortie: DisplayEnabled                                    .. //
// ................................................................//
void LCD_Status(boolean State)
{
  if (DisplayEnabled != State)
  {
    analogWrite(Buzzer, BUZZERSTATE);// Beep à l'extinction
    DisplayEnabled = State;
    if (DisplayEnabled)
    { // Allume l'écran
      lcd.display();
      lcd.backlight();
    } // if (DisplayEnabled)
    else
    { // Eteint l'écran
      lcd.noDisplay();
      lcd.noBacklight();
    } // (else) if (DisplayEnabled)
    TestPause200();
    analogWrite(Buzzer, 0);         // Beep à l'extinction
  } // if (DisplayEnabled != State)
} // void LCD_Status(boolean State)

// ............................................................... //
// .. Nom: String TemperatureToString()                         .. //
// .. Role: Convertie un flottant Probe1TempVal en chaine de    .. //
// ..       caractères avec une précision de 2 décimales        .. //
// .. Entrées: Probe1TempVal                                    .. //
// .. Sortie: TemperatureToString()                             .. //
// ................................................................//
String TemperatureToString()
{
  int TempInt;
  String TempString1, TempString2;

  // Arrondi avec la précision voulue
  TempInt =  (Probe1TempVal * 100);
  // Copie les caractères jusqu'au point
  TempString1 = (String) TempInt;
  TempString2 = TempString1.substring(0,(TempString1.length()-2));
  TempString2 = TempString2 + ".";
  TempString2 = TempString2 + TempString1.substring((TempString1.length()-2));
  return TempString2;
} // String TemperatureToString()

// ............................................................... //
// .. Nom: LCD_Config()                                         .. //
// .. Role: Affiche le fond d'ecran et les valeurs de paramétrage. //
// ..       et température                                      .. //
// .. Entrées: MyFocuser, Probe1TempVal                         .. //
// .. Sortie: Aucun                                             .. //
// ................................................................//
void LCD_Config()
{
  // Fond ecran
  lcd.setCursor(0,0);
  lcd.print("Pos:      [0..      ");
  lcd.setCursor(0,1);
  lcd.print("T:           RJeu:  ");
  lcd.setCursor(0,2);
  lcd.print("u-pas:1/    Pwr:    ");
  lcd.setCursor(0,3);
  lcd.print("Vit:  Rot:   L.Sw:  ");
  // 1ere ligne
  LCD_MotorPosition();              // Affiche la position courante
  lcd.setCursor(14,0);              // Positionne le curseur
  lcd.print((String) MyFocuser.MaxStep);// Affiche la limite haute
  lcd.print("]");
  // 2nde Ligne
  LCD_Temperature();                // Affiche la température
  lcd.setCursor(18,1);              // Positionne le curseur
  lcd.print((String) (MyFocuser.BacklashLevel - 1));
  // 3eme Ligne
  lcd.setCursor(8,2);               // Positionne le curseur
  lcd.print((String) (MyFocuser.StepMode));// Le mode µ-pas
  lcd.setCursor(16,2);              // Positionne le curseur
  if (MyFocuser.CoilPwr) lcd.print("On");// Le type d'alimention moteur
    else lcd.print("Off");
  // 4eme Ligne
  lcd.setCursor(4,3);               // Positionne le curseur
  switch(MyFocuser.MotorSpeed)      // Affiche la vitesse
  {
    case MOTORSPEEDSLOWmS : lcd.print("1");
    break;
    case MOTORSPEEDMEDmS : lcd.print("2");
    break;
    case MOTORSPEEDFASTmS : lcd.print("3");
    break;
    default : lcd.print("?");
  } // switch(MyFocuser.MotorSpeed
  lcd.setCursor(10,3);              // Positionne le curseur
  if (MyFocuser.ReverseDirection) lcd.print("<-");// Affiche l'inversion de sens
  else lcd.print("->");
  lcd.setCursor(18,3);              // Positionne le curseur
  if (MyFocuser.LimitSwitch == LIMITSWITCHACTIVLOW) lcd.print("B");
    else if (MyFocuser.LimitSwitch == LIMITSWITCHACTIVHIGH) lcd.print("H");
          else lcd.print("A");
} // void LCD_Config()

// ............................................................... //
// .. Nom: LCD_MotorPosition()                                  .. //
// .. Role: Affiche la position courante du moteur en usage nor-.. //
// ..       mal.                                                .. //
// .. Entrées: MyFocuser.FocuserPosition                        .. //
// .. Sortie: Aucun                                             .. //
// ................................................................//
void LCD_MotorPosition()
{
  lcd.setCursor(4,0);               // Positionne le curseur
  lcd.print("     ");  
  lcd.setCursor(4,0);               // Positionne le curseur
  lcd.print((String) MyFocuser.FocuserPosition);
} // void LCD_MotorPosition()


// ............................................................... //
// .. Nom: LCD_MotorPositionCal()                               .. //
// .. Role: Affiche la position courante du moteur en calibration. //
// .. Entrées: MyFocuser.FocuserPosition                        .. //
// .. Sortie: Aucun                                             .. //
// ................................................................//
void LCD_MotorPositionCal()
{
  lcd.setCursor(8,1);               // Positionne le curseur
  lcd.print("     ");  
  lcd.setCursor(8,1);               // Positionne le curseur
  lcd.print((String) MyFocuser.FocuserPosition);
} // void LCD_MotorPositionCal()

// ............................................................... //
// .. Nom: LCD_MotorTaget()                                     .. //
// .. Role: Affiche la consigne de position du moteur           .. //
// .. Entrées: Target_Position                                  .. //
// .. Sortie: Aucun                                             .. //
// ................................................................//
void LCD_MotorTaget()
{
  lcd.setCursor(9,0);               // Positionne le curseur
  lcd.print(" Cons:     ");
  lcd.setCursor(15,0);              // Positionne le curseur
  lcd.print((String) TargetPosition);
} // void LCD_MotorTaget()

// ............................................................... //
// .. Nom: LCD_PosLimit()                                       .. //
// .. Role: Affiche les limites de  position du moteur          .. //
// .. Entrées: MyFocuser.MaxStep                                .. //
// .. Sortie: Aucun                                             .. //
// ................................................................//
void LCD_PosLimit()
{
  lcd.setCursor(10,0);              // Positionne le curseur
  lcd.print("[0..      ");
  lcd.setCursor(14,0);              // Positionne le curseur
  lcd.print((String) MyFocuser.MaxStep);
  lcd.print("]");
} // void LCD_PosLimit()

// ............................................................... //
// .. Nom: LCD_Temperature(float Temperature)                   .. //
// .. Role: Affiche la température courante en °C, précision 2  .. //
// ..       décimales                                           .. //
// .. Entrées: Probe1TempVal, ProbeTemp1Present                 .. //
// .. Sortie: Aucun                                             .. //
// ................................................................//
void LCD_Temperature()
{
  lcd.setCursor(2,1);               // Positionne le curseur
  if (ProbeTemp1Present)
  {
    lcd.print("        ");          // Efface les anciennes valeurs
    lcd.setCursor(2,1);             // Positionne le curseur
    lcd.print(TemperatureToString());
    lcd.print((char) 223);
    lcd.print("C");
  } // if (ProbeTemp1Present)
  else lcd.print("-");
} // void LCD_Temperature(float Temperature)

// ............................................................... //
// .. Nom: LCD_Store(boolean NeedToStore)                       .. //
// .. Role: Affiche une 'M' pour signifié patamètre non         .. //
// ..       sauvegardé en EEPROM                                .. //
// .. Entrées: NeedToStore                                      .. //
// .. Sortie: Aucun                                             .. //
// ................................................................//
void LCD_Store(boolean NeedToStore)
{
  lcd.setCursor(19,1);              // Positionne le curseur
  if (NeedToStore) lcd.print('M');
  else lcd.print(' ');
} // void LCD_Store(boolean NeedToStore)

// ............................................................... //
// .. Nom: Set_Menu2()                                          .. //
// .. Role: Prépare le tableau contenant les textes des Menu de .. //
// ..       niveau 2, en fonction du menu de niveau 1           .. //
// .. Entrées: Menu1                                            .. //
// .. Sortie: MenuRow2[]                                        .. //
// ................................................................//
void Set_Menu2(unsigned char Menu1)
{   
  //Menu2
  switch (Menu1)
  {
    case 1 :  // Vitesse
    {
      MenuRow2[0]="Lent";
      MenuRow2[1]="Moyenne";
      MenuRow2[2]="Rapide ";
      break; 
    } // case 1 :
    case 3 :  // Rattrapage jeux
    {
      MenuRow2[0]="Off";
      MenuRow2[1]="Faible";
      MenuRow2[2]="Moyen";
      MenuRow2[3]="Fort";
      break; 
    } // case 3 :
    case 6 :  // u-pas
    {
      MenuRow2[0]="1/1";
      MenuRow2[1]="1/2";
      MenuRow2[2]="1/4";
      MenuRow2[3]="1/8";
      MenuRow2[4]="1/16";
      MenuRow2[5]="1/32";
      break; 
    } // case 6 :
    case 8 :  // Fin de course
    {
      MenuRow2[0]="Absent";
      MenuRow2[1]="0V";
      MenuRow2[2]="5V";
      break; 
    } // case 8 :
    case 10 :  // Calibration
    {
      MenuRow2[0]="Non";
      MenuRow2[1]="Manu.";
      MenuRow2[2]="Auto";
      break; 
    } // case 10 :
    default :
    {
      MenuRow2[0]="Non";
      MenuRow2[1]="Oui";
    } // default :
  } // Fin switch (Menu1)
} // Fin void Set_Menu2(unsigned char Menu1)

// ............................................................... //
// .. Nom: LCD_Menu(unsigned char Menu1,unsigned char Menu2)    .. //
// .. Role: Affiche les menus et sous menus                     .. //
// .. Entrées: Menu1, Menu2                                     .. //
// .. Sortie: Aucun                                             .. //
// ................................................................//
void LCD_Menu(unsigned char Menu1,unsigned char Menu2)
{
  unsigned char Idx1,Idx2;
  
  if (Menu2 == 0)
  {
    lcd.clear();
    lcd.setCursor(3,0);
    lcd.print("MENU (");
    lcd.print(ProgramVersion);
    lcd.print(")");
    // Affiche le menu 
    lcd.setCursor(0,1);
    lcd.print(">");          
    Idx1 = Menu1 - 1;
    Idx2 = 1;
    while ((NBREMENU1 > Idx1) && (Idx2 < LCDLINE))   
    {
      lcd.setCursor(1,Idx2);
      lcd.print(MenuRow1[Idx1]);
      Idx1++;         
      Idx2++;
    } // while ((NBREMENU1 > Idx1) && (Idx2 < LCDLINE))
  } // if (Menu2 == 0)
  else
  {
    // Efface les lignes
    lcd.setCursor(7,1);
    lcd.print(">             ");
    Set_Menu2(Menu1);           // Prépare la table texte du 2nd menu
    // Affiche les réglages
    lcd.setCursor(8,1);
    lcd.print(MenuRow2[(Menu2 - 1)]);
  } // (else) if (Menu2 == 0)
} // void LCD_Menu(unsigned char Menu1,unsigned char Menu2)

//*************************************************************************//
//*************************************************************************//
//***                           BOUTONS POUSSOIRS                       ***//
//*************************************************************************//
//*************************************************************************//
// Declaration des boutons poussoir, Utilise un diviseur de tension pour les 2 boutons poussoirs
// Traitement Anti-Rebond logiciel
// GESTION DES BOUTONS
// Normale:
// NOSWITCH  = 0 = SW2 OFF et SW1 OFF : aucune action
// SWEXTRA   = 1 = SW2 OFF et SW1 ON  : "+", Focus Out (déplacement vers extrafocale pour un crayford, MAP vers distances courtes pour un objectif)
// SWINTRA   = 2 = SW2 ON  et SW1 OFF : "-", Focus In (déplacement vers intra-focale pour un crayford, MAP vers infini pour un objectif)
// ALLSWITCH = 3 = SW2 ON and SW1 ON  : entre dans le menu
//
// Menu:
// NOSWITCH  = 0 = SW2 OFF et SW1 OFF : aucune action
// SWPLUS    = 1 = SW2 OFF et SW1 ON  : Change le menu/choix
// SWVALID   = 2 = SW2 ON  et SW1 OFF : Valide le menu/choix
// ALLSWITCH = 3 = SW2 ON and SW1 ON  : aucune action

// ............................................................... //
// .. Nom: ReadPB_Switches(unsigned char pinNum)                .. //
// .. Role: Lit la tension sur la ligne "PinNum" et en fonction .. //
// ..       détermine la combinaison de boutons enfoncés        .. //
// ..       Tension entre 650..720 = SW2 OFF et SW1 ON Renvoie 1.. //
// ..       Tension entre 310..380 = SW2 ON et SW1 OFF Renvoie 2.. //
// ..       Tension entre 460..530 = SW2 ON and SW1 ON Renvoie 3.. //
// ..       Sinon SW2 OFF et SW1 OFF Renvoie 0                  .. //
// .. Entrées:  PinNum                                          .. //
// .. Sortie: ReadPB_Switches                                   .. //
// ................................................................//
unsigned char ReadPB_Switches() 
{
  int ValPBIntra; // Actif à l'état bas
  int ValPBExtra; // Actif à l'état bas
  
  ValPBIntra = digitalRead(PBIntra);
  ValPBExtra = digitalRead(PBExtra);

  // Traitement anti-rebond
    if ((ValPBIntra == LOW) or (ValPBExtra == LOW))
    {
      delay(50);
      ValPBIntra = digitalRead(PBIntra);
      ValPBExtra = digitalRead(PBExtra);

    if (ValPBIntra == LOW)                       // Intra enfoncé
    {
      if (ValPBExtra == LOW) return ALLSWITCH;    // Extra enfoncé (SW2 ON  et SW1 ON)  
      else return SWINTRA;                        // Extra relaché (= SWVALID  SW2 ON  et SW1 OFF)
    }
    else                                        // Intra relaché
    {
      if (ValPBExtra == LOW) return SWEXTRA;      // Extra enfoncé (= SWPLUS   SW2 OFF et SW1 ON) 
      else return NOSWITCH;                       // Extra relaché (SW2 OFF et SW1 OFF)
    }
  }
  else return NOSWITCH;                         // Extra relaché (SW2 OFF et SW1 OFF)
} // void ReadPB_Switches()

// ............................................................... //
// .. Nom: MenuAdjust()                                         .. //
// .. Role: Gère les menus et règle les paramètre MyFocuser     .. //
// .. Entrées: MyFocuser                                        .. //
// .. Sortie: MyFocuser, WriteEepromNow                         .. //
// ................................................................//
void MenuAdjust()
{
  unsigned char PB_Val;             // Fonction des bts appuyés
  unsigned char Tampon;
  unsigned char Menu1Number = 1;    // 1..11
  unsigned char Menu2Number = 0;    // Si=0 pas actif, 1..5
  unsigned char Menu2NumberMax[NBREMENU1] = {3,2,4,2,2,6,2,3,2,3,2};    // Nbre de sous menus max par menus
  boolean MenuAdjust = false;       // On est dans les valeurs de réglage des menu
  boolean GoOut = false;            // Pour sortir bcle Menu
  unsigned char FinalStepMode;      // Valeur du mode paramétré, la valeur final validé en sortie de procédure

 // Paramètre le timer 1 pour la vitesse de calibration
 // Rétabli le timer normal
  Timer1.stop(); 
  Timer1.detachInterrupt();

  LCD_Menu(Menu1Number,0);
  FinalStepMode = MyFocuser.StepMode;                     // permettra de recalculer la position et la limite max
  while (!GoOut)
  {
    while (ReadPB_Switches() != NOSWITCH) {TestPause200();}// Attends bouton relaché
    do
    { // Anti-rebond
      PB_Val = ReadPB_Switches();
      TestPause200();
      if (ReadPB_Switches() != PB_Val) PB_Val = NOSWITCH;
    } 
    while ((PB_Val == NOSWITCH) || (PB_Val == ALLSWITCH));// Attends bouton renfoncé

    if (PB_Val == SWPLUS)                                 // Appui sur "+"
    {
      if (MenuAdjust)
      { // Change la valeur de réglage
        Menu2Number++;
        if (Menu2Number > Menu2NumberMax[Menu1Number-1]) Menu2Number = 1;
        if ((Menu1Number == 10) && (Menu2Number == 3) && (MyFocuser.LimitSwitch == LIMITSWITCHDISABLE)) Menu2Number = 1;
        LCD_Menu(Menu1Number,Menu2Number);
      } // if (MenuAdjust)
      else
      { // Change le Menu
        Menu1Number++;
        if (Menu1Number > NBREMENU1) Menu1Number = 1;
        LCD_Menu(Menu1Number,0);
      } // (else) if (MenuAdjust)
    } // if (PB_Val == SWPLUS)
    else                                                  // Appui sur "Valider", PB_Val == SWVALID
    {
      if (MenuAdjust)
      { // Valide le réglage
        if (Menu1Number != 12)  WriteEepromNow = true;
        switch (Menu1Number) 
        {
          case 1: // Vitesse moteur
          {
            if (Menu2Number == 1) MyFocuser.MotorSpeed = MOTORSPEEDSLOWmS;
            else if (Menu2Number == 2) MyFocuser.MotorSpeed = MOTORSPEEDMEDmS;
                 else MyFocuser.MotorSpeed = MOTORSPEEDFASTmS;
            break;
          } // case 1:
          case 2: // Inversion focus+/focus-
          {
            MyFocuser.ReverseDirection = (Menu2Number == 2);
            break;
          } // case 2:
          case 3: // Rattrapage du jeu
          {
            MyFocuser.BacklashLevel = Menu2Number;        // BACKLASHOFF=1, BACKLASHLOW=2, BACKLASHMED=3, BACKLASHHIGH=4
            break;
          } // case 3:
          case 4: // Fixe limite Min à la position courante et reset la limite haute
          {
            if (Menu2Number == 2)
            {
              MyFocuser.MaxStep = FACTORYMAXSTEP;
              MyFocuser.FocuserPosition = 0;
              TargetPosition = 0;
              TargetPositionStore = 0;
              lcd.setCursor(8,1);
              lcd.print("Ok!");
              TestPause1000();
            } // if (Menu2Number == 2)
            break;
          } // case 4:
          case 5: // Fixe limite haute à la position courante
          {
            if (Menu2Number == 2)
            {
              MyFocuser.MaxStep = MyFocuser.FocuserPosition;
              lcd.setCursor(8,1);
              lcd.print("Ok!");
              TestPause1000();
           }  // if (Menu2Number == 2)
            break;
          } // case 5:
          case 6: // Mode µ-pas (pas à pas)
          {
            FinalStepMode = bit(Menu2Number - 1);
             break;
          } // case 6:
          case 7: // Alimentation permanente moteur (pas à pas)
          {
            MyFocuser.CoilPwr = (Menu2Number == 2);
            break;
          } // case 7:
          case 8: // Fin de course
          {
            MyFocuser.LimitSwitch = Menu2Number;          // LIMITSWITCHDISABLE=1, LIMITSWITCHACTIVLOW=2, LIMITSWITCHACTIVHIGH=3
            break;
          } // case 8:
          case 9: // Chargement valeur usine
          {
            if (Menu2Number == 2)
            {
              Reset_Myfocuser();
              lcd.setCursor(8,1);
              lcd.print("Ok!");
              TestPause1000();
            } // if (Menu2Number == 2)
            break;
          } // case 9:
          case 10: // Calibration
          {
            if (Menu2Number == 2) AutoCal = MANUALCAL;
            else if (Menu2Number == 3) AutoCal = AUTOCAL;
            if (Menu2Number != 1)
            {
              SetStepMode(FinalStepMode);
              FocuserCalibration();
            } // if (Menu2Number != 1)
            break;
          } // case 10:
          case 11: // Sortie du menu
          {
            GoOut = (Menu2Number == 2);
          } // case 11:
        } // Fin switch (Menu1Number)
        MenuAdjust = false;
        Menu2Number = 0;
        LCD_Menu(Menu1Number,0);
      } // if (MenuAdjust)
      else 
      { // Rentre dans le réglage du menu
        MenuAdjust = true;
        switch (Menu1Number)
        {
          case 1:  // Vitesse moteur
          {
            if (MyFocuser.MotorSpeed == MOTORSPEEDSLOWmS) Menu2Number = 1;
            else if (MyFocuser.MotorSpeed == MOTORSPEEDMEDmS) Menu2Number = 2;
                  else Menu2Number = 3;
            break; 
          } // case 1:
          case 2:  // Inversion commaneds focus + /focus -
          {
            if (MyFocuser.ReverseDirection) Menu2Number = 2;
            else Menu2Number = 1;
            break; 
          } // case 2:
          case 3:  // Niveau de Backlash
          {
            Menu2Number = MyFocuser.BacklashLevel;        // BACKLASHOFF=1, BACKLASHLOW=2, BACKLASHMED=3, BACKLASHHIGH=4
            break; 
          } // case 3:
          case 6:  // Mode µ-pas
          {
            Tampon= 1;
            Menu2Number = 1;
            while (Tampon != FinalStepMode)
            {
              Tampon = (Tampon << 1);
              Menu2Number++;
            } // while (Tampon != FinalStepMode)
            break; 
          } // case 6:
          case 7:  // Alimentation moteur focuser
          {
            if (MyFocuser.CoilPwr) Menu2Number = 2;
            else Menu2Number = 1;
            break; 
          } // case 7:
          case 8:  // Capteur fin de course
          {
            Menu2Number = MyFocuser.LimitSwitch;          // LIMITSWITCHDISABLE=1, LIMITSWITCHACTIVLOW=2, LIMITSWITCHACTIVHIGH=3
            break; 
          } // case 8:
          default : Menu2Number = 1;
        } // Fin switch (Menu1Number)
        LCD_Menu(Menu1Number,Menu2Number);
      } // (else) if (MenuAdjust)
    } // (else) if (PB_Val == SWPLUS)
  } // while (!GoOut)
  // Réglage µ-Pas
  SetStepMode(FinalStepMode);
  // Rétabli le Timer1
  SetTimerMotor();
  Timer1.attachInterrupt(MotorPulse);
  Timer1.stop(); 
  
  // Réglage de l'alimentation des bobines
  if (MyFocuser.CoilPwr) EnableCoilOutput();
  else DisableCoilOutput();      
  // Sauvegarde en EEPROM
  if (WriteEepromNow) WriteMyFocuserEeprom();
} // void MenuAdjust()

// ............................................................... //
// .. Nom: PB_IntraExtra()                                      .. //
// .. Role: En fonction des boutons appuyés,génère un déplace-  .. //
// ..       ment Intra/Extra focale ou un arrêt du moteur. Gère .. //
// ..       la vitesse de déplacement du moteur en fonstion de  .. //
// ..       la durée d'appui.                                   .. //
// .. Entrées: IsMoving, LastPB_Check,IsMovingByButton,         .. //
// ..         WaitMotorStop, MyFocuser.MaxStep                  .. //
// .. Sortie: TargetPosition, GotoNewPosition,SlowSpeedForced   .. //
// ..         IsMovingByButton, WaitMotorStop, PushCount        .. //
// ..         StepMotorNumber, LastPB_Check                     .. //
// ................................................................//
void PB_IntraExtra()
{
  unsigned char PB_Value;
 
  if ((CurrentMillis - LastPB_Check) >= PBCHECKRATE)
  {
    PB_Value = ReadPB_Switches();
    switch (PB_Value)
    {
      case SWEXTRA:
      {
        if (!IsMoving)
        {
          TargetPosition = MyFocuser.MaxStep;
          GotoNewPosition = true;
          SlowSpeedForced = true;
          IsMovingByButton = true;
          WaitMotorStop = false;
        } // if (!IsMoving)
        else
        {
          if (PushCount > 1) PushCount--;
          else SlowSpeedForced = false;
        } // (else) if (!IsMoving)
        break;
      } // case :
      case SWINTRA:
      {
        if (!IsMoving)
        {
          TargetPosition = 0;
          GotoNewPosition = true;
          SlowSpeedForced = true;
          IsMovingByButton = true;
          WaitMotorStop = false;
        } // if (!IsMoving)
        else
        {
          if (PushCount > 1) PushCount--;
          else SlowSpeedForced = false;
        } // (else) if (!IsMoving)
        break;
      } // case :
      default:
      { // Arrêt moteur
        if (IsMovingByButton && !WaitMotorStop)
        {
          if(SlowSpeedForced) MotorHalt();                // Arrêt immédiat
          else
          {                                               // Arrêt après ralentissement
            if (GotoExtra)
            {
              TargetPosition = (MyFocuser.FocuserPosition + DeltaSpeed);
              if (TargetPosition > MyFocuser.MaxStep) TargetPosition = MyFocuser.MaxStep;
            } // if (GotoExtra)
            else
            {
              TargetPosition = (MyFocuser.FocuserPosition - DeltaSpeed);
              if (TargetPosition < 0) TargetPosition = 0;
            } // (else) if (GotoExtra)
            WaitMotorStop = true;
            StepMotorNumber = STEPMOTORINIT;
          } // (else) if(SlowSpeedForced)
        } // if (IsMovingByButton && !WaitMotorStop)
      } // default:
    } // switch (PB_Value)
    LastPB_Check = CurrentMillis;
  } // if ((CurrentMillis - LastPB_Check) >= PBCHECKRATE
} // Fin PB_IntraExtra()

//*************************************************************************//
//*************************************************************************//
//***                           GESTION EEPROM                          ***//
//*************************************************************************//
//*************************************************************************//

//*** PROCEDURES
void Reset_Myfocuser()
{
  MyFocuser.MotorSpeed = FACTORYMOTORSPEED;               // Vitesse du moteur 
  MyFocuser.ReverseDirection = FACTORYREVERSEDIR;         // Pas d'inversion des cmdes de rotation
  MyFocuser.BacklashLevel = FACTORYBACKLASH;              // Nbre de pas à effectuer lors d'un appuie bouton
  MyFocuser.MaxStep = FACTORYMAXSTEP;                     // Limite haute du focuser
  MyFocuser.FocuserPosition = FACTORYFOCUSPOS;            // Position courante
  MyFocuser.StepMode = FACTORYSTEPMODE;                   // Pas entier
  MyFocuser.CoilPwr = FACTORYCOILPWR;                     // Moteur reste alimenté
  MyFocuser.LimitSwitch = FACTORYSWITCH;                  // Détecteurs fin de course
  MyFocuser.TempWaitDelay = FACTORYTEMPWAITDELAY;         // Delai min entre 2 acquisition température
  TargetPosition = MyFocuser.FocuserPosition;
  TargetPositionStore = MyFocuser.FocuserPosition;
} // void Reset_MyFocuser()

// ............................................................... //
// .. Nom: ReadEepromInitValues()                               .. //
// .. Role: Recherche en EEPROM la présence de donnée valides   .. //
// ..       Si trouvés initialise la structure MyFocuser avec,  .. //
// ..       sinon utilise les valeurs par défaut et sauvegarde  .. //
// ..       la structure en EEPROM                              .. //
// .. Entrées: Constantes par défaut                            .. //
// .. Sortie: MyFocuser, CurrentAddr, DataEepromFound, DataSize .. //
// ..         WriteEepromNow,DataSize, NumberOfLocations        .. //
// ..         CurrentAddr, LocationIdx, DataEepromFound         .. //
// ................................................................//
void ReadEepromInitValues()
{
  int LocationIdx;                                        // Pour balayer emplacements structure en EEPROM
  
  // Initialisation des variables
  WriteEepromNow = false;                                 // Pas d'écriture en EEPROM

  // Calcul du nombre d'emplassement strucure en EEPROM
  DataSize = sizeof( MyFocuser );                         // Taille de la structure, Normalement 14 octets
  // pour AT328P = 1024 / taille des données = 73 emplacments
  NumberOfLocations = EEPROMSIZE / DataSize;              // Nbre d'emplacement structure 

  // Recherche de la présence de données valide en EEPROM
  CurrentAddr = 0;                                        // Pointe sur la structure en EEPRM
  LocationIdx = 0;
  DataEepromFound = false;                                // Données valides non trouvées
  do 
  { // Lit et rempli la structure
    EEPROM_readAnything(CurrentAddr,MyFocuser);
    // Vérifie que les données en EEPROM sont valides
    DataEepromFound = (MyFocuser.ValidData == DATAOK);
    if (!DataEepromFound) CurrentAddr += DataSize; 
    LocationIdx ++;
  } while ((LocationIdx < NumberOfLocations) && (!DataEepromFound));

  // Aucune données valide en EEPROM, Initialisation aux valeur usine
  // Sauvegarde en EEPROM
  if ((!DataEepromFound) || (MyFocuser.MaxStep < MINFOCUSERLIMIT) || (MyFocuser.MaxStep < MyFocuser.FocuserPosition))
  {
    // Valeurs Usines car structure valide non trouvées en EEPROM
    MyFocuser.ValidData = DATAOK;                         // Indique des données valides
    Reset_Myfocuser();
    CurrentAddr = 0;                                      // Sauvegarde à l'adresse 0
    EEPROM_writeAnything(CurrentAddr,MyFocuser);          // Sauvegarde en EEPROM
  }
} // void ReadEepromInitValues()

// ............................................................... //
// .. Nom: WriteMyFocuserEeprom()                               .. //
// .. Role: Dévalide l'emplacement de structure actuel          .. //
// ..       Sauvegarde des données à l'amplacement suivant      .. //
// ..       Evite de toujours écrire à la même adresse en EEPROM.. //
// .. Entrées: CurrentAddr, MyFocuser,DataSize,NumberOfLocations.. //
// .. Sortie: CurrentAddr,WriteEepromNow                        .. //
// ................................................................//
void WriteMyFocuserEeprom()
{
  MyFocuser.ValidData = DATAWRONG;                        // Dévalide l'empacement courant     
  EEPROM_writeAnything(CurrentAddr,MyFocuser);
  CurrentAddr += DataSize;                                // Passe à l'adresse suivante 
  if (CurrentAddr >= (NumberOfLocations * DataSize)) CurrentAddr = 0;
  //Mémorise les données
  MyFocuser.ValidData = DATAOK;                           // Données valides     
  EEPROM_writeAnything(CurrentAddr, MyFocuser);           // Mise en EEPROM
  WriteEepromNow = false;
  LCD_Store(false);
} // void WriteMyFocuserEeprom()

//*************************************************************************//
//*************************************************************************//
//***                           GESTION DS18B20                         ***//
//*************************************************************************//
//*************************************************************************//
// Declaration de l'objet sonde de temperature
OneWire oneWirech1(Ch1Temp);                              // Setup Sonde de temperature 1
// Passe la reference oneWire à la librairie DallasTemperature
DallasTemperature sensor1(&oneWirech1);                   // Transfert des donnés entre les 2 bibliothèques
// DB18B20 info
DeviceAddress TpAddress;                                  // Utilise pour envoyer la precision à un capteur particulier

//*** PROCEDURES
// ............................................................... //
// .. Nom: TempValDisplay2()                                    .. //
// .. Role: Initialisation du capteur DS18B20                   .. //
// ..       Initialsie les variables                            .. //
// ..       Cherche si une sonde est présente                   .. //
// ..       Oui: ajuste le précision de mesure                  .. //
// ..            Fait une première mesure                       .. //
// ..       Non: Fixe la température arbitrairement à 20°C      .. //
// .. Entrées: Aucune                                           .. //
// .. Sortie: ProbeTemp1Present, TempPreviousMillis,            .. //
// ..         Probe1TempVal                                     .. //
// ................................................................//
void InitTempProbe()
{
  unsigned char Idx;

  // Par défaut
  ProbeTemp1Present = false;                              // Capteur de température absent par défaut
  Probe1TempVal = 20.0;                                   // Met la température par défaut
  // Détection capteur
  sensor1.begin();                                        // Démarre le capteur de température 1 (sensor1)
  sensor1.getDeviceCount();                               // Retourne 1 si le capteur est présent
  for (Idx = 0; Idx < MAXPROBES; Idx++)                   // Balaye les n° de capteur
  {                                                       // Cherche son adresse
    if (sensor1.getAddress(TpAddress, Idx)) 
    {
      ProbeTemp1Present = true;                           // Un capteur est présent
      sensor1.setResolution(TpAddress, TEMP_PRECISION);   // Résolution sur 10bit 0.25degC
    } // if (sensor1.getAddress(TpAddress, Idx))
  } // for (Idx = 0; Idx < MAXPROBES; Idx++)
  // Fait une acquisition de température maintenant pour être prêt à renvoyer une valeur valide
  // en cas de requête.
  if (ProbeTemp1Present)                                  // Récupère la température courante
  {
    sensor1.requestTemperatures();
    TempPreviousMillis = millis();
  } // if (ProbeTemp1Present)
} // void InitTempProbe()

// ............................................................... //
// .. Nom: boolean StartTemperatureAcquisition()                .. //
// .. Role: Démarre une acquisition si il n'y en a pas déjà en  .. //
// ..       cours. Retourne vrai dans ce cas. Si le moteur tour-.. //
// ..       ne ou la sonde absente, retourne faux               .. //
// .. Entrées: ProbeTemp1Present, TempPreviousMillis            .. //
// .. Sortie: Probe1TempVal, IsMoving                           .. //
// .. Sortie: StartTemperatureAcquisition                       .. //
// ................................................................//
boolean StartTemperatureAcquisition()
{
  unsigned long CurrentMillis = millis();
  
  if (ProbeTemp1Present && (!IsMoving))
  { // Durée d'acquisition écoulé?
    if ((CurrentMillis - TempPreviousMillis) > TEMPACQUISITIONDELAY)
    { // Delai d'acquisition écoulé : lecture des valeurs
        sensor1.requestTemperatures();
        TempPreviousMillis = CurrentMillis;
    }
    return true;                                          // Acquisition en cours
  }
  else return false;                                      // Pas d'acquisition
} // boolean StartTemperatureAcquisition()

// ............................................................... //
// .. Nom: boolean ReadTemperature()                            .. //
// .. Role: Si le temps de conversion est écoulé récupère la    .. //
// ..       valeur de température de la sonde.                  .. //
// ..       Renvoie vrai si la conversion est terminée.Affiche  .. //
// ..       la valeur sur le LCD.                               .. //
// .. Entrées: ProbeTemp1Present, TempPreviousMillis            .. //
// .. Sortie: Probe1TempVal, ReadTemperature()                  .. //
// ................................................................//
boolean ReadTemperature() 
{
  unsigned long CurrentMillis = millis();

  if (ProbeTemp1Present)
  { // Durée d'acquisition écoulé?
    if ((CurrentMillis - TempPreviousMillis) > TEMPACQUISITIONDELAY)
    { // Delai d'acquisition écoulé : lecture des valeurs
      // Acquisition température probe en °C
      Probe1TempVal = sensor1.getTempCByIndex(0);
      // Affiche sur le LCD
      LCD_Temperature();
      return true;                                          // Conversion terminée
    }
    else return false;                                      // Conversion en cours
  }
  else return true;                                         // valeur par défaut
} // boolean ReadTemperature() 

//*************************************************************************//
//*************************************************************************//
//***                     MOTEUR PAS A PAS et DRV8825                   ***//
//*************************************************************************//
//*************************************************************************//

// ............................................................... //
// .. Nom: CalcNewLimits(unsigned char NewStepMode)             .. //
// .. Role: Calcul les nouvelles limites en fonction du mode    .. //
// ..       µ-pas sélectionné. recalcul aussi la position.      .. //
// .. Entrées:  NewStepMode, MAXFOCUSERLIMIT                    .. //
// .. Sortie: MyFocuser.StepMode, MyFocuser.MaxStep,            .. //
// ..         MyFocuser.FocuserPosition, TargetPosition,        .. //
// ..         TargetPositionStore, GotoNewPosition              .. //
// ................................................................//
void CalcNewLimits(unsigned char NewStepMode)
{  
  while (MyFocuser.StepMode != NewStepMode)
  {
    if (MyFocuser.StepMode < NewStepMode)
    {
      MyFocuser.StepMode = MyFocuser.StepMode << 1;         // Multiplication par 2 (décalage 1 bit)
      MyFocuser.MaxStep = MyFocuser.MaxStep << 1;
      MyFocuser.FocuserPosition = MyFocuser.FocuserPosition << 1;
      if (MyFocuser.MaxStep > MAXFOCUSERLIMIT) MyFocuser.MaxStep = MAXFOCUSERLIMIT;
      if (MyFocuser.FocuserPosition > MyFocuser.MaxStep) MyFocuser.FocuserPosition = MyFocuser.MaxStep;
    } else if (MyFocuser.StepMode > NewStepMode)
          {
            MyFocuser.StepMode = MyFocuser.StepMode >> 1;   // Division par 2 (décalage 1 bit)
            MyFocuser.MaxStep = MyFocuser.MaxStep >> 1;
            MyFocuser.FocuserPosition = MyFocuser.FocuserPosition >> 1;
            if (MyFocuser.MaxStep < MINFOCUSERLIMIT) MyFocuser.MaxStep = MINFOCUSERLIMIT;
            if (MyFocuser.FocuserPosition > MyFocuser.MaxStep) MyFocuser.FocuserPosition = MyFocuser.MaxStep;
          } // Fin else if (MyFocuser.StepMode > NewStepMode)
  } // Fin while (MyFocuser.StepMode != NewStepMode)
  TargetPosition = MyFocuser.FocuserPosition;
  TargetPositionStore = MyFocuser.FocuserPosition;
  GotoNewPosition = false;
} // void CalcNewLimits(NewStepMode)

// ............................................................... //
// .. Nom: CalcBacklashOffset()                                 .. //
// .. Role: Calcul le nombre de pas pour rattraper le backlash  .. //
// .. Entrées: MyFocuser.StepMode,MyFocuser.BacklashLevel       .. //
// ..          MOTORSTEP, MOTORDIV, BACKLASHMED, BACKLASLOW     .. //
// .. Sortie: BacklashOffset                                    .. //
// ................................................................//
void CalcBacklashOffset()
{
  // Nbre de pas pour faire un tour moteur
  BacklashOffset = MyFocuser.StepMode * MOTORSTEP * MOTORDIV;
  if (MyFocuser.BacklashLevel == BACKLASHMED) BacklashOffset = (BacklashOffset >> 1);
  if (MyFocuser.BacklashLevel == BACKLASHLOW) BacklashOffset = (BacklashOffset >> 2);
} // CalcBacklashOffset()

// ............................................................... //
// .. Nom: SetStepMode(unsigned char NewStepMode)               .. //
// .. Role: Paramètre le mode pas/micro-pas du DRV8825          .. //
// .. Entrées:  MyFocuser.StepMode                              .. //
// .. Sortie: MyM0, MyM1, MyM2                                  .. //
// ................................................................//
void SetStepMode(unsigned char NewStepMode)
{
  if (!IsMoving)
  {
    // Calcul des nlle limites et met à jour Myfocuser.StepMode
    CalcNewLimits(NewStepMode);
    // Calcul du backclash
    CalcBacklashOffset();
    // Programmation du DRV8825
    switch (MyFocuser.StepMode)
    {
      case 2:      // 1/2
      {
        digitalWrite(MyM0, 1);
        digitalWrite(MyM1, 0);
        digitalWrite(MyM2, 0);
        SpeedCal = MOTORSPEEDAUTOCALuS2;
        break;
      } // case 2:
      case 4:     // 1/4
      {
        digitalWrite(MyM0, 0);
        digitalWrite(MyM1, 1);
        digitalWrite(MyM2, 0);
        SpeedCal = MOTORSPEEDAUTOCALuS4;
        break;  
      } // case 4:
      case 8:     // 1/8
      {
        digitalWrite(MyM0, 1);
        digitalWrite(MyM1, 1);
        digitalWrite(MyM2, 0);
       SpeedCal = MOTORSPEEDAUTOCALuS8;
        break;
      } // case 8:
      case 16:    // 1/16
      {
        digitalWrite(MyM0, 0);
        digitalWrite(MyM1, 0);
        digitalWrite(MyM2, 1);
       SpeedCal = MOTORSPEEDAUTOCALuS16;
        break;
      } // case 16:
      case 32:    // 1/32
      {
        digitalWrite(MyM0, 1);
        digitalWrite(MyM1, 0);
        digitalWrite(MyM2, 1);
       SpeedCal = MOTORSPEEDAUTOCALuS32;
        break;
      } // case 32:
      default:      // Pas Entier
      {
        digitalWrite(MyM0, 0);
        digitalWrite(MyM1, 0);
        digitalWrite(MyM2, 0);
        MyFocuser.StepMode = 1;
       SpeedCal = MOTORSPEEDAUTOCALuS1;
      } // default:
    } // switch (Myfocuser.StepMode)
  } // if (!IsMoving)
} // void SetStepMode(unsigned char NewStepMode)

// ............................................................... //
// .. Nom: DisableCoilOutput()                                  .. //
// .. Role: Desactive l'alimentation des bobines moteur         .. //
// ..       Si l'alimentation est coupé en mode micro-pas le    .. //
// ..       moteur se positionne sur le pas entier le plus      .. //
// ..       proche. A proscrire donc en mode micro-pas          .. //
// .. Entrées: Aucune                                           .. //
// .. Sortie: MyEnable                                          .. //
// ................................................................//
void DisableCoilOutput() 
{
  digitalWrite(MyEnable, HIGH);
} // void DisableCoilOutput()

// ............................................................... //
// .. Nom: EnableCoilOutput()                                   .. //
// .. Role: Active l'alimentation des bobines moteur            .. //
// ..       Garder l'alimentation en mode micro-pas             .. //
// ..       Régler la limitation en courant                     .. //
// .. Entrées: Aucune                                           .. //
// .. Sortie: MyEnable                                          .. //
// ................................................................//
void EnableCoilOutput() 
{
  digitalWrite(MyEnable, LOW);
} // void EnableCoilOutput()

// ............................................................... //
// .. Nom: L_SwIntra()                                          .. //
// .. Role: Renvoi vrai si le capteur de fin de course intra-   .. //
// ..       focale est présent et activé                        .. //
// .. Entrées: MyFocuser.LimitSwitch, LimitIntra                .. //
// .. Sortie: L_SwIntra                                         .. //
// ................................................................//
boolean L_SwIntra()
{
  if (MyFocuser.LimitSwitch == LIMITSWITCHDISABLE) return false;
  else if (MyFocuser.LimitSwitch == LIMITSWITCHACTIVLOW) return (digitalRead(LimitIntra) == LOW);
        else return (digitalRead(LimitIntra) == HIGH);
} // boolean L_SwIntra()

// ............................................................... //
// .. Nom: L_SwExtra()                                          .. //
// .. Role: Renvoi vrai si le capteur de fin de course extra-   .. //
// ..       focale est présent et activé                        .. //
// .. Entrées: MyFocuser.LimitSwitch, LimitExtra                .. //
// .. Sortie: L_SwExtra                                         .. //
// ................................................................//
boolean L_SwExtra()
{
  if (MyFocuser.LimitSwitch == LIMITSWITCHDISABLE) return false;
  else if (MyFocuser.LimitSwitch == LIMITSWITCHACTIVLOW) return (digitalRead(LimitExtra) == LOW);
        else  return (digitalRead(LimitExtra) == HIGH);
} // boolean L_SwExtra()

// ............................................................... //
// .. Nom: MotorGotoExtra()                                     .. //
// .. Role: Configure le DRV8825 pour un mouvement vers l'extra .. //
// ..       focale                                              .. //
// .. Entrées: MyFocuser.ReverseDirection                       .. //
// .. Sortie: LED_ToIntra,LED_ToExtra,MyDir                     .. //
// ................................................................//
void MotorGotoExtra()
{
  analogWrite(LED_ToIntra, 0);                              // Eteint la LED Intrafocale
  analogWrite(LED_ToExtra, 1023);                           // Allume la LED Extrafocale
  if (MyFocuser.ReverseDirection) digitalWrite(MyDir, HIGH);// Choix du sens de rotation
  else digitalWrite(MyDir, LOW);
} // void MotorGotoExtra()

// ............................................................... //
// .. Nom: MotorGotoIntra()                                     .. //
// .. Role: Configure le DRV8825 pour un mouvement vers l'extra .. //
// ..       focale                                              .. //
// .. Entrées: MyFocuser.ReverseDirection                       .. //
// .. Sortie: LED_ToIntra,LED_ToExtra,MyDir                     .. //
// ................................................................//
void MotorGotoIntra()
{
  analogWrite(LED_ToIntra, 1023);                           // Allume la LED Intrafocale
  analogWrite(LED_ToExtra, 0);                              // Eteint la LED Extrafocale
  if (MyFocuser.ReverseDirection) digitalWrite(MyDir, LOW); // Choix du sens de rotation
  else digitalWrite(MyDir, HIGH);
} // void MotorGotoIntra()

// ............................................................... //
// .. Nom: MotorPulseCal()                                      .. //
// .. Role: Génère une impulsion pour faire tourner le moteur et.. //
// ..       met à jour la variable position du focuser. Stoppe  .. //
// ..       Fin de course atteint ou appuie bouton en mode      .. //
// ..       manuel. EN MODE CALIBRATION                         .. //
// .. Entrées: MyFocuser.FocuserPosition, AutoCal,MyStep        .. //
// ..          STEPONTIMEuS                                     .. //
// .. Sortie: AutoCalEnd                                        .. //
// ................................................................//
void MotorPulseCal()
{
  // Teste les fins de course
  // Filtrage des impulsions intenpestives
  if (EnableLSwitch && (L_SwExtra() || L_SwIntra())) FilterCount--;
  else FilterCount = MAXBYPASSPULSE;
  AutoCalEnd = (FilterCount == 0);
  AutoCalEnd = AutoCalEnd || ((AutoCal == MANUALCAL) && (ReadPB_Switches() != NOSWITCH));
  if (AutoCalEnd) Timer1.stop();
  else if (FilterCount == MAXBYPASSPULSE)
        { // Génère l'impulsion
          MyFocuser.FocuserPosition++;
          digitalWrite(MyStep, 1);          
          delayMicroseconds(STEPONTIMEuS);
          digitalWrite(MyStep, 0);
        } // (else) if (AutoCalEnd) Timer1.stop();
} // void MotorPulseCal()

// ............................................................... //
// .. Nom: FocuserCalibration(boolean Auto)                     .. //
// .. Role: configure automatiquement les valeurs de fin de     .. //
// ..      course, la position du focuser. Detecte si il faut   .. //
// ..      inversé ou pas le sens de rotation du moteur et si   .. //
// ..      il faut alimenté en permanence les bobines moteur en .. //
// ..      utilisant les capteurs de fin de course              .. //
// ..      il faut avoir appeler:                               .. //
// ..                   SetStepMode(unsigned char NewStepMode)  .. //
// .. Entrées: Capteurs fin de course, MyFocuser.StepMode,Auto  .. //
// .. Sortie: MyFocuser.CoilPwr/ReverseDirection/FocuserPosition.. //
// ..         /MaxStep/LimiSwitch                               .. //
// ................................................................//
void FocuserCalibration()
{
  unsigned char StepCal;
  boolean ReverseDirectionStore;

  Timer1.attachInterrupt(MotorPulseCal,SpeedCal);
  Timer1.stop(); 

  // CALIBRATION
  // Attends que le focuser ne soit plus en buté en mode auto
  DisableCoilOutput();                                      // Moteur plus alimenté permet les mouvements manuels
  // Autorise l'utilisation des fins de course
  EnableLSwitch = true;
  // Attente démarrage calibration
  do
  {
    // Attend crayford plus en buté
    while (L_SwIntra() || L_SwExtra())
    {
      lcd.setCursor(8,1);
      lcd.print("Fin-course");
      analogWrite(Buzzer, BUZZERSTATE);
      TestPause200();
      TestPause200();
      lcd.setCursor(8,1);
      lcd.print("          ");
      analogWrite(Buzzer, 0);
      TestPause200();
      TestPause200();
    } // while (L_SwIntra() || L_SwExtra())
    // Confirmation démarrage
    // Attend boutons relachés
    while (ReadPB_Switches() != NOSWITCH) {TestPause200();}
    TestPause1000();
    // Attends validation
    do
    {
      lcd.setCursor(8,1);
      lcd.print("Start?");
      analogWrite(Buzzer, BUZZERSTATE);
      TestPause200();
      TestPause200();
      lcd.setCursor(8,1);
      lcd.print("      ");
      analogWrite(Buzzer, 0);
      TestPause200();
      TestPause200();
    } while (ReadPB_Switches() != SWVALID);
    // Attend boutons relachés
    while (ReadPB_Switches() != NOSWITCH) {TestPause200();}
  } while (L_SwIntra() || L_SwExtra());
  // Initialisation Procédure de Calibration
  // Réglage du paramètre d'alimentation
  MyFocuser.CoilPwr = true;
  // Initialisation paramétrage sens de rotation
  ReverseDirectionStore = MyFocuser.ReverseDirection;
  MyFocuser.ReverseDirection = false;
  // Alimente le moteur
  EnableCoilOutput();
  // Passe à l'etape de recherche butée basse
  StepCal = STEPCAL1;
  while (StepCal  != STEPCALEXIT)
  { 
    switch (StepCal)
    {
      case STEPCAL1: // Recherche de la première butée
      {
        // Attend boutons relachés en mode manuel
        while ((AutoCal == MANUALCAL) && (ReadPB_Switches() != NOSWITCH)) {TestPause200();}
        // Cherche la première butée
        MyFocuser.FocuserPosition = 0;
        AutoCalEnd = false;           
        MotorGotoIntra();    // Rotation dans un sens
        FilterCount = MAXBYPASSPULSE;
        Timer1.start();
        while (!AutoCalEnd && (StepCal != STEPCALERR1))
        {
          if (MyFocuser.FocuserPosition > MAXFOCUSERLIMIT)  StepCal = STEPCALERR1;
          LCD_MotorPositionCal();
          TestPause200();
        } // while (!AutoCalEnd && (StepCal != STEPCALERR1))
        if (AutoCalEnd) StepCal = STEPCAL2; 
        break;
      } // case STEPCAL1:
      case STEPCAL2: // Recherche de la 2nde butée
      {
        MotorGotoExtra();                                   // Inverse le sens de rotation du moteur
        // Repos 1 seconde
        TestPause1000();
        // Attend boutons relachés en mode manuel
        while ((AutoCal == MANUALCAL) && (ReadPB_Switches() != NOSWITCH)) {TestPause200();}
        
        // focuser se déplace de MINFOCUSERLIMIT (limite minimum)
        MyFocuser.FocuserPosition = 0;
        AutoCalEnd = false;
        EnableLSwitch = false;                              // Désactive les fins de course le temps que le moteur se décolle
        TestPause200();
        FilterCount = MAXBYPASSPULSE;
        Timer1.start();
        while (!AutoCalEnd && (MyFocuser.FocuserPosition < MINFOCUSERLIMIT)) 
        {
          LCD_MotorPositionCal();
          TestPause200();
          // Réactive les fins de course
          if (!EnableLSwitch && (MyFocuser.FocuserPosition > BYPASSFOCUSERLIMIT)) EnableLSwitch = true;
        } // while (!AutoCalEnd && (MyFocuser.FocuserPosition < MINFOCUSERLIMIT)) 
        
        if (MyFocuser.FocuserPosition < MINFOCUSERLIMIT) StepCal = STEPCALERR2;
        else
        { // Message Nbre de pas minimum effectué
          // Cherche la 2nde butée
          while (!AutoCalEnd && (StepCal != STEPCALERR3))
          {
            if (MyFocuser.FocuserPosition > MAXFOCUSERLIMIT)  StepCal = STEPCALERR3;
            TestPause200();
            LCD_MotorPositionCal();
          } // while (!AutoCalEnd && (StepCal != STEPCALERR3))
          if (AutoCalEnd) StepCal = STEPCAL3;
        } // (else) if (MyFocuser.FocuserPosition < MINFOCUSERLIMIT) StepCal = STEPCALERR2;
        break;
      } // case STEPCAL2
      case STEPCAL3:  // Confirmation postion focuser
      {
        // Message calibration terminée
        // Mémorise l'amplitude du déplacement
        MyFocuser.MaxStep = MyFocuser.FocuserPosition;
        delay(3000);
        // Confirmation position focuser
        lcd.setCursor(8,1);
        lcd.print("Pos.Focus?");
        // Attente choix
        while ((ReadPB_Switches() != SWPLUS)) {TestPause200();}
        lcd.setCursor(14,1);
        lcd.print("tra?");
        do
        {
          lcd.setCursor(12,1);
          if (MyFocuser.ReverseDirection) lcd.print("In");
          else lcd.print("Ex");
          // Attend relachement bouton
          while ((ReadPB_Switches() != NOSWITCH)) {TestPause200();}
          // Attend appui
          while ((ReadPB_Switches() == NOSWITCH)) {TestPause200();}
          if (ReadPB_Switches() == SWPLUS) MyFocuser.ReverseDirection = (!MyFocuser.ReverseDirection);
         } while ((ReadPB_Switches() != SWVALID));
        
        // Paramétrage du sens du moteur
        StepCal = STEPCALEXIT;
        if (MyFocuser.ReverseDirection)
        { // On est parti vers l'intra-focale au lieu de l'extra focale
          MyFocuser.FocuserPosition = 0;
          TargetPosition = 0;
          TargetPositionStore = 0;
          if ((MyFocuser.LimitSwitch != LIMITSWITCHDISABLE) && L_SwExtra()) StepCal = STEPCALERR4;
        } // if (MyFocuser.ReverseDirection)
        else
        { // On est parti dans le bon sens, vers l'extra-focale
          TargetPosition = MyFocuser.FocuserPosition;
          TargetPositionStore = MyFocuser.FocuserPosition;
          if ((MyFocuser.LimitSwitch != LIMITSWITCHDISABLE) && L_SwIntra()) StepCal = STEPCALERR4;
        } // (else) if (MyFocuser.ReverseDirection)

        if (StepCal == STEPCALEXIT) 
        { // Calibration Ok
          lcd.setCursor(8,1);
          lcd.print("Cal. Ok!  ");
        }
        break;
      } // case STEPCAL3:
      default:
      {
        Timer1.stop();
        // Chargement valeurs par défauts
        MyFocuser.MaxStep = FACTORYMAXSTEP;                 // Limite haute du focuser
        MyFocuser.FocuserPosition = FACTORYFOCUSPOS;        // Position courante
        TargetPosition = FACTORYFOCUSPOS;                   // Position courante
        TargetPositionStore = FACTORYFOCUSPOS;              // Position courante
        MyFocuser.ReverseDirection = ReverseDirectionStore; // Remet le sens du moteur initial
        // Message d'erreur
        lcd.setCursor(8,1);
        lcd.print("Err.      ");
        lcd.setCursor(13,1);
        lcd.print((StepCal-STEPCAL3));
        // Sortie de la boucle
        StepCal = STEPCALEXIT;
        break;
      } // default:
    } // switch (StepCal)
  } // While (StepCal  != STEPCALEXIT)
  // Rétabli le timer normal
  Timer1.detachInterrupt();
  analogWrite(LED_ToIntra, 0);                              // Eteint la LED Intrafocale
  analogWrite(LED_ToExtra, 0);                              // Eteint la LED Extrafocale
  // Laisse le message affiché 5s
  delay(5000);
} // void FocuserCalibration()

// ............................................................... //
// .. Nom: SetTimerMotor()                                      .. //
// .. Role: Ajuste le timer en fonction de la vitesse moteur    .. //
// ..       choisie.                                            .. //
// .. Entrées: MOTORSPEEDSTARTSTOPmS, MyFocuser.MotorSpeed      .. //
// .. Sortie: -                                                 .. //
// ................................................................//
void SetTimerMotor()
{
  long MotorPeriod;
  
  MotorPeriod = (((long)MyFocuser.MotorSpeed) << 10);       // Multiplie la vistesse par 1024 (consersion approx. ms->us)
  Timer1.setPeriod(MotorPeriod);                            // Periode en µs
  Timer1.stop();
} // void SetTimerMotor()

// ............................................................... //
// .. Nom: MotorHalt()                                          .. //
// .. Role: Stop le moteur à la position courante.              .. //
// .. Entrées: MyFocuser.FocuserPosition                        .. //
// .. Sortie: TargetPosition,TargetPositionStore,               .. //
// ..         StepMotorNumber                                   .. //
// ................................................................//
void MotorHalt()
{
  Timer1.stop();
  TargetPosition = MyFocuser.FocuserPosition;
  TargetPositionStore = MyFocuser.FocuserPosition;
  StepMotorNumber = STEPMOTOREND;
} // void MotorHalt()

// ............................................................... //
// .. Nom: InitSpeed()                                          .. //
// .. Role: Initialise les variables pour gérer l'accélération  .. //
// ..       et la décélération                                  .. //
// .. Entrées: SpeedDiff                                        .. //
// .. Sortie: CurrentWait, CountWait, MotorStopping             .. //
// ................................................................//
void InitSpeed()
{
  // Nbre de pas pour accélérer et ralentir
  switch(MyFocuser.MotorSpeed)
  {
    case MOTORSPEEDSLOWmS:
    { // Vitesse lente
      CurrentWait = STEPSTARTSTOPSLOW;
      PushCountInit = PUSHCOUNTSLOW;
      break;
    } // case MOTORSPEEDSLOWmS:
    case MOTORSPEEDMEDmS:
    { // Vitesse moyenne
      CurrentWait = STEPSTARTSTOPMED;
      PushCountInit = PUSHCOUNTMED;
      break;
    } // case MOTORSPEEDMEDmS:
    case MOTORSPEEDFASTmS:
    { // Vitesse rapide
      CurrentWait = STEPSTARTSTOPFAST;
      PushCountInit = PUSHCOUNTFAST;
    } // case MOTORSPEEDFASTmS:
  } // switch(MyFocuser.MotorSpeed)
  DeltaSpeed = CurrentWait;
  CountWait = 0;
  MotorStopping = false;
} // InitSpeed()

// ............................................................... //
// .. Nom: MotorPulse()                                         .. //
// .. Role: Génère une impulsion pour faire tourner le moteur et.. //
// ..       met à jour la variable position du focuser. Stoppe  .. //
// ..       le timer si la consigne est atteinte. Gére les fins .. //
// ..       de course.                                          .. //
// .. Entrées: MyFocuser.FocuserPosition, GotoExtra,            .. //
// ..          TargetPosition, STEPONTIMEuS                     .. //
// .. Sortie: MyFocuser.FocuserPosition                         .. //
// ................................................................//
void MotorPulse()
{
  if (CountWait == 0)
  {
    // Gestion de la vitesse 
    if (MotorStopping) CurrentWait++;                       // Déccélération
    else if ((CurrentWait != 0) && !SlowSpeedForced) CurrentWait--;// Accélération
    CountWait = CurrentWait;                                // Pour la gestion de la vitesse
    
    if (StepMotorNumber == STEPMOTOREXTRA)
    { // Déplacement en Extrafocal
      if (L_SwExtra())
      {
        FilterCount--;
        if (FilterCount == 0) MotorHalt();                  // Fin de course Extrafocal atteint
      }
      else
      { // Génération impulsion + Maj variable position
        FilterCount = MAXBYPASSPULSE;
        MyFocuser.FocuserPosition++;
        digitalWrite(MyStep, 1);          
        delayMicroseconds(STEPONTIMEuS);
        digitalWrite(MyStep, 0);
        if (TargetPosition <= MyFocuser.FocuserPosition) Timer1.stop();// Consigne atteinte
      } // (else) if L_SwExtra() TargetPosition = MyFocuser.FocuserPosition;
    } // if (StepMotorNumber == STEPMOTOREXTRA)
    else
    { // Déplacement en Intrafocal
      if (L_SwIntra())
      {
        FilterCount--;
        if (FilterCount == 0) MotorHalt();                  // Fin de course Intrafocal atteint
      }
      else
      { // Génération impulsion + Maj variable position
        FilterCount = MAXBYPASSPULSE;
        MyFocuser.FocuserPosition--;
        digitalWrite(MyStep, 1);          
        delayMicroseconds(STEPONTIMEuS);
        digitalWrite(MyStep, 0);
        if (TargetPosition >= MyFocuser.FocuserPosition) Timer1.stop();// Consigne atteinte
      } // if L_SwIntra() TargetPosition = MyFocuser.FocuserPosition;
    } // (else) if (StepMotorNumber == STEPMOTOREXTRA)
  } // if (CountWait == 0)
  else CountWait--;
} // void MotorPulse()

// ............................................................... //
// .. Nom: MotorControl()                                       .. //
// .. Role: Initialise les variables pour la rotation. Gère le  .. //
// ..       sens de rotation et la gestion du backlash. Ré-ini  .. //
// ..       tialise les variables à la fin de la rotation       .. //
// .. Entrées: StepMotorNumber,GotoNewPosition,TargetPosition   .. //
// ..          DeltaSpeed, LED_Intra, LED_Extra                 .. //
// .. Sortie: StepMotorNumber, IsMoving, TargetPosition         .. //
// ..         TargetPositionStore, GotoExtra, DeltaPosition     .. //
// ..         MotorStopping, IsMovingByButton, WaitMotorStop    .. //
// ..         GotoNewPosition, SlowSpeedForced, PushCount       .. //
// ..         PreviousMillisEeprom, LastWakeeUpLCD,             .. //
// ..         WriteEepromNow                                    .. //
// ................................................................//
void MotorControl()
{
  switch (StepMotorNumber)
  {
    case STEPMOTORINIT: // Moteur à l'arrêt
    {
      if (GotoNewPosition)
      {
        IsMoving = (TargetPosition != MyFocuser.FocuserPosition);
        if  (IsMoving) 
        { // Gestion de l'écran
          LCD_Status(LCDON);                              // Allume l'écran LCD
          LCD_MotorTaget();                               // Affiche la consigne
          // Gestion des variables
          TargetPositionStore = TargetPosition;           // Mémorise la consigne initiale
          EnableCoilOutput();                             // Alimente le moteur
          // Détermine le sens de rotation
          GotoExtra = (TargetPosition > MyFocuser.FocuserPosition);
          if (GotoExtra)
          {
            MotorGotoExtra();
            StepMotorNumber =  STEPMOTOREXTRA;            // Passage à l'étape suivante
            FilterCount = MAXBYPASSPULSE;
            Timer1.start();                               // Active le Timer1
          }
          else
          {
            // Gestion du Backlash, re-calcul de la consigne
            if (MyFocuser.BacklashLevel != BACKLASHOFF)
            { // Recalcul la consigne pour gérer le backlash
              TargetPosition = (TargetPosition - BacklashOffset);
              if (TargetPosition < 0) TargetPosition = 0;
            } // if (MyFocuser.BacklashLevel != BACKLASHOFF)
            MotorGotoIntra();        
            StepMotorNumber =  STEPMOTORINTRA;
            FilterCount = MAXBYPASSPULSE;
            Timer1.start();                               // Active le Timer1
          }
        }
        else  GotoNewPosition = false;
      } // if (GotoNewPosition)
      break;
    } // case STEPMOTORINIT
    case STEPMOTORINTRA: // Moteur Tourne en intrafocal
    {
      // Gestion de la vitesse moteur
      DeltaPosition = (MyFocuser.FocuserPosition - TargetPosition);
      MotorStopping = (DeltaPosition < DeltaSpeed);

      // Passage à l'étape suivante
      if (TargetPosition == MyFocuser.FocuserPosition) StepMotorNumber =  STEPMOTORPAUSE;// Passage à l'étape suivante
      
      break;
    } // case STEPMOTORINTRA :
    case STEPMOTORPAUSE:
    {
      if (MyFocuser.BacklashLevel == BACKLASHOFF) StepMotorNumber =  STEPMOTOREND;
      else
      {
        TargetPosition = TargetPositionStore;             // Recharge la consigne initiale
        if (TargetPosition == 0) StepMotorNumber =  STEPMOTOREND;
        else
        { 
          InitSpeed();
          TestPause200();                                 // Marque une pause
          StepMotorNumber =  STEPMOTOREXTRA;              // Passage à l'étape suivante
          MotorGotoExtra();
          FilterCount = MAXBYPASSPULSE;
          Timer1.start();                                 // Active le Timer1
        } // (else) if (TargetPosition == 0) StepMotorNumber =  STEPMOTOREND;
      } // (else) if (MyFocuser.BacklashLevel == BACKLASHOFF) StepMotorNumber =  STEPMOTOREND;
      break;
    } // case STEPMOTORPAUSE:
    case  STEPMOTOREXTRA:
    {
      // Gestion de la vitesse moteur
      DeltaPosition = (TargetPosition - MyFocuser.FocuserPosition);
      MotorStopping = (DeltaPosition < DeltaSpeed);

      // Passage à l'étape suivante
      if (TargetPosition == MyFocuser.FocuserPosition) StepMotorNumber =  STEPMOTOREND;
      break;
    } // case STEPMOTOREXTRA:
    case STEPMOTOREND: // Moteur s'arrête
    { // Gestion des variables
      IsMoving = false;                                   // Moteur arrété
      IsMovingByButton = false;
      GotoNewPosition = false;                            // Pas d'activation du moteur
      WaitMotorStop = false;                              // Gestion consigne: pour les boutons
      SlowSpeedForced = false;                            // Gestion vitesse: pour les boutons
      MotorStopping = false;                              // Gestion vitesse
      InitSpeed();
      PushCount = PushCountInit;                          // Gestion vitesse : pour les boutons
      analogWrite(LED_ToIntra, 0);                        // Eteint la LED Intrafocale
      analogWrite(LED_ToExtra, 0);                        // Eteint la LED Extrafocale
      // Gestion LCD
      LCD_MotorPosition();                                // Affichage position finale
      LCD_MotorTaget();                                   // Affiche la consigne
      if (!MyFocuser.CoilPwr) DisableCoilOutput();        // Remet l'alimentation bobines moteur comme programmé      
       // Lancement tempo mémorisation EEPROM
      WriteEepromNow = true;                              // Maj EEPROM nécessaire
      PreviousMillisEeprom = millis();                    // Tempo mémorisation
      LastWakeeUpLCD = PreviousMillisEeprom;              // Tempo extinction LCD
      StepMotorNumber = STEPMOTORINIT;                    // Pour le prochain démarrage
    } // case STEPMOTOREND:
  } // switch (StepMotorNumber)
} // Void MotorControl()

//*************************************************************************//
//*************************************************************************//
//***                         PORT SERIE & ASCOM                        ***//
//*************************************************************************//
//*************************************************************************//
boolean EndOfCmd = false;                                 // Reçu caractère fin commande '#'
int Idx = 0;                                              // Index caractère dans chaine de commande

#define MAXCOMMAND 8                                      // Nbre caractères max commande
char LineRx[MAXCOMMAND];                                  // Caractères reçus sauf ":" et "#"

// ............................................................... //
// .. Nom: serialEvent()                                        .. //
// .. Role: Déclenché par l'arrivée d'un caractère sur le port  .. //
// ..       série.                                              .. //
// ..       Lit les données stockée dans le buffer réception du .. //
// ..       port série. Une commande débute par ":" et fini par .. //
// ..       "#". Le buffer LineRx contient les caractères reçus .. //
// ..       sauf ":" et "#"                                     .. //
// .. Entrées: Aucune                                           .. //
// .. Sortie: LineRx, EndOfCmd, Idx                             .. //
// ................................................................//
void serialEvent() 
{
  char InChar;                                            // Caractère reçu sur port serie

  while (Serial.available() && !EndOfCmd) 
  {
    InChar = Serial.read();                               // Lit le caractère sur le port série
    if (InChar != '#' && InChar != ':') 
    {
      LineRx[Idx++] = InChar;                             // Stocke tout sauf ":" et "#"
      if (Idx >= MAXCOMMAND)                              // Vérifie Nbre caractères reçu Ok
      {
        Idx = MAXCOMMAND - 1;
      } // if (Idx >= MAXCOMMAND)
    } // if (InChar != '#' && InChar != ':')
    else 
    {
      if (InChar == '#') 
      {
        EndOfCmd = true;                                  // Fin de réception de la commande
        Idx = 0;
      } // if (InChar == '#') 
    } // (else) if (InChar != '#' && InChar != ':')
  } // while (Serial.available() && !EndOfCmd)
} // void serialEvent() 

// ............................................................... //
// .. Nom: HexStr2Long                                          .. //
// .. Role: Convertie un tableau de caractères hexadécimaux     .. //
// ..       en un entier long                                   .. //
// .. Entrées: *LineRx                                          .. //
// .. Sortie: HexStr2Long                                       .. //
// ................................................................//
long HexStr2Long(char *LineRx) 
{
  long Ret = 0;

  Ret = strtol(LineRx, NULL, 16);                         // string To long
  return (Ret);
} // void HexStr2Long(char *LineRx)

//*** TRAITEMENT DES COMMANDES ASCOM
// ............................................................... //
// .. Nom: ProcessCommand()                                     .. //
// .. Role: Interpréteur de commandes.                          .. //
// ..       Interprète et répond aux commandes sur port série   .. //
// .. Entrées: *LineRx,ProgramName,ProgramVersion               .. //
// .. Sortie: GotoNewPosition,StepMotorNumber,TargetPosition    .. //
// ..         MyFocuser.FocuserPosition,MyFocuser,WriteEepromNow.. //
// ..         PreviousMillisEeprom,TempOffsetVal,               .. //
// ..         MyFocuser.MotorSpeed                              .. //
// ..         StepDelay,MaxIncrement,MyFocuser.MaxStep,         .. //
// ..         MaxFocuserLimit,                                  .. //
// ................................................................//
void ProcessCommand() 
{
  long TempLong;                                          // Variable tampon
  char TempTabChar[12];                                   // Variable tampon
  String TempString;                                      // Variable tampon
  unsigned int NumCmd;                                    // Numéro de commande
  char Cmd[MAXCOMMAND];                                   // Commande
  char Param[MAXCOMMAND];                                 // Paramètre

  // Séparation commande/paramètre
  memset(Cmd, 0, MAXCOMMAND);                             // Efface buffer commande
  memset(Param, 0, MAXCOMMAND);                           // Efface buffer paramètre
  int LenghtRx = strlen(LineRx);                          // Nbre de caractères reçus
  if (LenghtRx >= 2) strncpy(Cmd, LineRx, 2);             // Récupère 2 1er caractères (commande)
  if (LenghtRx > 2) strncpy(Param, LineRx + 2, LenghtRx - 2);// Reste caractère (Paramètre)
  memset(LineRx, 0, MAXCOMMAND);                          // Ré-initialise buffer réception
  EndOfCmd = false;
  Idx = 0;

  // Transforme les minuscules en majuscule
  if (Cmd[0] >= 'a' and Cmd[0] <= 'z') Cmd[0] -=  'a' - 'A';
  if (Cmd[1] >= 'a' and Cmd[1] <= 'z') Cmd[1] -=  'a' - 'A';
  
  // Transforme la commande en numéro de commande
  NumCmd = (Cmd[0] *256 + Cmd[1]);

  // Traitement des commandes
  switch (NumCmd)
  {
    // Requête GP = Get Position
    case ('G'*256+'P'): // Position courante focuser en Hexa non signé
    {
      sprintf(TempTabChar, "%04X", MyFocuser.FocuserPosition);
      Serial.print(TempTabChar);                          // Position courante
      Serial.print("#");
      break;
    } // case ('G'*256+'P'):
    //Requête GI = Get full In position
    case ('G'*256+'I'): // Le moteur tourne-t-il?
    {
      if (IsMoving) Serial.print("01#");                  // Oui
        else Serial.print("00#");                         // Non
      break;
    } // case ('G'*256+'I'):
    // Commande FG = Focuser Goto
    case ('F'*256+'G'): // Démarre le moteur vers la position consigne
    {
      if (!IsMoving)
      {
        GotoNewPosition = true;
        StepMotorNumber = STEPMOTORINIT;
      } // if (!IsMoving)
      break;
    } // case ('F'*256+'G'):
    // Commande FQ = Focus Quit
    case ('F'*256+'Q'): // Stop Moteur et Consigne <- position courante
    {
      MotorHalt();
      break;
    } // case ('F'*256+'Q'):
    // Commande PH = Position Home
    case ('P'*256+'H'): // Moteur va en position repos, Codé en dur
    {                   // ignore les Parametres (Non driver INDI)
      if (!IsMoving)
      {
        TargetPosition = 0;
        GotoNewPosition = true;
        StepMotorNumber = STEPMOTORINIT;
      } // if (!IsMoving)
      break;
    } // case ('P'*256+'H'):
    // Requête GN = Get New position
    case ('G'*256+'N'): // Récupère nouvelle consigne moteur 
    {                     // en Hexa non signé (Non driver INDI)
      sprintf(TempTabChar, "%04X", TargetPosition);
      Serial.print(TempTabChar);
      Serial.print("#");
      break;
    } // case ('G'*256+'N'):
    // Paramétrage SF = Set motor Full step
    case ('S'*256+'F'): // Réglage en pas entier
    {
      if (!IsMoving)
      {
        SetStepMode(1);
        WriteEepromNow = true;                            // Ecriture EEPROM nécessaire
        PreviousMillisEeprom = millis();                  // Pour tempo avant écriture
        LCD_Config();
      } // if (!IsMoving)
      break;
    } // case ('S'*256+'F'):
    // Paramétrage SH = Set motor  Half step
    case ('S'*256+'H'): // Reglage en 1/2 pas
    {
      if (!IsMoving)
      {
        SetStepMode(2);
        WriteEepromNow = true;                            // Ecriture EEPROM nécessaire
        PreviousMillisEeprom = millis();                  // Pour tempo avant écriture
        LCD_Config();
      } // if (!IsMoving)
      break;
    } // case ('S'*256+'H'):
    // Requête GH = Get Half step
    case ('G'*256+'H'): // Le moteur en 1/2 pas?
    {
      if ( MyFocuser.StepMode == 2 ) Serial.print("FF#"); // Oui
        else Serial.print("00#");                         // Non
      break;
    } // case ('G'*256+'H'):
    // Requête GS = Get Scale
    case ('G'*256+'S'): // Recupere Le mode micro pas utilisé
    {
      sprintf(TempTabChar, "%02X", MyFocuser.StepMode);
      Serial.print(TempTabChar);
      Serial.print("#");
      break;
    } // case ('G'*256+'S'):
    // Paramétrage SS = Set Scale
    case ('S'*256+'S'): // Règle le mode micro pas à utiliser
    {
      if (!IsMoving)
      {
        TempLong = HexStr2Long(Param);
        if (((unsigned char) TempLong) > STEPLIMITDRIVER) TempLong = STEPLIMITDRIVER;
        SetStepMode((unsigned char) TempLong);
        WriteEepromNow = true;                            // Ecriture EEPROM nécessaire
        PreviousMillisEeprom = millis();                  // Pour tempo avant écriture
        LCD_Config();
      } // if (!IsMoving)
      break;
    } // case ('S'*256+'S'):
    // Paramétrage SP = Set Position
    case ('S'*256+'P'): // Position courante moteur Initialisé avec paramètre
    {                   //  en Hexa non signé (Pas rotation moteur)
                        // Driver INDI, utilise seulement pour 0 SP0000 au reset()
      if (!IsMoving)
      {
        TempLong = HexStr2Long(Param);
        if ( TempLong > MyFocuser.MaxStep ) TempLong = MyFocuser.MaxStep;
        if ( TempLong < 0 ) TempLong = 0;
        MyFocuser.FocuserPosition = TempLong;
        TargetPosition = TempLong;
        GotoNewPosition = false;
        StepMotorNumber = STEPMOTORINIT;
        // Sauvegarde en  EEPROM
        WriteEepromNow = true;                            // Ecriture EEPROM nécessaire
        PreviousMillisEeprom = millis();                  // Pour tempo avant écriture
        LCD_MotorPosition();
      } // if (!IsMoving)
      break;
    } // case ('S'*256+'P'):
    // Paramétrage SN = Set New target position
    case ('S'*256+'N'): // Change la consigne 
    {                   // Moteur attend cmde 'FG' pour démarrer
      if (!IsMoving)
      {
        TempLong = HexStr2Long(Param);
        if ( TempLong > MyFocuser.MaxStep ) TempLong = MyFocuser.MaxStep;
        if ( TempLong < 0 ) TempLong = 0;
        TargetPosition = TempLong;
        GotoNewPosition = false;
        StepMotorNumber = STEPMOTORINIT;
        LCD_MotorTaget();
      } // if (!IsMoving)
      break;
    } // case ('S'*256+'N'):
    // Requête GT = Get the current Temperature (Compatible Moonlite)
    case ('G'*256+'T'): // Température courante en Hexa non signé
    {
      int TempInt;
      // Pas acquisition en cours on interroge le capteur, sinon on attend le résultat
      if (StartTemperatureAcquisition()) while (!ReadTemperature()) {delayMicroseconds(1);};
      TempInt = (int) ((Probe1TempVal * 2) + TempOffsetVal);
      sprintf(TempTabChar, "%04X", TempInt);
      Serial.print(TempTabChar);
      Serial.print("#");
      break;
    } // case ('G'*256+'T'):
    // Requête GZ = Get the current Temperature (Compatible Moonlite)
    case ('G'*256+'Z'): // Récupère température courante en Hexa non signé
    {
      float TempFloat;
      // Pas acquisition en cours on interroge le capteur, sinon on attend le résultat
      TempFloat = Probe1TempVal;
      Serial.print(TempFloat);
      Serial.print("#");
      break;
    } // case ('G'*256+'Z'):
    // Requête GV = Get Version
    case ('G'*256+'V'): // Récupère la version du firmware (Moonlite)
    {
      Serial.print("10#");
      break;
    } // case ('G'*256+'V'):
    // Requête GF = Get Firmware
    case ('G'*256+'F'): // Récupère la version du firmware
    {
      Serial.println(ProgramName);
      Serial.print(ProgramVersion + "#");
      break;
    } // case ('G'*256+'F'):
    // Requête GD = Get current step Delay (specifique Moonlite, Non Utilisé)
    case ('G'*256+'D'): // Renvoi "StepDelay" courant, valeurs valides 02, 04, 08, 10, 20
    {
      sprintf(TempTabChar, "%02X", StepDelay);
      Serial.print(TempTabChar);
      Serial.print("#");
      break;
    } // case ('G'*256+'D'):
    // Paramétrage SD = Set current step Delay (specifique Moonlite, Non Utilisé
    case ('S'*256+'D'): // Règle "StepDelay", valeurs valides: 02, 04, 08, 10, 20
    {                   // Correspond au stepping delay 250, 125, 63, 32 et 16 pas/s
      TempLong = HexStr2Long(Param);
      StepDelay = (int) TempLong;
      break;
    } // case ('S'*256+'D'):
    // Paramétrage SC = Set temperature Coefficient
    case ('S'*256+'C'): // Ne fait rien, ignoré
    {
      TempLong = HexStr2Long(Param);
      TempComp = (int) TempLong;
      break;
    } // case ('S'*256+'C'):
    // Requête GC = Get temperature Coefficient
    case ('G'*256+'C'): // Ne fait rien, ignoré
    {
      sprintf(TempTabChar, "%02X", TempComp);
      Serial.print(TempTabChar);
      Serial.print("#");
      break;
    } // case ('G'*256+'C'):
    // Paramétrage + , Active compensation température automatique du focuser
    case ('+'*256):
    { // Ignoré
      break;
    } // case ('+'*256):
    // Paramétrage - , Désactive compensation température automatique du focuser
    case ('-'*256):
    { // Ignoré
      break;
    } // case ('-'*256):
    // Paramétrage PO = temperature calibration Offset (Spécifique Moonlite)
    case ('P'*256+'O'): // POxx par incréments de 0.5 degree (hex)
    {
      // Ajoute ou soustrait un offset à la température lue par pas de 0.5°C
      // FA -3, FB -2.5, FC -2, FD -1.5, FE -1, FF 
      //-.5, 00 0, 01 0.5, 02 1.0, 03 1.5, 04 2.0, 05 2.5, 06 3.0
      TempOffsetVal = 0.0;
      if( Param == "FA" ) TempOffsetVal = -3.0;
      else if ( Param == "FB") TempOffsetVal = -2.5;
      else if ( Param == "FC") TempOffsetVal = -2.0;
      else if ( Param == "FD") TempOffsetVal = -1.5;
      else if ( Param == "FE") TempOffsetVal = -1.0;
      else if ( Param == "FF") TempOffsetVal = -0.5;
      else if ( Param == "00") TempOffsetVal = 0.0;
      else if ( Param == "01") TempOffsetVal = 0.5;
      else if ( Param == "02") TempOffsetVal = 1.0;
      else if ( Param == "03") TempOffsetVal = 1.5;
      else if ( Param == "04") TempOffsetVal = 2.0;
      else if ( Param == "05") TempOffsetVal = 2.5;
      else if ( Param == "06") TempOffsetVal = 3.0;
      break;
    } // case ('P'*256+'O'):
    // Paramétrage SM = Set new Maxsteps position SMxxxx
    case ('S'*256+'M'): // Règle nouvelle postion max du moteur
    {
      if (!IsMoving)
      {
        TempLong = HexStr2Long(Param);
        // Test limite haute et basse
        if ( TempLong > MAXFOCUSERLIMIT) TempLong = MAXFOCUSERLIMIT;
        if ( TempLong < MINFOCUSERLIMIT) TempLong = MINFOCUSERLIMIT;
        // Pour un NEMA17 à 400 pas, correspond à 5 rotations entière de la vis du focuser
        // fPour un 28BYG-28 c'est moins d'1/2 tour de vis du focuser
        MyFocuser.MaxStep = TempLong;
        // Vérifie que maxIncement ne dépasse pas la limite
        if( MaxIncrement > MyFocuser.MaxStep ) MaxIncrement = MyFocuser.MaxStep;
        // Déclenche une sauvegarde en EEPROM
        WriteEepromNow = true;        // Ecriture EEPROM nécessaire
        PreviousMillisEeprom = millis();// Pour tempo avant écriture
        LCD_Config();
      } // if (!IsMoving)
      break;
    } // case ('S'*256+'M'):
    // Requête GM = Get the Maxsteps
    case ('G'*256+'M'): // Renvoie la position limite du moteur
    {
      sprintf(TempTabChar, "%04X", MyFocuser.MaxStep);
      Serial.print(TempTabChar);
      Serial.print("#");
      break;
    } // case ('G'*256+'M'):
    // Paramétrage SY = Set new maxIncrement SYXXXX
    case ('S'*256+'Y'): // Paramètre l'incrément maximum autorisé
    {                   // Ignoré
      TempLong = HexStr2Long(Param);
      MaxIncrement = MyFocuser.MaxStep;
      break;
    } // case ('S'*256+'Y'):
    // Requêtr GY = Get the maxIncrement
    case ('G'*256+'Y'): // Renvoie l'incrément maximum autorisé
    {                   // toujours à MaxSteps
      sprintf(TempTabChar, "%04X", MaxIncrement);
      Serial.print(TempTabChar);
      Serial.print("#");
      break;
    } // case ('G'*256+'Y'):
    // Requêt GO = Get the cOilpwr setting
    case ('G'*256+'O'): // Bobines moteur alimentée entre 2 pas?
    {
      if (MyFocuser.CoilPwr) TempString = "01#";          // Oui
        else TempString = "00#";                          // Non
      Serial.print(TempString);
      break;
     }  // case ('G'*256+'O'):
     // Paramétrage SO = Set the cOilpwr setting
    case ('S'*256+'O'): // Bobines moteur doivent restent alimentée entre 2 pas
    {
      TempLong = HexStr2Long(Param);
      MyFocuser.CoilPwr = (TempLong != 0);
      WriteEepromNow = true;                              // Ecriture EEPROM nécessaire
      PreviousMillisEeprom = millis();                    // Pour tempo avant écriture
      LCD_Config();
      if (MyFocuser.CoilPwr) EnableCoilOutput();
        else DisableCoilOutput();
      break;
    } // case ('S'*256+'O'):
    // Reqête GR = Get the Reverse direction setting
    case ('G'*256+'R'): // Indique si le moteur réagi de manière inversé
    {
      String TempString;
      if (MyFocuser.ReverseDirection) TempString = "01#"; // Oui 
        else TempString = "00#";                          // Non
      Serial.print(TempString);
      break;
    } // case ('G'*256+'R'):
    // Paramétrage SR = Set the Reverse direction setting
    case ('S'*256+'R'): // Règle l'inversion de réaction moteur
    {
      if (!IsMoving)
      {
        TempLong = HexStr2Long(Param);
        MyFocuser.ReverseDirection = (TempLong != 0);
        WriteEepromNow = true;                            // Ecriture EEPROM nécessaire
        PreviousMillisEeprom = millis();                  // Pour tempo avant écriture
        LCD_Config();
      } // if (!IsMoving)
      break;
    } // case ('S'*256+'R'):
    // Paramètrage MS = set Motorspeed - Delais entre 2 impulsions successives
    case ('M'*256+'S'): // Valeurs : 00, 01 et 02 <=> Slow, Med, Fast
    {
      if (!IsMoving)
      {
        TempLong = HexStr2Long(Param);
        switch (TempLong)
        {
          case 0:
            MyFocuser.MotorSpeed = MOTORSPEEDSLOWmS;
            break;
          case 1:
            MyFocuser.MotorSpeed = MOTORSPEEDMEDmS;
            break;
          case 2:
            MyFocuser.MotorSpeed = MOTORSPEEDFASTmS;
            break;
          default:
            MyFocuser.MotorSpeed = MOTORSPEEDMEDmS;       // Paramètre non conforme, forcé à Medium
        } // switch (TempLong)
        InitSpeed();
        SetTimerMotor();
        LCD_Config();
      } // if (!IsMoving)
      break;
    } // case ('M'*256+'S'):
    // Paramétrage DS = set Display Status
    case ('D'*256+'S'): // Allume/Eteint l'afficheur
    {
      TempLong = HexStr2Long(Param);
      if (TempLong == 0)
      {
        LCD_Status(LCDOFF);
        LCD_OnBySerial = false;
      } // if (TempLong == 0)
      else if (TempLong == 1)
            {
              LCD_Status(LCDON);
              LCD_OnBySerial = true;
            } // else if (TempLong == 1)
      break;
    } // case ('D'*256+'S'):
    // Requête DG = Get Display status
    case ('D'*256+'G'): // Renvoie si l'afficheur est allumé/éteint
    {
      if (DisplayEnabled) Serial.print("01#");            // Allumé
        else Serial.print("00#");                         // Eteint
      break;
    } // case ('D'*256+'G'):
    // Paramétrage SU = Set diplay Update
    case ('S'*256+'U'): // Mise à jour afficheur quand moteur tourne
    {
      TempLong = HexStr2Long(Param);
      UpdateLCD_WhileMoving = (TempLong != 0);
      break;
    } // case ('S'*256+'U'):
    // Requêt GE = Get tempwaitdelay setting
    case ('G'*256+'E'): // Delai min entre 2 acquisitions successive température
    {
      sprintf(TempTabChar, "%04X", MyFocuser.TempWaitDelay);
      Serial.print(TempTabChar);
      Serial.print("#");
      break;
    } // case ('G'*256+'E'):
    // Paramétrage SE = Set tempwaitdelay setting
    case ('S'*256+'E'): // Delai min entre 2 acquisitions successive température
    {
      TempLong = HexStr2Long(Param);
      // Limites autorisées 2000-10000 <=> 2s-10s
      if( TempLong < MINTEMPWAITDELAY) TempLong = MINTEMPWAITDELAY;
      if( TempLong > MAXTEMPWAITDELAY) TempLong = MAXTEMPWAITDELAY;
      MyFocuser.TempWaitDelay = TempLong;
      WriteEepromNow = true;                              // Ecriture EEPROM nécessaire
      PreviousMillisEeprom = millis();                    // Pour tempo avant écriture
      break;
    } // case ('S'*256+'E'):
    // Requête GB = Get LED Backlight value (moonlite)
    case ('G'*256+'B'): // Renvoie toujours "00"
    {
      Serial.print("00#");
      break;
    } // case ('G'*256+'B'):
    // Paramétrage DM = Display Measurement
    case ('D'*256+'M'): // Affichage de la température en °C ou °F
    {
      // Ne fait rien, reste en celsius
      // TempLong = HexStr2Long(Param);
      // DisplayInCelsius = ( TempLong <= 0 ); 
      break;
    } // case ('D'*256+'M'):
    // Requête XY en cas de problème seulement
    case ('X'*256+'Y'): // Mode monitoring sur le port série
    {                   // Transfert tous les paramètres
      Serial.println();
      Serial.print("Current EEPROM Addr=");
      Serial.println(CurrentAddr);
      Serial.print("NumberOfLocations=");
      Serial.println(NumberOfLocations);
      Serial.print("Number of writes=");
      Serial.println(0);
      Serial.print("focuser=");
      Serial.println(MyFocuser.FocuserPosition);
      Serial.print("MaxSteps=");
      Serial.println(MyFocuser.MaxStep);
      Serial.print("MaxIncrement=");
      Serial.println(MaxIncrement);
      Serial.print("StepMode=");
      Serial.println(MyFocuser.StepMode);
      Serial.print("CoilPower=");
      Serial.println(MyFocuser.CoilPwr);
      Serial.print("ReverseDirection=");
      Serial.println(MyFocuser.ReverseDirection);
      Serial.print("MotorSpeed=");
      Serial.println(MyFocuser.MotorSpeed);
      if (ProbeTemp1Present) Serial.print("TempProbePresent= True"); 
        else Serial.print("TempProbePresent= False");
      Serial.print("Display Mode=");
      if (true) Serial.println("Celsius"); 
        else Serial.println("Fahrenheit");
      Serial.print("TempWaitDelay=");
      Serial.println(MyFocuser.TempWaitDelay);
      Serial.print("#");
      break;
    } // case ('X'*256+'Y'):
    // Commande XZ en cas de problème uniquement
    case ('X'*256+'Z'): // Remet les valeurs usine
    {
      CurrentAddr = 0;
      MyFocuser.ValidData = DATAOK;
      Reset_Myfocuser();
      // Ecriture des données en EEPROM
      WriteEepromNow = true;                              // Ecriture EEPROM nécessaire
      PreviousMillisEeprom = millis();                    // Pour tempo avant écriture
      // Réglage du focuser par défaut.
      TargetPosition = MyFocuser.FocuserPosition;
      break;
    } // case ('X'*256+'Z'):
  } // switch (NumCmd)
} // void ProcessCommand()

//*************************************************************************//
//*************************************************************************//
//***                             APPLICATION                           ***//
//*************************************************************************//
//*************************************************************************//

// ............................................................... //
// .. Nom: setup()                                              .. //
// .. Role: Ensemble des actions a effectuer au démarrage du    .. //
// ..       du programme                                        .. //
// .. Entrées: ProgramName,ProgramVersion                       .. //
// .. Sortie: Buzzer,LED_ToExtra,BlueLED_OUT,EndOfCmd,Idx,      .. //
// ..         StepMotorNumber,GotoNewPosition,TempOffsetVal,    .. //
// ..         DisplayEnabled,LastTempDisplay,                   .. //
// ..         LastModeDisplay,LastPosDisplay,MyDir,MyStep,MyM0  .. //
// ..         MyM1,MyM0,MyEnable,MyFocuser.FocuserPosition,     .. //
// ..         TargetPosition,MyFocuser.MaxStep                  .. //
// ................................................................//
void setup() 
{
  // Port Série
  Serial.begin(9600);                                     // Vitesse du port

  // Nécessaire pour driver ASCOM Moonlite, nécessite réponse rapide au démarrage
  // à la reqête :GB#
  Serial.print("00#");

  // LED & Buzzer au démarrage
  analogWrite( Buzzer, BUZZERSTATE );                     // Beep au démarrage
  analogWrite( LED_ToIntra, 1023 );                       // LED Intra ON
  analogWrite( LED_ToExtra, 1023 );                       // LED Extra ON
  
  // Initialisation des variables
  // Traitement commande série
  EndOfCmd = false;                                       // Caractère fin commande reçu
  Idx = 0;                                                // Indexe de caractère reçu
  memset(LineRx, 0, MAXCOMMAND);                          // Buffer de commande reçu
  
  // Afficheur LCD
  lcd.init();                                             // Initialise le LCD
  LCD_Status(LCDON);                                      // Allume le LCD

  // Message d'acceuil
  lcd.clear();                                            // Efface LCD
  lcd.setCursor(((LCDLENGTH - ProgramName.length()) / 2), 0);
  lcd.print(ProgramName);      // Affichage Nom programme
  lcd.setCursor(((LCDLENGTH - ProgramVersion.length()) / 2), 1);
  lcd.print(ProgramVersion);   // Affichage version programme

  // Configuration E/S de l'arduino
  // DRV8825
  pinMode(MyDir, OUTPUT);                                 // Sens de rotation
  pinMode(MyStep, OUTPUT);                                // Signal horloge
  pinMode(MyM0, OUTPUT);                                  // Config µ-pas
  pinMode(MyM1, OUTPUT);
  pinMode(MyM2, OUTPUT);
  pinMode(MyEnable, OUTPUT);                              // Alim bobine
  // Capteurs de fin de course
  pinMode(LimitIntra, INPUT);                             // Détection limite Intra-Focale
  pinMode(LimitExtra, INPUT);                             // Détection limite Extra-Focale
  
  // Réglage des données moteurs initiales
  digitalWrite(MyDir, 0);                                 // Etat bas
  digitalWrite(MyStep, 0);                                // Etat bas
  DisableCoilOutput();                                    // Bobines moteurs non alimentées

  // Recherche initialisation des données en EEPROM
  ReadEepromInitValues();
  
  // Recharge la config usine au démmarage
//  if (ReadPB_Switches() == ALLSWITCH) ResetToFactory();

  // Réglage des données moteurs
  TargetPosition = MyFocuser.FocuserPosition;             // Initialisation des variables de position
  TargetPositionStore = MyFocuser.FocuserPosition;
  SetTimerMotor();
  Timer1.initialize();                                    // Periode en µs
  SetTimerMotor();
  Timer1.attachInterrupt(MotorPulse);
  Timer1.stop();
  SetStepMode(MyFocuser.StepMode);                        // Réglage mode µ-pas du DRV8825
  if (MyFocuser.CoilPwr) EnableCoilOutput();              // Alimentation du moteur
    else DisableCoilOutput();
  InitSpeed();
  PushCount = PushCountInit;

  // Initialisation sonde & Mesure de température
  InitTempProbe();                

  // Eteint les LED et coupe le buzzer
  analogWrite( LED_ToIntra, 0 );                          // LED Intra-focale OFF
  analogWrite( LED_ToExtra, 0 );                          // LED Extra-focale OFF
  analogWrite( Buzzer, 0);                                // Arrêt buzzer

  // Affiche la config
  LCD_Config();
  LCD_OnBySerial = false;
  LastWakeeUpLCD = millis();
} // setup()

// PROGRAMME PRINCIPAL
void loop() 
{
  // Pour les tempos
  CurrentMillis = millis();
  
  if (!IsMoving)
  { // MOTEUR A L'ARRET
    // Status du LCD
    if (!LCD_OnBySerial)
    { // LCD n'est pas forcé allumé par le port série
      // Tempo avant extinction écoulée?
      if ((CurrentMillis - LastWakeeUpLCD) >= LCDBEFOREOFF) LCD_Status(LCDOFF);
    } // if (!LCD_OnBySerial)

    // Acquisition température & affichage à intervalles réguliers
    if (((CurrentMillis - LastTempDisplay) >= LCDUPDATERATETEMP) && !TempAcqProcessing)
    { // Lance une demande d'acquisition
      TempAcqProcessing = StartTemperatureAcquisition();
      LastTempDisplay = CurrentMillis;
    } // if (((CurrentMillis - LastTempDisplay) >= LCDUPDATERATETEMP) && !TempAcqProcessing)
    // Affichage si acquistion terminée
    if (TempAcqProcessing) {TempAcqProcessing = !ReadTemperature();}

    // Commande nécessitant une sauvegarde EEPROM reçue
    if (WriteEepromNow)
    {
      LCD_Store(true);
      if ((CurrentMillis - PreviousMillisEeprom) > EEPROMWRITETEMPO)
      {
        WriteMyFocuserEeprom();
        LCD_PosLimit();
      } // if ((CurrentMillis - PreviousMillisEeprom) > EEPROMWRITETEMPO)
    } // if (WriteEepromNow)

    // Gestion des boutons
    if (DisplayEnabled)
    { // On rentre dans le menu
      if (ReadPB_Switches() != NOSWITCH)
      {
        TestPause200();                                                     // pour permettre appui des 2 boutons
        if (ReadPB_Switches() == ALLSWITCH)                   // ENTRE DANS LE MENU?
        {
          MenuAdjust();                                                     // Appelle le menu
          LCD_Config();                                                     // Affichage paramètres focuser
          InitSpeed();                                                      // Paramètre les variables de gestion vitesse
          SetTimerMotor();                                                  // Ajuste le Timer1
          while (ReadPB_Switches() != NOSWITCH) {TestPause200();}        
          LastWakeeUpLCD = millis();
        } // if (ReadPB_Switches() == ALLSWITCH)
        else if (ReadPB_Switches() != NOSWITCH) PB_IntraExtra();// CONSIGNE POSITION?
      }
    } // if (DisplayEnabled)
    else if (ReadPB_Switches() != NOSWITCH)                     // ALLUMAGE LCD?
          { // On allume l'écran
            LCD_Status(LCDON);
            // Ré-morce la tempo d'extinction LCD
            LastWakeeUpLCD = millis();
          } // if (ReadPB_Switches() != NOSWITCH)
  } // if (!IsMoving)
  else
  { // MOTEUR TOURNE
    if ((UpdateLCD_WhileMoving) && ((CurrentMillis - LastPosDisplay) >  LCDUPDATEPOS))  // AFFICHAGE POSITION?
    {
      LCD_MotorPosition();
      LastPosDisplay  = CurrentMillis;
    } // if (UpdateLCD_WhileMoving) && ((CurrentMillis - LastPosDisplay) >  LCDUPDATEPOS))
    else PB_IntraExtra();                                                                    // CHANGEMENT CONSIGNE?

  } // (else) if (!IsMoving)

  // Traite la commande à l'arrivé d'un ":" sur le port série
  if (EndOfCmd) 
  {
    ProcessCommand();
    memset(LineRx, 0, MAXCOMMAND);
    EndOfCmd = false;
  } // if (EndOfCmd)
  
  // Gestion du moteur
  MotorControl();
} // void loop()

