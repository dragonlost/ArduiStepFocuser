/*
 * EEPROM Clear
 *
 * Réinitialisation de tout les bytes de l'EEPROM à 0.
 * S'il vous plait regarder la variable eeprom_iteration pour une profondeur plus grande
 * Regardez comment parcourir l'EEPROM.
 *
 * Ce code d'exemple est dans le domaine publique. 
 */

#include <EEPROM.h>

void setup() {
  // initialise la pin de la LED pin en mode output.
  pinMode(13, OUTPUT);
  // Passe la LED OFF en début d'opération.
  digitalWrite(13, LOW);
  
  /***
    Itére à travers chaque octet de la mémoire de l'EEPROM.

    Les microcontroleurs AVR plus gros ont des tailles d'EEPROM plus grandes, par exemple:
    - Arduno Duemilanove: stockage EEPROM 512b.
    - Arduino Uno/Nano: stockage EEPROM 1kb.
    - Arduino Mega: stockage EEPROM 4kb.

    Le code est écrit pour s'adapater automatiquement à la taille de l'EEPROM des microcontroleurs AVR
  ***/

  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }

  // Passe la LED ON quand l'opération est fini.
  digitalWrite(13, HIGH);
}

void loop() {
  /** boucle vide. **/
}
