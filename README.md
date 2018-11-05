# ArduiStepFocuser
Ce projet est destiné à piloter un moteur pas à pas pour effectuer la mise au point d'un instrument d'astronomie (lunette, télescope, objectif photo). 
Une sonde fournit la température de l'instrument. 

L'ArduiStepFocus est basé sur la version « Focuserv151_DRV8825_HW203_F » du projet Arduino ASCOM Focuser Pro DIY (https://sourceforge.net/projects/arduinofocuscontrollerpro/).
Le programme de l'arduino a été ré-écrit dans sa grande majorité, mais les commandes séries restent compatibles. Le driver ASCOM associé à ce projet est donc compatible avec l'ArduiStepFocus.

Le Github commence avec la version 0.7.5 du code ArduiStepFocuser.

# Développeurs :
Gandalf (WebAstro) - Développeur Principal, Etmay53 (Webastro) - Testeur, Dragonlost (WebAstro/GitHub) - Mise en ligne

# Documentation : 
https://github.com/dragonlost/ArduiStepFocuser/raw/master/Documentations/ArduitStepFocuser.pdf

# Prérequi : 

- Arduino IDE 1.8.5
- Bibliothéque Arduino : DallasTemperature, TimerOne, OneWire, LiquidCrystal I2C (Frank de Brabander)

Tuto d'installation :
https://github.com/dragonlost/ArduiStepFocuser/raw/master/Documentations/Tuto_biblio_arduino.pdf

# PCB et liste des composants :

Le nouveaux PCB et le schema est dévellopé sur la platforme EasyEDA.
Lien du projet :

https://easyeda.com/dragonlost/ArduiStepFocuser-05782649d5d647508e7d01ccf179df9b

La liste des composants est sous forme de classeur XLSX (lisible par libreoffice) avec les references constructeurs ainsi que les références Digikey et Mouser.
https://github.com/dragonlost/ArduiStepFocuser/raw/master/Documentations/BOM_ArduiStepFocuser.xlsx

__Attention :__ La liste n'est pas identique au site EASY-EDA, elle contient les jumper un deuxième modèle d'Arduino Nano ( choisir entre les 2 avant de commander, ou l'acheter une copie à coté).
La liste ne contient pas les 2 LED, le switch ON-ON et le driver moteur, à choisir par vous même.

Le BOM marche sans aucune modification avec le revendeur [Mouser](https://www.mouser.fr/ProjectManager/ProjectDetail.aspx?AccessID=8962ca1cb3) et [Digi-Key](https://www.digikey.fr/short/j0r299)

# Vidéos :
vidéo de Etmay53 (WebAstro)

https://www.youtube.com/watch?v=-dUdCSXep7k


# Développements futures :
- support de la carte Wemos D1 Mini
- support d'un écran OLED
- support des drivers TMC2100 et TMC2130 en SPI

# Description des branches :
- master = branche principale (écran LCD20x04 + DS1820 + DRV8825 + buzzer + bouton analogique + fin de course).
- master-TOR = branche principale (écran LCD20x04 + DS1820 + DRV8825 + buzzer + bouton numerique + fin de course).
- dev = branche master en développement.
