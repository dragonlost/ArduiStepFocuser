# ArduiStepFocuser
Ce projet est destiné à piloter un moteur pas à pas pour effectuer la mise au point d'un instrument d'astronomie (lunette, télescope, objectif photo). 
Une sonde fournit la température de l'instrument. 

L'ArduiStepFocus est basé sur la version « Focuserv151_DRV8825_HW203_F » du projet Arduino ASCOM Focuser Pro DIY (https://sourceforge.net/projects/arduinofocuscontrollerpro/)
Le programme de l'arduino a été ré-écrit dans sa grande majorité, mais les commandes séries restent compatibles. Le driver ASCOM associé à ce projet est donc compatible avec l'ArduiStepFocus.

Le Github commence avec la version 0.7.5 du code ArduiStepFocuser.

# Développeur :

# Documentation : 

# PCB et liste des composants :

# Développements futures
- support de la carte Weemos D1 Mini
- support d'un écran OLED

# Description des branches :
master = branche principale (écran LCD20x04 + DS1820 + DRV8825 + buzzer + bouton + fin de course)