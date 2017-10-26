# ArduiStepFocuser
Ce projet est destiné à piloter un moteur pas à pas pour effectuer la mise au point d'un instrument d'astronomie (lunette, télescope, objectif photo). 
Une sonde fournit la température de l'instrument. 

L'ArduiStepFocus est basé sur la version « Focuserv151_DRV8825_HW203_F » du projet Arduino ASCOM Focuser Pro DIY (https://sourceforge.net/projects/arduinofocuscontrollerpro/).
Le programme de l'arduino a été ré-écrit dans sa grande majorité, mais les commandes séries restent compatibles. Le driver ASCOM associé à ce projet est donc compatible avec l'ArduiStepFocus.

Le Github commence avec la version 0.7.5 du code ArduiStepFocuser.

# Développeurs :
Gandalf (WebAstro) - Développeur Principal, Etmay53 (Webastro) - Testeur, Dragonlost (WebAstro/GitHub) - Mise en ligne

# Documentation : 
https://github.com/dragonlost/ArduiStepFocuser/raw/master/ArduitStepFocuser.pdf

# PCB et liste des composants :

Le nouveaux PCB et le schema est dévellopé sur la platforme EasyEDA.
Lien du projet :

https://easyeda.com/dragonlost/ArduiStepFocuser-05782649d5d647508e7d01ccf179df9b

La liste des composants est sous forme de classeur ODS (libreoffice) avec les references constructeurs ainsi que les références Digikey et Mouser.

# Vidéos :
vidéo de Etmay53 (WebAstro)

https://www.youtube.com/watch?v=-dUdCSXep7k


# Développements futures :
- support de la carte Wemos D1 Mini
- support d'un écran OLED
- support des drivers TMC2100 et TMC2130 en SPI

# Description des branches :
- master = branche principale (écran LCD20x04 + DS1820 + DRV8825 + buzzer + bouton + fin de course).
- dev = branche master en développement.
- dev-oled = remplacement du LCD 20x04 par un écran OLED monochrome.
- dev-oled+wemos = remplacement du LCD 20x04 par un écran OLED monochrome et remplecement de l'arduino Nano par un Wemos.
- dev-html = dévellopement de la page html du Wemos.
- dev-TMC2100 = branche Master avec DRV8825 remplacé par un driveur TMC2100.
- dev-TMC2130 = branche Master avec DRV8825 remplacé par un driveur TMC2130-SPI.