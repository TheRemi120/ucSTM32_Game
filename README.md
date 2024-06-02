# FORMULA ONE

## Objectif
Créer un jeu de course de voiture avec une manette externe à la carte.

## Principe
Le jeu consiste à conduire une Formule1 sur une piste de course et à essayer d'aller le plus vite possible sans prendre de pénalités (qui sont données à chaque instant où la F1 se trouve en dehors de la piste) !

## Contrôles
- Utilisation de l'application android nommée "Arduino Bluetooth Control" pour utiliser son smartphone comme manette afin de déplacer la F1 sur la carte.

## Ressources STM32 Utilisées
### Périphériques
- GPIO pour la liaison série avec le module bluetooth RNBT-52B6.
- SAI (Serial Audio Interface) pour la gestion du son, utilisé avec la bibliothèque BSP_AUDIO.
- Écran LCD via la bibliothèque BSP_LCD.

### FreeRTOS
Utilisation des fonctionnalités de FreeRTOS :
- Tâches pour gérer les différentes fonctions du jeu.
- Files d'attente pour récupérer les messages reçus via liaison série (bluetooth) et pour le réveil des tâches.
- Interruptions.

### FATFS
Lecture des fichiers audio au format .WAV et visuels au format .BMP grâce à la bibliothèque FATFS.

### Gestion de la transparence
Modification de la librairie d'affichage des images .BMP pour implémenter la gestion de fonds transparents sur les images.

## Utilisation
1. Mettez les fichiers audio sur la carte SD présente dans le dossier Ressources du projet.
2. Insérez la carte SD dans le lecteur de la carte STM32.
3. Branchez un périphérique audio pour profiter de l'expérience sonore.
4. Utilisez votre smartphone android pour déplacer la voiture sur la piste de course.
# uc_STM32_Game
# uc_STM32_Game
# uc_STM32_Game
# uc_STM32_Game
# ucSTM32_Game
