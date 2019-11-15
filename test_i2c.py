import cv2
import smbus
import time
import sys
import tkinter
from tkinter import *


# Remplacer 0 par 1 si nouveau Raspberry
bus = smbus.SMBus(1)
address = 0x03

# Pause de 1 seconde pour laisser le temps au traitement de se faire

while True:
    reponse = bus.read_byte(address)               
    print("distance obstacle : ",reponse)
    if (reponse<50) :  
        print("les moteurs sont à l'arrêt")
        
    key = cv2.waitKey(1) & 0xFF

	# la touche q permet d'interrompre la boucle principale
    if key == ord("q"):
        break    
    
#sys.exit(0)

    time.sleep(1)