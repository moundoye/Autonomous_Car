import smbus
import time
import sys
import tkinter
from tkinter import *

fenetre = tkinter.Tk()
# Remplacer 0 par 1 si nouveau Raspberry
bus = smbus.SMBus(1)
address = 0x03

def clavier(event):
    touche = event.keysym
    print(touche)
    bus.write_byte(address, 0)
    if (touche == "Up"):
        bus.write_byte(address, 1)
        print(1)
    if (touche == "Down"):
        bus.write_byte(address, 2)
        print(2)
    if (touche == "Left"):
        bus.write_byte(address, 3)
        print(3)
    if (touche == "Right"):
        bus.write_byte(address, 4)
        print(4)
    # Pause de 1 seconde pour laisser le temps au traitement de se faire
    time.sleep(1)

canvas = Canvas(fenetre, width=500, height=500)
canvas.focus_set()
canvas.bind("<Key>", clavier)
canvas.pack()

