# importer tout les packages requis
from imutils.video import VideoStream
from imutils.video import FPS
import imutils
import time
import cv2
import sys
import smbus
import numpy as np
#from matplotlib import pyplot as plt

####on initialise la liaison spi####
# Remplacer 0 par 1 si nouveau Raspberry
bus = smbus.SMBus(1)
address = 0x03

"""
print("...démarrage de la Picamera...")
vs = VideoStream(usePiCamera=True, resolution=(800, 300)).start()
time.sleep(2.0)
fps = FPS().start()
"""

###on globalise certains variables###
num_order = 0
wait = 0
cpt = 0
order = "none"
Turn_flag_right = False
Turn_flag_left = False

#### je définie plusieurs blocs de fonction qui correspondent aux ordres à envoyer ###
def none ():
    a=1
    #bus.write_byte(address, 0)
    
def Ahead ():
    b=2
    #bus.write_byte(address, 1)

def turn_left ():
    c=3
    #bus.write_byte(address, 3)

def turn_right ():
    d=4
    #bus.write_byte(address, 0)
        
       
#### j'associe l'input aux blocs de fonction
order = {0 : none,
         1 : Ahead,
         3 : turn_left,
         4 : turn_right,
         }

while True:
    right = False
    left = False
    left_Ahead = False
    right_Ahead = False
    # récupération du flux vidéo, redimension 
    # afin d'afficher au maximum 800 pixels 

    frame = cv2.imread('Image1.png')
    #frame = vs.read()
    #frame = imutils.resize(frame, width=800)

    #je recupère les dimensions de l'image
    C,H,W=frame.shape[::-1]
    #ON enlève le bruit
    # Remove noise by blurring with a Gaussian filter
    frame = cv2.GaussianBlur(frame, (3, 3), 0)
    #img = cv2.imread('test.jpg')
    
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0,0,0], dtype=np.uint8)
    upper_white = np.array([17,85,255], dtype=np.uint8)
    
    # Threshold the HSV image to get only white colors
    mask = cv2.inRange(hsv, lower_white, upper_white)
    #je binarise l'image
    
    #ret,thresh = cv2.threshold(mask,145,255,cv2.THRESH_BINARY)
    ret,thresh = cv2.threshold(mask,145,255,cv2.THRESH_BINARY_INV)

    #th3 = cv2.adaptiveThreshold(mask,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
    #on détermine l'image de contour et on sort les lignes droites
    #print(thresh.shape)
    ###############
    edges = cv2.Canny(frame,80,120,apertureSize = 3)
    #lines = cv2.HoughLines(edges,1,np.pi/180,200)
    #print(edges)
    #nb_lignes = len(lines)
    
    #je fais une détection de contours sur la partie gauche de l'image
    im_1, contours_1, hierarchy_1 = cv2.findContours(thresh[:,0:int(H/4),],cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #je fais une détection de contours sur la partie droite de l'image
    im_2, contours_2, hierarchy_2 = cv2.findContours(thresh[:,int(3*H/4):H,],cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #je fais une détection de contours sur toute l'image
    im_3, contours_3, hierarchy_3 = cv2.findContours(thresh[:,int(H/4):int(3*H/4),],cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #nb_contours = len(contours)
    #print(contours[0])
    #cv2.drawContours(frame,contours,4,(255,0,0),4)
    
    #on trace le ou les contours détectés


    for i in range(len(contours_3)):
        perimetre = cv2.arcLength(contours_3[i],True)
        #print("",perimetre)
        #print(perimetre)
        if perimetre > 400 and perimetre < 900:
            convex_3 = cv2.minAreaRect(contours_3[i])
            print("convex_3 ", convex_3[2])
            if abs(convex_3[2]) > 20 and abs(convex_3[2]) < 60:
                left = True
                right = False
                
            elif abs(convex_3[2]) < 20 and abs(convex_3[2]) > 0:
                left = False
                right = True 
            cv2.drawContours(frame[:,int(H/4):int(3*H/4),],contours_3,i,(255,0,0),4)
        
    
    for i in range(len(contours_2)):
        perimetre = cv2.arcLength(contours_2[i],True)
        #print("",perimetre)
        #print(perimetre)
        if perimetre > 200 and perimetre < 350:
            cv2.drawContours(frame[:,int(3*H/4):H,],contours_2,i,(0,255,0),4)
            right_Ahead = True
            if num_order == 3 and wait > 65 :
                num_order = 0

    for i in range(len(contours_1)):
        perimetre = cv2.arcLength(contours_1[i],True)
        #print("",perimetre)
        #print(perimetre)
        if perimetre > 200 and perimetre < 350:
            cv2.drawContours(frame,contours_1,i,(0,255,0),4)
            left_Ahead = True        
            
     #######on vérifie quelle direction suivre########
    if num_order == 1 or num_order == 0:
        #print("i'm in")
        if left == True and right == False and Turn_flag_left == False:
    #        print("check left")
            num_order = 3
        elif right == True and left == False and Turn_flag_right == False:
    #        print("check right")
            num_order = 4
        elif (right == False) and (left == False) and ((left_Ahead == True) or (right_Ahead == True)):
    #        print("check ahead")
            num_order = 1
        else :
            num_order = num_order

    if (num_order == 1) :        
        print("ahead : True")
    else :
        print("ahead : False")
    print("turn left : ", left)
    print("turn right : ", right)
    print("wait : ", wait)
    #print(num_order)
    print("----------------------------------------------------------------------")
    
    #gestion de la boucle d'attente et envoie des ordres
    order [num_order]()
    if num_order == 3 or num_order == 4 :
        wait+=1;
        if wait > 300 :
            wait = 0
            num_order=0
            
    if num_order == 0 or num_order == 1 : 
        wait = 0
            
    # affichage du flux vidéo dans une fenètre
    cv2.imshow("mask",hsv)
    cv2.imshow("frame",thresh)
    
    cv2.imwrite('img_thresh.png', thresh)
    
    key = cv2.waitKey(1) & 0xFF

    # la touche q permet d'interrompre la boucle principale
    if key == ord("q"):
        break
"""
    # mise à jour du FPS 
    fps.update()


# arret du compteur et affichage des informations dans la console
fps.stop()
print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
"""

cv2.destroyAllWindows()
vs.stop()
    


