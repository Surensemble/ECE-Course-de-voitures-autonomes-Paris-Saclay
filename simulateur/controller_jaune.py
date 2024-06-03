from vehicle import Driver
from controller import Lidar
import numpy as np

driver = Driver()

basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep

#Lidar
lidar = Lidar("RpLidarA2")
lidar.enable(sensorTimeStep)
lidar.enablePointCloud() 

#clavier
keyboard = driver.getKeyboard()
keyboard.enable(sensorTimeStep)

# vitesse en km/h
speed = 0
maxSpeed = 100 #km/h   28 km/h de base

# angle de la direction
angle = 0
maxangle_degre = 16


# mise a zéro de la vitesse et de la direction
driver.setSteeringAngle(angle)
driver.setCruisingSpeed(speed)

tableau_lidar_mm=[0]*360

def set_vitesse_m_s(vitesse_m_s):
    speed = vitesse_m_s*3.6
    if speed > maxSpeed :
        speed = maxSpeed
    if speed < 0 :
        speed = 0
    driver.setCruisingSpeed(speed)
    return speed
     
def set_direction_degre(angle_degre):
    if angle_degre > maxangle_degre:
        angle_degre = maxangle_degre
    elif angle_degre < -maxangle_degre:
        angle_degre = -maxangle_degre   
    angle = -angle_degre * 3.14/180
    driver.setSteeringAngle(angle)

def recule(): #sur la voiture réelle, il y a un stop puis un recul pendant 1s.
    driver.setCruisingSpeed(-1)  
      
    
        

# mode auto desactive
modeAuto = False
print("cliquer sur la vue 3D pour commencer")
print("a pour mode auto (pas de mode manuel sur TT02_jaune), n pour stop")

while driver.step() != -1:
    while True:
    #acquisition des donnees du lidar
         # recuperation de la touche clavier
        currentKey = keyboard.getKey()
 
        if currentKey == -1:
            break
       
        elif currentKey == ord('n') or currentKey == ord('N'):
            if modeAuto :
                modeAuto = False
                print("--------Modes Auto TT-02 jaune Désactivé-------")
        elif currentKey == ord('a') or currentKey == ord('A'):
            if not modeAuto : 
                modeAuto = True
                print("------------Mode Auto TT-02 jaune Activé-----------------")
    
    #acquisition des donnees du lidar
    donnees_lidar_brutes = lidar.getRangeImage()
    for i in range(360) :
        if (donnees_lidar_brutes[-i]>0) and (donnees_lidar_brutes[-i]<200) :
            tableau_lidar_mm[i] = 1000*donnees_lidar_brutes[-i]
        else :
            tableau_lidar_mm[i] = 0
   
    if not modeAuto:
        set_direction_degre(0)
        set_vitesse_m_s(0)
        
    if modeAuto:
    ########################################################
    # Programme etudiant avec
    #    - le tableau tableau_lidar_mm
    #    - la fonction set_direction_degre(...)
    #    - la fonction set_vitesse_m_s(...)
    #    - la fonction recule()
    #######################################################
   
        #un secteur par tranche de 20° donc 10 secteurs numérotés de 0 à 9    
        
        
       #angle_degre = 0.02*(tableau_lidar_mm[60]-tableau_lidar_mm[-60])
       dir=[18,16,16,5,3,0,-3,-5,-16,-16,-18]
       lid=[tableau_lidar_mm[90],
            tableau_lidar_mm[60], 
            tableau_lidar_mm[45],
            tableau_lidar_mm[20],
            tableau_lidar_mm[10],
            tableau_lidar_mm[0],
            tableau_lidar_mm[-10],
            tableau_lidar_mm[-20],
            tableau_lidar_mm[-45],
            tableau_lidar_mm[-60], 
            tableau_lidar_mm[-90]]
       ind=np.argmax(lid)
       angle_degre=dir[ind]
        
        #correction de l'angle de direction en cas de detection d'obstacle proche du vehicule      
       if tableau_lidar_mm[30]-tableau_lidar_mm[-30]<0:            
           if tableau_lidar_mm[20]<300 or tableau_lidar_mm[10]<800 or tableau_lidar_mm[2]<300 or tableau_lidar_mm[60]<300:
              angle_degre-=18
              set_vitesse_m_s(0.08)
           elif tableau_lidar_mm[-20]<300 or tableau_lidar_mm[-10]<800 or tableau_lidar_mm[-2]<300 or tableau_lidar_mm[-60]<300:
              angle_degre+=18
              set_vitesse_m_s(0.08)	
       else:  
           if tableau_lidar_mm[-20]<300 or tableau_lidar_mm[-10]<800 or tableau_lidar_mm[-2]<300 or tableau_lidar_mm[-60]<300:
              angle_degre+=18
              set_vitesse_m_s(0.08)
           elif  tableau_lidar_mm[20]<300 or tableau_lidar_mm[10]<800 or tableau_lidar_mm[2]<300 or tableau_lidar_mm[60]<300:
              angle_degre-=18
              set_vitesse_m_s(0.08) 
        
       set_direction_degre(angle_degre) 
       
       set_vitesse_m_s(0.7)
        
        
       
        
        
    #########################################################

