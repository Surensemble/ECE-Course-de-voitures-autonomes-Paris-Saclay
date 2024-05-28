#----------------------------------------  bibliotheque ----------------------------------------------#
#######################################################################################################
from rplidar import RPLidar
import time
from rpi_hardware_pwm import HardwarePWM
import numpy as np
import sys
import select
import smbus





#------------------------ fonctions, initialisations et variables utiles -----------------------------#
#######################################################################################################

# initialisation et fonction pour utiliser le capteur ultrason 
bus = smbus.SMBus(1)
address = 0x70

#REQUIRES 5V
def write(value):
        bus.write_byte_data(address, 0, value)
        return -1

def ultra():
        MSB = bus.read_byte_data(address, 2)
        LSB = bus.read_byte_data(address, 3)
        ultra = (MSB << 8) + LSB
        return ultra
     


#parametres de la fonction vitesse_m_s
direction_prop = 1 # -1 pour les variateurs inverses ou un petit rapport correspond à une marche avant
pwm_stop_prop = 7.39
point_mort_prop = 0.39
delta_pwm_max_prop = 1. #pwm à  laquelle on atteint la vitesse maximale 
vitesse_max_m_s_hard = 8 #vitesse que peut atteindre la voiture
vitesse_max_m_s_soft = 2 #vitesse maximale que l'on souhaite atteindre

#parametres de la fonction set_direction_degre
direction = -1 #1 pour angle_pwm_min à gauche, -1 pour angle_pwm_min à droite
angle_pwm_min = 4.8   #min
angle_pwm_max = 7.7   #max
angle_pwm_centre= 6.25
angle_degre_max = +18 #vers la gauche
angle_degre=0



pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
pwm_prop.start(pwm_stop_prop)

#fonction de reglage de la vitesse en m/s
def set_vitesse_m_s(vitesse_m_s):
    if vitesse_m_s > vitesse_max_m_s_soft :
        vitesse_m_s = vitesse_max_m_s_soft
    elif vitesse_m_s < -vitesse_max_m_s_hard :
        vitesse_m_s = -vitesse_max_m_s_hard
    if vitesse_m_s == 0 :
        pwm_prop.change_duty_cycle(pwm_stop_prop)
    elif vitesse_m_s > 0 :
        vitesse = vitesse_m_s * (delta_pwm_max_prop)/vitesse_max_m_s_hard
        pwm_prop.change_duty_cycle(pwm_stop_prop + direction_prop*(point_mort_prop + vitesse ))
    elif vitesse_m_s < 0 :
        vitesse = vitesse_m_s * (delta_pwm_max_prop)/vitesse_max_m_s_hard
        pwm_prop.change_duty_cycle(pwm_stop_prop - direction_prop*(point_mort_prop - vitesse ))


"""FONCTION POUR RECULER
cette fonction n'est pas utilisé dans le programme car l'utilisation de time.sleep
crée un bug avec l'utilsation du LIDAR """        
def recule():
    set_vitesse_m_s(-vitesse_max_m_s_hard)
    time.sleep(0.2)
    set_vitesse_m_s(0)
    time.sleep(0.2)
    set_vitesse_m_s(-1)

  
pwm_dir = HardwarePWM(pwm_channel=1,hz=50)
pwm_dir.start(angle_pwm_centre)

#fonction pour regler l'angle de direction en degre
def set_direction_degre(angle_degre) :
    global angle_pwm_min
    global angle_pwm_max
    global angle_pwm_centre
    angle_pwm = angle_pwm_centre + direction * (angle_pwm_max - angle_pwm_min) * angle_degre /(2 * angle_degre_max )
    if angle_pwm > angle_pwm_max : 
        angle_pwm = angle_pwm_max
    if angle_pwm < angle_pwm_min :
        angle_pwm = angle_pwm_min
    pwm_dir.change_duty_cycle(angle_pwm)


    
#connexion et démarrage du lidar
lidar = RPLidar("/dev/ttyUSB0",baudrate=256000)
lidar.connect()
print (lidar.get_info())
lidar.start_motor()
time.sleep(1)


#variables utiles
tableau_lidar_mm = [0]*360 #création d'un tableau de 360 zéros
timer_ms=0                 #permet de simuler un compteur de seconde 
state=1                    #initialisation de l'etat de la voiture
tabt0=[0,0,0]              #permet d'enregistrer les conditions de la voitue à un instant t
tabt1=[0,0,0]              #permet d'enregistrer les conditions de la voiture à un instant t+1 
count=0                    #permet de faire 2 tours de boucle avant de verifier la condition pour reculer 




#---------------------------------- conduite autonome --------------------------------------------------#
#########################################################################################################

try : 
    for scan in lidar.iter_scans(scan_type='express') : 
    #Le tableau se remplissant continuement, la boucle est infinie
        #rangement des données dans le tableau
        for i in range(len(scan)) :
            angle = min(359,max(0,359-int(scan[i][1]))) #scan[i][1] : angle 
            tableau_lidar_mm[angle]=scan[i][2]  #scan[i][2] : distance    
        #############################################
        ## Code de conduite (issu du simulateur ou non)
        #############################################
               
    #mesure avec ultrason     (cause un bug)
        """write(0x51)
           rng= ultra()
           print(rng)"""
    #gestion des commandes du clavier afin de pouvoir demarrer et 
    #arreter la voiture sans avoir à relancer le programme        (cause un bug)
        """ input = select.select([sys.stdin],[],[],1)[0]
            if input:
                 value = sys.stdin.readline().rstrip()
                 if (value == "q"):
                    print("Existing")
                    sys.exit(0)
                 elif (value == "a"):
                    print("you entered: %s " % value)
                    state = 1
                 elif (value == "s"):
                    state =0  """
        
        timer_ms+=1
        count +=1

        #enregistrement des données lidar pour savoir si la voiture est immobile
        tabt0=[tabt1[0],tabt1[1],tabt1[2]]
        tabt1=[tableau_lidar_mm[-30],tableau_lidar_mm[0],tableau_lidar_mm[30]] 
        
        match state:
            case 0: # voiture immobile prete à demarer la course

                set_direction_degre(0)
                set_vitesse_m_s(0)

			
            case 1: # voiture en course	
                 
                #generation des tentacules rectilignes  
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
                      
                ind=np.argmax(lid)   #choix du tentacule le plus long
                angle_degre=dir[ind] #choix de l'angle de direction associé au tentacule le plus long
                
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
                
                set_direction_degre(angle_degre)# application de l'angle choisi sur le vehicule 
                vitesse_m_s = 0.5               # choix d'une vitesse de constante. elle peut être modifiée
                set_vitesse_m_s(vitesse_m_s)    # application de la vitesse choisi sur le vehicule
                

                #condition pour reculer 
                if count>=2:
                   count=0
                   if abs(tabt0[1]-tabt1[1])<3 and abs(tabt0[0]-tabt1[0])<3 and abs(tabt0[2]-tabt1[2])<3: #verifie si la voiture est immobile
                     state=2
                     set_vitesse_m_s(-8000)
                     timer_ms=0
                     

                     
            case 2: # debut de la marche arriere
                if timer_ms>=2:
                    state=3
                    set_vitesse_m_s(0)
                    timer_ms=0

            case 3: # marche arriere et repositionnement
                if timer_ms>=2:
                    timer_ms=0
                    if tableau_lidar_mm[90]<tableau_lidar_mm[-90]:
                        set_direction_degre(18)
                    else:
                        set_direction_degre(-18)	
                          
                    set_vitesse_m_s(-2)                       
                    state=4
                
            case 4: # retour à la course
                if timer_ms>=10:
                    state=1				
           
        
        
        
#----------------------------------arret total du vehicule-----------------------------------#        
##############################################################################################
except KeyboardInterrupt: #récupération du CTRL+C
    print("fin des acquisitions")

#arrêt et déconnexion du lidar et des moteurs
lidar.stop_motor()
lidar.stop()
time.sleep(1)
lidar.disconnect()
pwm_dir.stop()
pwm_prop.start(pwm_stop_prop)