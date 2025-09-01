# Code from PROJET-S8

from dynamixel_sdk import PortHandler, PacketHandler
from config import *
from motor import Motor
import time

class MotorController:
    def __init__(self):
        
        self.open_port()
        
        # Définition des moteurs
        self.motors = [
            Motor(33, "MX-28", "abs_z", 80, 100, 90, 1),
            Motor(34, "MX-28", "bust_y", 110, 200, 180, 1),
            # Motor(35, "MX-28", "bust_x", 110, 190, 185, -1),
            Motor(36, "AX-12A", "head_z", 60, 240, 150, 1),
            Motor(37, "AX-12A", "head_y", 110, 140, 110, 1),
            Motor(41, "MX-28", "l_shoulder_y", 90, 270, 270, -1),
            Motor(42, "MX-28", "l_shoulder_x", 90, 270, 270, -1),
            Motor(43, "MX-28", "l_arm_z", 90, 270, 90, 1),
            Motor(44, "MX-28", "l_elbow_y", 35, 180, 5, 1),
            Motor(51, "MX-28", "r_shoulder_y", 90, 270, 90, 1),
            Motor(52, "MX-28", "r_shoulder_x", 90, 270, 90, 1),
            Motor(53, "MX-28", "r_arm_z", 90, 270, 270, -1),
            Motor(54, "MX-28", "r_elbow_y", 180, 325, 365, -1),
        ]

        # Création d'un dictionnaire {ID: Motor}
        self.motors_dic = {motor.ID: motor for motor in self.motors}

        # Vérification de la connection avec tous les moteurs
        self.ping_all()

        # Initialisation de la position des moteurs
        self.set_all_compliance(True)
        self.init_position()

    def open_port(self):
        """Ouvre le port de communication"""
        try:    
            portHandler.openPort()
            print("Port ouvert")
        except Exception as e:
            print(f"Erreur lors de l'ouverture du port: {e}")
            exit()
            
        try:
            portHandler.setBaudRate(BAUDRATE)
            print("Baud rate set")
        except Exception as e:
            print(f"Erreur lors du changement de baud rate: {e}")
            exit()

    def close_port(self):
        """Ferme le port proprement"""
        portHandler.closePort()
        print("Port fermé")

    def init_position(self):
        """Déplace tous les moteurs vers leur position de départ"""
        print("\nDéplacement de tous les moteurs en position 0...")
        for motor in self.motors:
            motor.move_to(0, 2)

    def set_all_compliance(self, enable):
        """Active/désactive le couple pour tous les moteurs"""
        print("\nModification du couple de tous les moteurs...")
        for motor in self.motors:
            motor.set_compliance(enable)

    def shut_down(self):
        """Arrête le robot proprement"""
        self.init_position()
        self.move_all_to({44:90, 54:130}, 1)
        print("\nDéconnexion...")
        time.sleep(2)
        self.set_all_compliance(False)
        self.close_port()

    def ping_all(self):
        """Teste tous les moteurs"""
        print("\nPing de tous les moteurs...")
        for motor in self.motors:
            try:
                motor.ping()
            except Exception as e:
                print(f"[ERREUR] Moteur {motor.ID} : {str(e)}")
            
    def move_all(self, angles_dic, duration):
        """Déplace les moteurs de X degrés"""
        for ID, angle in angles_dic.items():
            if isinstance(ID, str):
                try:
                    ID = int(ID)
                except:
                    print(f"Le moteur ID:{ID} n'existe pas")
                    continue
            motor = self.motors_dic.get(ID)
            if motor:
                motor.move(angle, duration)
                if(abs(motor.goal_position/motor.degree_to_position - motor.min_angle)==0 or abs(motor.goal_position/motor.degree_to_position - motor.max_angle)==0):
                    print(f"[ID:{ID}]: LIMIT POSITION")
            else:
                print(f"Le moteur ID:{ID} n'existe pas")
        
    def move_all_to(self, angles_dic, duration):
        """Déplace les moteurs vers la position X degrés"""
        for ID, angle in angles_dic.items():
            if isinstance(ID, str):
                try:
                    ID = int(ID)
                except:
                    print(f"Le moteur ID:{ID} n'existe pas")
                    continue
            motor = self.motors_dic.get(ID)
            if motor:
                motor.move_to(angle, duration)
                if(abs(motor.goal_position/motor.degree_to_position - motor.min_angle)==0 or abs(motor.goal_position/motor.degree_to_position - motor.max_angle)==0):
                    print(f"[ID:{ID}]: LIMIT POSITION")
            else:
                print(f"Le moteur ID:{ID} n'existe pas")

    def update_all_speed(self, duration):
        for motor in self.motors:
            motor.set_speed(motor.speed_for_fixed_duration(duration))

    def print_all_positions(self):
        """Récupère la position actuelle de tous les moteurs"""
        print("\nPositions actuelles des moteurs :")
        for motor in self.motors:
            print(f"[ID:{motor.ID}]: {round(motor.ref1_to_ref2(motor.get_position()/motor.degree_to_position), 2)}°")