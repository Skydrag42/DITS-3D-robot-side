# Code from PROJET-S8

from config import *
import matplotlib.pyplot as plt

class Motor:
    def __init__(self, ID, model, name, min_angle, max_angle, offset, direction):
        """Initialise un moteur avec un ID, un nom et une plage d'angles"""
        self.ID = ID
        self.name = name
        
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.max_speed = 256 #u #max=1023
        self.max_accel = 254 #u

        self.offset = offset
        self.direction = direction

        self.rpm_unit = RPM_UNIT.get(model, 0)
        self.degree_to_position = MODEL_RESOLUTION.get(model, 0)
        
        self.goal_position, _, _ = packetHandler.read2ByteTxRx(portHandler, ID, ADDR_PRESENT_POSITION)

        self.times = [0]
        self.positions = [self.goal_position/self.degree_to_position]
        self.goal_positions = [self.goal_position/self.degree_to_position]
        self.goal_speeds = [0]
        self.speeds = [0]
        self.accels = [0]

    def move(self, angle, duration):
        """Déplace le moteur de X degrés"""
        self.goal_position += int(self.direction * angle * self.degree_to_position)
        self.goal_position = max(self.min_angle * self.degree_to_position, min(self.max_angle * self.degree_to_position, self.goal_position))

        self.set_speed(1)
        self.set_accel(0)
        
        packetHandler.write2ByteTxRx(portHandler, self.ID, ADDR_GOAL_POSITION, int(round(self.goal_position)))
        self.set_speed(self.speed_for_fixed_duration(duration))
        print(f"[ID:{self.ID}]: {round(self.ref1_to_ref2(self.get_position()/self.degree_to_position), 2)}° -> {round(self.ref1_to_ref2(self.goal_position/self.degree_to_position), 2)}°")
        
        self.m_update(duration)

    def move_to(self, angle, duration):
        """Déplace le moteur vers la position X degrés"""
        self.goal_position = int(self.ref2_to_ref1(angle) * self.degree_to_position)
        self.goal_position = max(self.min_angle * self.degree_to_position, min(self.max_angle * self.degree_to_position, self.goal_position))

        self.set_speed(1)
        self.set_accel(0)

        packetHandler.write2ByteTxRx(portHandler, self.ID, ADDR_GOAL_POSITION, int(round(self.goal_position)))
        self.set_speed(self.speed_for_fixed_duration(duration))
        print(f"[ID:{self.ID}]: {round(self.ref1_to_ref2(self.get_position()/self.degree_to_position), 2)}° -> {round(self.ref1_to_ref2(self.goal_position/self.degree_to_position), 2)}°")

        self.m_update(duration)
        
    def get_position(self):
        """Récupère la position actuelle du moteur"""
        present_position, _, _ = packetHandler.read2ByteTxRx(portHandler, self.ID, ADDR_PRESENT_POSITION)
        return present_position # u

    def get_temperature(self):
        """Récupère la position actuelle du moteur"""
        present_temperature, _, _ = packetHandler.read2ByteTxRx(portHandler, self.ID, ADDR_PRESENT_TEMPERATURE)
        return present_temperature

    def ref1_to_ref2(self, angle_ref1):
        """Convertion d'un angle dans le ref du moteur vers le ref de la camera"""
        angle_ref2 = self.direction * (angle_ref1 - self.offset)
        return angle_ref2

    def ref2_to_ref1(self, angle_ref2):
        """Convertion d'un angle dans le ref de la camera vers le ref du moteur"""
        angle_ref1 = self.direction * angle_ref2 + self.offset
        return angle_ref1
    
    def set_speed(self, speed_dps):
        """Ajuste la vitesse de rotation à une valeur de X degrès/seconde"""
        speed_rpm = speed_dps/6 # rpm
        speed = min(speed_rpm/self.rpm_unit, self.max_speed)
        if speed > 0 and speed < 1:
            speed = 1
        packetHandler.write2ByteTxRx(portHandler, self.ID, ADDR_GOAL_SPEED, int(round(speed)))
    
    def speed_for_fixed_duration(self, duration): # s
        """Ajuste la vitesse de rotation pour aller jusqu'à la position cible en X secondes"""
        distance = abs(self.goal_position - self.get_position()) # u
        speed_dps = distance/(self.degree_to_position*duration) # °/s
        return speed_dps # °/s
    
    def set_accel(self, accel_dpsc): # °/s²
        """Ajuster l'accélération à une valeur de X degrès/seconde²"""
        accel = min(abs(accel_dpsc/ACCEL_UNIT), self.max_accel)
        if accel > 0 and accel < 1:
            accel = 1
        packetHandler.write1ByteTxRx(portHandler, self.ID, ADDR_GOAL_ACCELERATION, int(round(accel)))
        
    def set_compliance(self, enable):
        """Active ou désactive le couple d'un moteur"""
        value = TORQUE_ENABLE if enable else TORQUE_DISABLE
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.ID, ADDR_TORQUE_ENABLE, value)

        if dxl_comm_result != 0:
            print(f"Erreur de communication [ID:{self.ID}]: {packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"Erreur du moteur [ID:{self.ID}]: {packetHandler.getRxPacketError(dxl_error)}")
        else:
            state = "activé" if enable else "désactivé"
            print(f"[ID:{self.ID}]: Couple {state}")
            
    def ping(self):
        """Teste la connexion avec le moteur"""
        dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, self.ID)
        
        if dxl_comm_result != 0:
            raise RuntimeError(f"Erreur de communication [ID:{self.ID}]: {packetHandler.getTxRxResult(dxl_comm_result)}")
        
        if dxl_error != 0:
            raise RuntimeError(f"Erreur du moteur [ID:{self.ID}]: {packetHandler.getRxPacketError(dxl_error)}")

        model_name = MODEL_TYPES.get(dxl_model_number, f"Inconnu ({dxl_model_number})")
        print(f"[ID:{self.ID}] ({model_name}): PING SUCCES")

    def m_plot(self):
        
        # Création des subplots
        fig, axs = plt.subplots(2, 1, figsize=(8, 10), sharex=True)

        # Position
        axs[0].plot(self.times, self.positions, label='Current Position', color='b')
        axs[0].plot(self.times, self.goal_positions, label='Goal Position', color='violet', linestyle='dashed')
        axs[0].set_ylabel('Position (°)')
        axs[0].legend()
        axs[0].grid()

        # Vitesse
        axs[1].plot(self.times, self.speeds, label='Current Speed', color='g')
        axs[1].plot(self.times, self.goal_speeds, label='Goal Speed', color='orange', linestyle='dashed')
        axs[1].set_xlabel('Time (s)')
        axs[1].set_ylabel('Speed (°/s)')
        axs[1].legend()
        axs[1].grid()
        
        plt.show()

    def m_update(self, rate):
        self.times.append(self.times[-1]+rate)
        self.positions.append(self.get_position()/self.degree_to_position)
        self.goal_positions.append(self.goal_position/self.degree_to_position)
        self.speeds.append(packetHandler.read1ByteTxRx(portHandler, self.ID, ADDR_PRESENT_SPEED)[0] * self.rpm_unit * 6)
        self.goal_speeds.append(packetHandler.read2ByteTxRx(portHandler, self.ID, ADDR_GOAL_SPEED)[0] * self.rpm_unit * 6)
        self.accels.append(packetHandler.read1ByteTxRx(portHandler, self.ID, ADDR_GOAL_ACCELERATION)[0] * ACCEL_UNIT)