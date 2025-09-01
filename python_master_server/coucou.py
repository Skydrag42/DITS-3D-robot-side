# Code from PROJET-S8 slightly modified to fit the new architecture

import signal
import time
from motor_controller import MotorController


class Coucou:

    def __init__(self):
        self.ctrl = MotorController()

        signal.signal(signal.SIGTERM, self.exit_gracefully)
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGBREAK, self.exit_gracefully)


    def exit_gracefully(self, signum, frame):
        self.ctrl.set_all_compliance(False)
        self.ctrl.close_port()


    def run(self):
        # Initialisation du contrôleur
        try:
    
            self.ctrl.ping_all()
            self.ctrl.set_all_compliance(True)

            print("\nPositions actuelles des moteurs :")
            for motor in self.ctrl.motors:
                print(f"[ID:{motor.ID}] {motor.name}: {round(motor.get_position()/motor.degree_to_position, 2)}°")

            #Déplacement d'un moteur
            interv = 1
    
            self.ctrl.init_position()
    
            time.sleep(1)
            self.ctrl.motors_dic[52].move_to(90, 1)
            time.sleep(interv)
            self.ctrl.motors_dic[54].move_to(-90, 1)
            time.sleep(interv)
            self.ctrl.motors_dic[54].move_to(90, 1)
            time.sleep(interv)
            self.ctrl.motors_dic[54].move_to(-90, 1)
            time.sleep(interv)
            self.ctrl.motors_dic[54].move_to(90, 1)
            time.sleep(interv)
            self.ctrl.motors_dic[52].move_to(-90, 1)
            time.sleep(interv)

        except Exception as e:
            print(f"Erreur : {e}")

        finally:
            print("\nDéconnexion...")
            time.sleep(1)
            self.exit_gracefully(signal.SIGINT, None)
        return


if __name__ == "__main__":
    prog = Coucou()
    prog.run()
