import json
import threading
from config import *
from motor_controller import MotorController

class Robot:
    """
    High-level helper class to communicate with dynamixel motors.

    All the methods in this class are thread-safe.
    """
    
    
    def __init__(self):
        self.controller = MotorController()
        self.lock = threading.Lock()
        return


    def call_command(self, command, args=[]):
        with self.lock:
            if command == "move_all_to" and len(args) >= 2:
                self.controller.move_all_to(json.loads(args[0]), float(args[1]))
            elif command == "init_position":
                self.controller.init_position()
            elif command == "update_all_speed" and len(args) >= 1:
                self.controller.update_all_speed(float(args[0]))
            else:
                return "Unknown command or missing arguments"
        return None
    

    def read_status(self, status_refresh_rate) -> str:
        data = {"motor_status": [], "status_refresh_rate": status_refresh_rate}
        with self.lock:
            for motor in self.controller.motors:
                position = motor.get_position()
                data["motor_status"].append({
                    "motor_id": motor.ID,
                    "motor_data": {
                        "temperature": motor.get_temperature(),
                        "position": position,
                        "angle": position / motor.degree_to_position
                    }
                })
        return json.dumps(data)


    def stop_robot(self):
        with self.lock:
            self.controller.shut_down()