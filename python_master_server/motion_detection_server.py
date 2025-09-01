# Code from PROJET-S8 slightly modified to fit the new architecture

import json
import pickle
import signal
import socket
import struct
from sys import stderr
import sys
import time
from config import *
from logger import Logger
from motion_detection_utils import *


class MotionDetectionServer:


    def __init__(self, host, port, logger: Logger):
        self.__logger = logger

        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client = None

        self.time = time.time()

        signal.signal(signal.SIGTERM, self.exit_gracefully) # ISSUE#2
        # signal.signal(signal.SIGINT, self.exit_gracefully) # ISSUE#2
        signal.signal(signal.SIGBREAK, self.exit_gracefully)
        self.looping = True
        return


    def exit_gracefully(self, signum, frame):
        self.looping = False
        self.stop()
        return


    def start(self):
        """Démarre le serveur et attend une connexion"""
        self.socket.bind((self.host, self.port))
        self.socket.listen(1)
        self.socket.settimeout(1.0)
        
        self.__logger.logline(f"{HEAD_STRING} Waiting for incoming connection on port: {self.port}")
        return


    def stop(self):
        """Ferme la connexion entre les clients et le serveur"""
        self.deco()
        self.socket.close()
        self.__logger.logline(f"{HEAD_STRING} Closing...")
        
        return
        

    def deco(self):
        if self.client:
            self.client.close()
            self.client = None
        return

        
    def send_all(self, sock, data):
        sent = 0
        to_send = len(data)
        while sent < to_send:
            ret = sock.send(data[sent:])
            if ret == 0:
                self.__logger.logline(f"{INFO_STRING} Connection interrupted")
                self.exit_gracefully(signal.SIGBREAK, None)
                # TODO: write message to replace call to ctrl
                self.__logger.logline(f"{CONTROL_STRING};init_position")
                #self.ctrl.init_position()
                return
            sent += ret
        return

            
    def recv_all(self, sock, to_receive):
        data = b''
        received = 0
        while received < to_receive:
            chunk = sock.recv(to_receive - received)
            if not chunk:
                self.__logger.logline(f"{INFO_STRING} Connection interrupted")
                self.exit_gracefully(signal.SIGBREAK, None)
                # TODO: write message to replace call to ctrl
                self.__logger.logline(f"{CONTROL_STRING};init_position")
                #self.ctrl.init_position()
                return
            data += chunk
            received += len(chunk)
        return data
    

    def main(self):

        try:
            self.start()
            while self.looping:
                self.__logger.logline(f"{HEAD_STRING} Waiting...")

                if self.client is None:
                    try:
                        self.client, addr = self.socket.accept()
                        self.__logger.logline(f"{HEAD_STRING} Connection established with client")
                        # compliance is already set by master server
                        #self.ctrl.set_all_compliance(True)
                    except socket.timeout:
                        continue


                try:
                    time.sleep(server_rate)
                    self.__logger.logline(f"{CONTROL_STRING};update_all_speed;{self.time-time.time()}")
                    #self.ctrl.update_all_speed(self.time-time.time())
                    size_data_raw = self.recv_all(self.client, 4)
                    if not size_data_raw:
                        continue
                    size_data = struct.unpack('!I', size_data_raw)[0]
                    size_data_raw = None

                    data = self.recv_all(self.client, size_data)
                    size_data = None
                    if not data:
                        continue

                    command = pickle.loads(data)
                    if not command:
                        continue
                    self.__logger.logline(f"{HEAD_STRING}: received {command}") 
                    # execute commande
                    angles_dic = {k: v for k, v in command.items() if isinstance(k, int)}
                    duration = command.get("rate", 0.1)
                    
                    # TODO: log message instead of using ctrl
                    self.__logger.logline(f"{CONTROL_STRING};move_all_to;{json.dumps(angles_dic)};{duration}")
                    #self.ctrl.move_all_to(angles_dic, duration)

                    self.time = time.time() + duration
                    command = None

                
                except ConnectionError as e:
                    self.__logger.log_error(f"{HEAD_STRING} connection error.\n{str(e)}")
                    self.client = None
                except ConnectionResetError as e:
                    self.__logger.log_error(f"{HEAD_STRING} connection reset error.\n{str(e)}")
                    self.client = None
                except ConnectionAbortedError as e:
                    self.__logger.log_error(f"{HEAD_STRING} connection aborted error.\n{str(e)}")
                    self.client = None
                except TimeoutError as e:
                    None
                except Exception as e:
                    self.__logger.log_error(f"{HEAD_STRING} Error in motion detection server.\n{str(e)}")
                    # self.exit_gracefully()
                    break  

        except KeyboardInterrupt:
            self.__logger.logline(f"{HEAD_STRING} Manual stop")
            self.exit_gracefully(signal.SIGINT, None)
        return


if __name__ == "__main__":
    port = 2769
    if len(sys.argv) > 1:
        port = int(sys.argv[1])

    logger = Logger()
    ser = MotionDetectionServer("0.0.0.0", port, logger)
    ser.main()