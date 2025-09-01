from doctest import master
from encodings import utf_8
import signal
import socket
import time
import json

from cv2 import line

from subprocess_manager import SubprocessManager
from logger import Logger
from motion_detection_utils import CONTROL_STRING
from robot_communication import Robot

DEFAULT_BUFFER_SIZE = 2048
MASTER_STRING = "\033[93m[MASTER-SERVER]\033[0m"


class MasterServer:

    def exit_gracefully(self, signum, frame):
        """
        Closes the program and all associated sockets and subprocesses.
        """
        self.__logger.logline(f"{MASTER_STRING} closing server")
        if self.__client:
            self.__client.close()
        self.__socket.close()

        try:
            self.__robot.stop_robot()
        except:
            None
        # needs to shutdown all subprocess/programs
        self.__sp_manager.close_all_current_programs()
        return


    def send_message(self, socket: socket.socket, content) -> None:
        """
        Takes str or bytes `content` and sends it through `socket` connection by prefixing message with byte size.

        :raises: TypeError if content isn't str or bytes.
        """
        if isinstance(content, str):
            buf = bytes(content, "utf_8")
        elif isinstance(content, bytes):
            buf = content
        else:
            raise TypeError("Unsupported content type: accepts only str or bytes.")

        buf = int.to_bytes(4 + len(buf), 4, "big") + buf
        socket.sendall(buf)
        return


    def read_message(self, message: str) -> None:
        """
        Tries parsing command, prints message if failed.
        """
        if not self.try_parse_command(message):
            self.__logger.logline(f"{MASTER_STRING} Received message: \n" + message)

        return


    def try_parse_command(self, str_command: str) -> bool:
        """
        Applies the given command.

        :returns: True if command was parsed successfully, false otherwise. 
        Note that a return value of true does not mean that the command was executed successfully. Check the stdout for more info in such a case.
        """
        try:
            json_command = json.loads(str_command)
        except json.JSONDecodeError as e:
            return False

        for command in json_command["commands"]:
            match command["command_name"]:
                case "say_hello":
                    self.__sp_manager.toggle_program("coucou.py", command["command_value"])
                case "test_prog":
                    self.__sp_manager.toggle_program("subprocess_test_program.py", command["command_value"])
                case "enable_motion_detection":
                    self.__sp_manager.toggle_program("motion_detection_server.py", command["command_value"], self.motion_detection_read_callback)
                case "set_status_refresh_rate":
                    self.set_status_refresh_rate(command["command_value"])
                case "reset_pose":
                    self.__robot.call_command("init_position")
                case _:
                    pass

        return True


    def motion_detection_read_callback(self, line_read: str):
        """
        Callback for motion detection messages:
        - messages starting with {motion_detection_utils.CONTROL_STRING} will send commands to the robot using dynamixel sdk
        - other messages will be logged
        """
        if line_read.startswith(CONTROL_STRING):
            data = line_read.split(";")
            if self.__robot != None:
                ret = self.__robot.call_command(data[1], args=data[2:])
            else:
                ret = None
            if ret != None:
                self.__logger.logline(f"{MASTER_STRING} Unexpected return from motion detection calling robot control command: {ret}")
        else:
            if line_read != "" and not line_read.isspace():
                self.__logger.log(line_read)
        return


    def set_status_refresh_rate(self, rate):
        try: 
            rate = float(rate)
            if rate <= 0:
                raise ValueError("rate cannot be less or equal to 0")
        except Exception as e:
            self.__logger.logerror(f"Couldn't parse status refresh rate: {e}")
            return

        self.__status_refresh_rate = rate
        return


    def __init__(self, host:str = "0.0.0.0", port:int = 4242, logger = Logger()) -> None:
        signal.signal(signal.SIGBREAK, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully) # ISSUE#2: SIGTERM handler does not (exactly) work on windows https://bugs.python.org/issue26350
        # signal.signal(signal.SIGINT, self.exit_gracefully) # See ISSUE#1

        self.__logger = logger
        self.__sp_manager = SubprocessManager(self.__logger)
        try:
            self.__robot = Robot()
        except:
            self.__robot = None
            self.__logger.logline(f"{MASTER_STRING} Robot not connected.")

        self.host: str = host
        self.port: int = port

        self.__socket: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        self.__client: socket.socket = None
        self.__addr: socket._RetAddress = None
        self.__just_disconnected: bool = True

        # writing
        self.__previous_time: float = time.monotonic()
        self.__ping_timer: float = 0
        self.__status_refresh_timer: float = 0
        self.__status_refresh_rate: float = 0.2


        # reading
        self.__is_reading: bool = False
        self.__bytes_to_read: int = 0
        self.__bytes_read: int = 0
        self.__message_read: str = ""
        return


    def start(self):
        """
        Starts the server
        """
        self.__listen()
        self.__main_loop()
        return


    def __listen(self) -> None:
        """
        Binds socket to host:port and starts listening.
        """
        self.__socket.bind((self.host, self.port))
        self.__socket.listen(1)
        self.__socket.settimeout(1.0)
        return

    
    def __connect(self) -> bool:
        """
        Tries to accept an incoming connection (nonblocking, uses timeout).

        :returns: False while not connected, True when connected.
        """
        if self.__client is None:
            if self.__just_disconnected:
                self.__logger.logline(f"{MASTER_STRING} waiting for connexion on port {self.port}")
                self.__just_disconnected = False
            try:
                self.__client, self.__addr = self.__socket.accept()
                self.__client.settimeout(0.2)
                self.__logger.logline(f"{MASTER_STRING} Connection established with client at address {self.__addr}")

            except TimeoutError:
                return False
        return True


    def __write(self, delta_time: float):
        """
        Non-blocking (timeout based) method to send messages to the client.

        This method sends pings every 2 seconds and robot status every x seconds.
        """
        self.__ping_timer += delta_time
        self.__status_refresh_timer += delta_time

        if self.__ping_timer >= 2:
            self.__ping_timer = 0
            #print("[MASTER-SERVER] Sending ping to ", self.__client)
            self.send_message(self.__client, "ping")

        if self.__status_refresh_timer >= self.__status_refresh_rate:
            self.__status_refresh_timer = 0
            # here, we need MotorController to access dynamixel motors data. But only one connection can be opened at once, 
            # so we can't have this and a subprocess like motion detection running in parallel, unless we give them the same
            # controller.
            if self.__robot != None:
                self.send_message(self.__client, self.__robot.read_status(self.__status_refresh_rate))
        return


    def __read(self, delta_time: float):
        """
        Non-blocking (timeout based) method to read incoming messages
        """
        if not self.__is_reading:
            # look for new messages (should start with 4 bytes indicating the byte size of the message)
            self.__message_read = ""
            recv_msg = self.__client.recv(4)
            self.__bytes_to_read = int.from_bytes(recv_msg, "big")
            self.__bytes_read = 4
            if self.__bytes_to_read > 4:
                self.__is_reading = True
        else:
            # receive entire message and read it
            if self.__bytes_read >= self.__bytes_to_read:
                self.read_message(self.__message_read)
                self.__is_reading = False
            else:
                recv_msg = self.__client.recv(DEFAULT_BUFFER_SIZE)
                self.__bytes_read += len(recv_msg)
                self.__message_read += recv_msg.decode("utf-8")
        return


    def __main_loop(self):
        """
        Connection + read/write loop.
        """
        try:
            while(True):
                temp = time.monotonic()
                delta_time = temp - self.__previous_time 
                self.__previous_time = temp

                # try connecting the potential clients
                if (not self.__connect()):
                    print("", end="")
                    # ISSUE#1:
                    # KeyboardInterrupt signal seems to not be handled properly after timeout from socket,
                    # so we add an (invisible to the user) IO operation that seems to make it work
                    # Using signal handler for sigint, although working fine on that aspect does make socket raise an OSError:
                    # [WinError 10038] Une operation a ete tentee sur autre chose qu'un socket
                    # So to not discard potientially important OSErrors, we will keep the KeyboardInterrupt exception
                    continue

                # if here then client connected
                try:
                    self.__write(delta_time)
                    self.__read(delta_time)

                except ConnectionError as e:
                    self.__logger.log_error(f"{MASTER_STRING} connection error.\n{str(e)}")
                    self.__just_disconnected = True
                    self.__client = None
                except ConnectionResetError as e:
                    self.__logger.log_error(f"{MASTER_STRING} connection reset error.\n{str(e)}")
                    self.__just_disconnected = True
                    self.__client = None
                except ConnectionAbortedError as e:
                    self.__logger.log_error(f"{MASTER_STRING} connection aborted error.\n{str(e)}")
                    self.__just_disconnected = True
                    self.__client = None
                except TimeoutError as e:
                    None
                except Exception as e:
                    self.__logger.log_error(f"{MASTER_STRING} error when sending/receiving data.\n{str(e)}")

        except KeyboardInterrupt:
            None
        except Exception as e:
            self.__logger.log_error(f"{MASTER_STRING} Error during program execution: \n{str(e)}")

        finally:
            self.exit_gracefully(signal.SIGINT, None)

        return


if __name__ == "__main__":
    server = MasterServer()

    server.start()