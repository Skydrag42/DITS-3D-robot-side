import os
from subprocess import Popen, PIPE, CREATE_NEW_PROCESS_GROUP
import signal
import threading

class SubprocessManager:

    def __init__(self, logger):
        self.__logger = logger
        self.__processes: dict[str, Popen] = {}
        self.__threads: dict[str, threading.Thread] = {}
        return


    def __process_stdout_reader(self, process: str, callback):
        while self.__processes[process] != None and self.__processes[process].poll() == None:
            line = str(self.__processes[process].stdout.readline(), encoding="utf-8")
            callback(line)
        return


    # TODO: 
    # implement read_callback for piping stderr?
    # find a way to add simple writing to piped stdin?
    def toggle_program(self, program: str, value: bool, read_callback=None):
        if program not in self.__processes:
            self.__processes[program] = None

        path = os.path.join(os.path.dirname(__file__), program)
        if value and (self.__processes[program] == None or self.__processes[program].poll() != None):
            # program not yet started, asked to start
            self.__logger.logline(f"Starting subprocess {path}")
            try:
                self.__processes[program] = Popen(["python", path], 
                                                  stdout=PIPE if read_callback != None else None, 
                                                  creationflags=CREATE_NEW_PROCESS_GROUP)
                self.__logger.logline(f"pid is {self.__processes[program].pid}")

                # create read callback here
                if read_callback != None:
                    self.__threads[program] = threading.Thread(target=self.__process_stdout_reader, args=[program, read_callback])
                    self.__threads[program].start()

            except Exception as e:
                self.__logger.log_error(f"Error when setting up new subprocess:\n {e}")
        elif not value and self.__processes[program] != None and self.__processes[program].poll() == None:
            # program running, asked to stop
            self.__logger.logline(f"Trying to stop subprocess {self.__processes[program].pid} by sending SIGBREAK")
            self.__processes[program].send_signal(signal.CTRL_BREAK_EVENT)
        return


    def close_all_current_programs(self):
        for process in self.__processes.values():
            if process != None and process.poll() == None:
                self.__logger.logline(f"Trying to stop subprocess {process.pid} by sending SIGBREAK")
                process.send_signal(signal.CTRL_BREAK_EVENT)
        for process in self.__processes.values():
            process.wait()
        return