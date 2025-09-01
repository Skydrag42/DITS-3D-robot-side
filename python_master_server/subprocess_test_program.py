import signal
import time


class SubprocTestProg:

    def __init__(self):
        signal.signal(signal.SIGTERM, self.exit_gracefully) # ISSUE#2
        signal.signal(signal.SIGINT, self.exit_gracefully) # ISSUE#2
        signal.signal(signal.SIGBREAK, self.exit_gracefully)
        self.looping = True

    def exit_gracefully(self, signum, frame):
        print("exiting gracefully")
        self.looping = False


    def run(self):
        while self.looping:
            print("bouh")
            time.sleep(1)
        return


if __name__ == "__main__":
    prog = SubprocTestProg()
    prog.run()
