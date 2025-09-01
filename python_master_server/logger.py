import sys


class Logger:

    def __init__(self, out=sys.stdout, err=sys.stderr):
        self.out = out
        self.err = err

    def log(self, message):
        print(message, file=self.out, end="", flush=True)

    def logline(self, message):
        print(message, file=self.out, end="\n", flush=True)

    def log_error(self, message):
        print(message, file=self.err, flush=True)
