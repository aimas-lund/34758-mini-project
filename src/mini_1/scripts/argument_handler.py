import argparse


class ArgumentHandler:

    def __init__(self):
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument('-m', '--refresh', help="Run in Refresh-mode")

    
    def should_run_refresh_mode(self):
        args = self.parser.parse_args()
        return args.refresh
    