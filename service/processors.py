from common import *

class Processor (Unit):
    def __init__ (self):
        Unit.__init__ (self)

    def name (self):
        return "Processor"

class Online_keyboard_control (Processor):
    def __init__ (self, config_file_ = "", log_file_ = ""):
        Processor.__init__ ()

        if (config_file_ == ""):

        else:
            print ("To be implemented")

        if (log_file_ == ""):
            print ("No path to log given to ")
            self.log_file = log_file_

    def process (self):


        return F

    def write

processors_list = {"online keyboard control": Online_keyboard_control}
