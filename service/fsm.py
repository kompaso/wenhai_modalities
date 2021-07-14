from common import *

class FSM_processor:
    def __init__ (self, config_file = ""):
        self.fsms = []
        self.active_fsm = -1
        self.current_state = 0

        if (config_file == ""):
            files = sorted (Path ("/Users/elijah/Dropbox/Programming/RoboCup/remote control/tasks/").glob('*.txt'))
            #print (files)

            fsm_num = 0

            for filename in files:
                print (filename)
                file = open (filename)
                ln = file.readline ()

                self.fsms.append ([])

                while (ln != ""):
                    self.fsms [fsm_num].append (ln [:-1])
                    ln = file.readline ()

                print ("fsmmmm", self.fsms [fsm_num])

                fsm_num += 1

            if (len (self.fsms) > 0):
                self.active_fsm = 0

        else:
            print ("to be implemented")

    def handle_command (self, command):
        #print ("command to fsm proc: ", command)

        if (command [0] [0] [0] == "/"):
            return [command]

        elif (command [0] [0] == "noaction"):
            return [command]

        else:
            if (command [0] [0] == "next"):
                print ("command next")
                curr_st = self.fsms [self.active_fsm] [self.current_state]

                self.current_state += 1

                if (self.current_state >= len (self.fsms [self.active_fsm])):
                    self.current_state = 0

                return [curr_st [0], curr_st [1]]
