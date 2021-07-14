from common import *
from processors import *
import json

class Criterion:
    def __init__ (self):
        pass

    def name (self):
        pass

class Key_criterion (Criterion):
    def __init__(self, key):
        Criterion.__init__ ()
        self.key = key_

    def passed (self, key):
        if (self.key == key):
            return True

        return False

criterions_list = {"key" : Key_criterion}

#class Card_criterion (Criterion):
#    def __init__ (self):
#        Criterion.__init__ (self)

#class Skeleton_criterion (Criterion):
#    def __init__ (self):
#        Criterion.__init__ (self)

class State:
    def __init__ (self, name_, state):
        criterion = state ["criterion"]
        self.criterion = criterions_list [criterion ["type"]] (criterion ["parameter"])

    def passage (self, event):
        new_state_name = ""

        for state in self.conditions.keys ():
            if (condition [state].passed (event) == True):
                if (new_state_name != ""):
                    print ("Warning: two or more states can be reached")

                new_state_name = state

        return new_state_name

class Task (Unit):
    def __init__ (self, config_file):
        Unit.__init__ (self)

        data = json.load (open (file))

        self.name = data ["name"]
        self.required_modalities = data ["required_modalities"]
        self.required_processors = data ["required_processors"]

        print (self.name, " task, ", self.required_modalities, "required modalities")

        self.states = [State (state) for state in states]

        if (len (self.states) != 0):
            self.curr_state = 0

        self.processors = {}

        for processor in self.required_processors:
            self.processors.update ({processor : processors.processors_list [processor] ()})

    def name (self):
        return "Task"

    # for a in data["filters"]:
    #    print(a["name"])

    def handle_command (self, command):
        #print ("command to fsm proc: ", command)



    def get_command (self):
        return "noaction"

class Tasks (Unit):
    def __init__ (self, tasks_path = ""):
        Unit.__init__ (self)

        self.tasks = {}
        self.data  = {}

        if (tasks_path == ""):
            files = sorted (Path ("/Users/elijah/Dropbox/Programming/RoboCup/remote control/tasks/").glob('*.json'))
            print (files)

            for file in files:
                task = Task (file)
                self.tasks.update ({task.name : task})

            if (len (self.tasks) > 0):
                self.current_task = 0

        else:
            print ("Non-default path to tasks config folder is to be implemented")

    def name (self):
        return "Tasks"

    def read_data (self):
        data = {}

        for modality in inputs.keys():
            skip_reading_data = False

            if (modality == "computer keyboard"):
                skip_reading_data = True

            modality_data = inputs [modality].read_data (skip_reading_data)
            data.update ({modality: modality_data})

    def process_data (self):
        pass

    def get_action ():

