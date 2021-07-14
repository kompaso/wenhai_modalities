import service.fsm as fsm
from common import *
from time import time, sleep
from service.value_tracker import Value_tracker
import service.input_output as input_output

class Manager:
    def __init__ (self, config_ = "", silent_mode_ = True, time_to_not_silent_ =  0, color_ = 190, draw_tracker_ = True):
        self.inputs = {}
        self.robots_list = {}
        self.silent_mode = silent_mode_
        self.time_to_not_silent = time_to_not_silent_
        self.color = color_
        self.quit = False
        self.draw_tracker = draw_tracker_

    def __del__ (self):
        self.logfile.close ()
        cv2.destroyAllWindows()

    def create_window (self, WIND_X, WIND_Y):
        self.WIND_X = WIND_X
        self.WIND_Y = WIND_Y

        cv2.namedWindow("remote_controller", cv2.WINDOW_AUTOSIZE)
        cv2.resizeWindow("remote_controller", (WIND_Y, WIND_X))
        self.canvas = np.ones ((WIND_Y, WIND_X, 3), np.uint8) * self.color

    def init (self):
        self.curr_time = time()
        self.logfile = open("log/" + str(self.curr_time) + ".txt", "w+")
        self.tracker = Value_tracker (self.draw_tracker)

        self.fsm_processor = fsm.FSM_processor ()
        self.start_time = self.curr_time

    def add_inputs (self, inputs):
        self.inputs.update (inputs)

    def add_robots (self, robots):
        self.robots_list.update (robots)

    def form_output_image (self, window_x_sz = -1, one_img_x_sz = -1):
        result = input_output.form_grid (self.output_images, window_x_sz, one_img_x_sz)

        return result

    def handle_modalities (self):
        self.output_images = []
        self.output_names  = []

        self.inputs["computer keyboard"][0]._read_data()
        keyboard_data = self.inputs["computer keyboard"][0].get_read_data()

        #print ("kb data:",keyboard_data)

        if (keyboard_data == ord("q")):
            self.quit = True

        if (keyboard_data == ord("-")):
            self.silent_mode = not self.silent_mode

        if (self.curr_time - self.start_time >= self.time_to_not_silent):
            self.silent_mode = False
            self.time_to_not_silent = 100000

        for modality in self.inputs.keys ():
            skip_reading_data = False

            if (modality == "computer keyboard"):
                skip_reading_data = True

            command = self.inputs [modality] [0].get_command (skip_reading_data)

            # print ("command", command)

            self.logfile.write (str (self.curr_time) + str (command))

            action = self.fsm_processor.handle_command (command)

            if (self.silent_mode == False):
                for key in self.inputs [modality] [1]:
                    if (key in self.robots_list.keys ()):
                        # print ("adding action", key, action)
                        self.robots_list [key].add_action (action)

            modality_frames = self.inputs [modality] [0].draw (self.canvas)

            #print ("shape", modality, modality_frames [0].shape [0])

            if (modality_frames [0].shape [0] > 1):
                self.output_images += modality_frames
                self.output_names.append (modality)

    def handle_robots (self):
        self.canvas = np.ones ((self.WIND_Y, self.WIND_X, 3), np.uint8) * self.color
        canvas_ = self.canvas.copy ()

        if (self.draw_tracker == True):
            self.output_images += self.tracker.draw (self.canvas)

        if (self.silent_mode == False):
            for key in self.robots_list.keys ():
                # print(key)
                self.robots_list [key].on_idle ()

        list (self.robots_list.items ()) [0] [1].plot_state (canvas_, 150, 40, 2.5)

        self.output_images.append (canvas_)
        self.output_names.append  ("remote controller")

    def on_idle (self):
        new_time = time ()
        #print (new_time - self.curr_time)
        self.curr_time = new_time
        # self.tracker.update("time", self.curr_time)

        self.handle_modalities ()
        self.handle_robots     ()

        if (self.silent_mode == True):
            self.canvas = cv2.putText (self.canvas, "silent mode", (30, 100), cv2.FONT_HERSHEY_SIMPLEX,
                   1, (0, 255, 0), 2, cv2.LINE_AA)

        sleep  (0.02)

        return {"quit" : self.quit}
