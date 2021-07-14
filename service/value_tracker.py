import numpy as np
import cv2

class Value_tracker:
    def __init__ (self, draw_ = True):
        self.tracked = {}
        self.draw_ = draw_

    def name (self):
        return "value_tracker"

    def update (self, value_name, value):
        self.tracked.update ({value_name : value})

    def draw (self, img):
        if (self.draw_ == False):
            return []

        result = np.array (img)

        i = 0
        for k, v in self.tracked.items():
            # print("AAAAAAAAAAAAA", v)
            result = cv2.putText (result, k + ": " + str (v), (30, 60 * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX,
                                 2, (100, 25, 130), 3, cv2.LINE_AA)
            i += 1

        return [result]
