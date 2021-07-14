from naoqi import ALProxy
import zmq
import random
import sys
import time
import qi


motionProxy = ALProxy("ALMotion", "192.168.1.73", 9559)
postureProxy = ALProxy("ALRobotPosture", "192.168.1.73", 9559)
# animation_player_service = session.service("ALAnimationPlayer")
animProxy = ALProxy("ALAnimationPlayer", "192.168.1.73", 9559)

#
# motionProxy.wbEnable(False)
postureProxy.goToPosture("Stand", 1)
#
# # # motionProxy.rest()
# # motionProxy.wbEnable(True)
# # motionProxy.wbFootState("Fixed", "Legs")
# # motionProxy.wbEnableBalanceConstraint(True, "Legs")
#
motionProxy.setBreathEnabled('Body', False)
motionProxy.setBreathEnabled('Head', False)
motionProxy.setIdlePostureEnabled('Body', False)
motionProxy.setIdlePostureEnabled('Head', False)

pNames = "Head"
pStiffnessLists = 0.7
pTimeLists = 0.4
motionProxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)


counter = 0
pose = 44.0
support_mode = "0.0"
port = "5556"
context = zmq.Context()
socket = context.socket(zmq.PAIR)
socket.bind("tcp://*:%s" % port)

print("READY")
start = time.time()
times_ = 0
while True:
    times_ += 1



    if time.time() - start > 0.13:
        msg = socket.recv()
        new_msg = msg.replace("'", "").replace(",", "").replace("(", "").replace(")", "").replace("[", "").replace("]", "").split(" ")

        #
        names = []
        angles = []
        for i in range(len(new_msg)):
            if i % 2 == 0:
                names.append(new_msg[i])
            else:
                angles.append(float(new_msg[i]))

        if angles[-1] != pose or times_ > 320:

            postureProxy.goToPosture("Stand", 0.6)
            times_ = 0
            if int(angles[-1]) == 0:
                animProxy.run("animations/Stand/Gestures/Hey_1")
            if int(angles[-1]) == 2:
                print("MYA")
                names = list()
                times = list()
                keys = list()

                names.append("HeadPitch")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([0.145688, 0.133416, 0.133416, 0.133416, 0.133416])

                names.append("HeadYaw")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([0.024502, 0.0183661, 0.016832, 0.0183661, 0.016832])

                names.append("LAnklePitch")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([-0.173384, -0.173384, -0.173384, -0.173384, -0.173384])

                names.append("LAnkleRoll")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([-0.0183661, -0.0183661, -0.0183661, -0.0183661, -0.0183661])

                names.append("LElbowRoll")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([-1.55083, -1.56004, -1.55237, -1.56004, -1.55237])

                names.append("LElbowYaw")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([-1.18582, -1.17202, -1.25332, -1.17202, -1.25332])

                names.append("LHand")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([0.0251173, 0.0251173, 0.0251173, 0.0251173, 0.0251173])

                names.append("LHipPitch")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([-0.291418, -0.289883, -0.291418, -0.289883, -0.291418])

                names.append("LHipRoll")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([-0.049046, -0.049046, -0.049046, -0.049046, -0.049046])

                names.append("LHipYawPitch")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([-0.162562, -0.162562, -0.162562, -0.162562, -0.162562])

                names.append("LKneePitch")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([0.532256, 0.532256, 0.532256, 0.532256, 0.532256])

                names.append("LShoulderPitch")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([1.22869, 0.674919, 1.64594, 0.674919, 1.64594])

                names.append("LShoulderRoll")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([0.00872665, 0.00872665, 0.00872665, 0.00872665, 0.00872665])

                names.append("LWristYaw")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([-1.11373, -1.10606, -1.10606, -1.10606, -1.10606])

                names.append("RAnklePitch")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([-0.380389, -0.380389, -0.380389, -0.380389, -0.380389])

                names.append("RAnkleRoll")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([0.039926, 0.039926, 0.039926, 0.039926, 0.039926])

                names.append("RElbowRoll")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([1.56165, 1.56012, 1.56012, 1.56012, 1.56012])

                names.append("RElbowYaw")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([1.1704, 1.16733, 1.24863, 1.16733, 1.24863])

                names.append("RHand")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([0.00802646, 0.00802646, 0.00802646, 0.00802646, 0.00802646])

                names.append("RHipPitch")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([-0.291501, -0.291501, -0.291501, -0.291501, -0.291501])

                names.append("RHipRoll")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([-0.101202, -0.101202, -0.101202, -0.101202, -0.101202])

                names.append("RKneePitch")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([0.737896, 0.736361, 0.737896, 0.736361, 0.737896])

                names.append("RShoulderPitch")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([0.840674, 1.50796, 0.561486, 1.50796, 0.561486])

                names.append("RShoulderRoll")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([-0.00872665, -0.00872665, -0.00872665, -0.00872665, -0.00872665])

                names.append("RWristYaw")
                times.append([1.26667, 1.93333, 2.6, 3.26667, 3.93333])
                keys.append([1.13665, 1.13665, 1.13665, 1.13665, 1.13665])





                # motionProxy.rest()

                # Send robot to Pose Init
                # postureProxy.goToPosture("StandInit", 0.5)
                names = list()
                times = list()
                keys = list()

                names.append("RElbowRoll")
                times.append([0.969697])
                keys.append([1.50447])

                names.append("RElbowYaw")
                times.append([0.969697])
                keys.append([0.945968])

                names.append("RHand")
                times.append([0.969697])
                keys.append([0.6])

                names.append("RShoulderPitch")
                times.append([0.969697])
                keys.append([-0.251327])

                names.append("RShoulderRoll")
                times.append([0.969697])
                keys.append([0.111701])

                names.append("RWristYaw")
                times.append([0.969697])
                keys.append([1.20777])


                motionProxy.angleInterpolation(names, keys, times, True)


                # Choregraphe bezier export in Python.
                names = list()
                times = list()
                keys = list()

                names.append("RElbowRoll")
                times.append([0.969697])
                keys.append([[1.50447, [3, -0.333333, 0], [3, 0, 0]]])

                names.append("RElbowYaw")
                times.append([0.969697])
                keys.append([[0.945968, [3, -0.333333, 0], [3, 0, 0]]])

                names.append("RHand")
                times.append([0.969697])
                keys.append([[0.6, [3, -0.333333, 0], [3, 0, 0]]])

                names.append("RShoulderPitch")
                times.append([0.969697])
                keys.append([[-0.251327, [3, -0.333333, 0], [3, 0, 0]]])

                names.append("RShoulderRoll")
                times.append([0.969697])
                keys.append([[0.111701, [3, -0.333333, 0], [3, 0, 0]]])

                names.append("RWristYaw")
                times.append([0.969697])
                keys.append([[1.20777, [3, -0.333333, 0], [3, 0, 0]]])



                motionProxy.angleInterpolationBezier(names, times, keys)

                #motionProxy.rest()

            # if int(angles[-1]) == 1:
            #     animProxy.run("animations/Stand/Gestures/Enthusiastic_6")

            if int(angles[-1]) == 3:
                names = list()
                times = list()
                keys = list()

                names.append("HeadPitch")
                times.append([0.733333, 4.8])
                keys.append([-0.00771196, -0.00771196])

                names.append("HeadYaw")
                times.append([0.733333, 4.8])
                keys.append([-0.00157596, -0.00157596])

                names.append("LAnklePitch")
                times.append([0.733333, 4.8])
                keys.append([-0.0922134, -0.0922134])

                names.append("LAnkleRoll")
                times.append([0.733333, 4.8])
                keys.append([0.0046224, 0.0046224])

                names.append("LElbowRoll")
                times.append([0.733333, 1.06667, 1.33333, 1.6, 1.86667, 2.13333, 2.4, 2.66667, 2.93333, 3.2, 3.46667, 3.73333, 4, 4.8])
                keys.append([-0.312894, -1.20253, -1.20253, -1.20253, -1.20253, -1.20253, -1.20253, -1.20253, -1.20253, -1.20253, -1.20253, -1.20253, -1.20253, -0.312894])

                names.append("LElbowYaw")
                times.append([0.733333, 1.06667, 1.2, 1.33333, 1.46667, 1.6, 1.73333, 1.86667, 2, 2.13333, 2.26667, 2.4, 2.53333, 2.66667, 2.8, 2.93333, 3.06667, 3.2, 3.33333, 3.46667, 3.6, 3.73333, 3.86667, 4, 4.13333, 4.8])
                keys.append([-0.770111, -0.541052, -0.907571, -0.541052, -0.907571, -0.541052, -0.907571, -0.541052, -0.907571, -0.541052, -0.907571, -0.541052, -0.907571, -0.541052, -0.907571, -0.541052, -0.907571, -0.541052, -0.907571, -0.541052, -0.907571, -0.541052, -0.907571, -0.541052, -0.907571, -0.770111])

                names.append("LHand")
                times.append([0.733333, 1.06667, 1.33333, 1.6, 1.86667, 2.13333, 2.4, 2.66667, 2.93333, 3.2, 3.46667, 3.73333, 4, 4.8])
                keys.append([0.916751, 0.945455, 0.945455, 0.945455, 0.945455, 0.945455, 0.945455, 0.945455, 0.945455, 0.945455, 0.945455, 0.945455, 0.945455, 0.916751])

                names.append("LHipPitch")
                times.append([0.733333, 4.8])
                keys.append([0.0442645, 0.0442645])

                names.append("LHipRoll")
                times.append([0.733333, 4.8])
                keys.append([-0.0155321, -0.0155321])

                names.append("LHipYawPitch")
                times.append([0.733333, 4.8])
                keys.append([0.00916048, 0.00916048])

                names.append("LKneePitch")
                times.append([0.733333, 4.8])
                keys.append([0.0564382, 0.0564382])

                names.append("LShoulderPitch")
                times.append([0.733333, 1.06667, 1.33333, 1.6, 1.86667, 2.13333, 2.4, 2.66667, 2.93333, 3.2, 3.46667, 3.73333, 4, 4.8])
                keys.append([1.57231, 0.994838, 0.994838, 0.994838, 0.994838, 0.994838, 0.994838, 0.994838, 0.994838, 0.994838, 0.994838, 0.994838, 0.994838, 1.57231])

                names.append("LShoulderRoll")
                times.append([0.733333, 1.06667, 1.33333, 1.6, 1.86667, 2.13333, 2.4, 2.66667, 2.93333, 3.2, 3.46667, 3.73333, 4, 4.8])
                keys.append([0.31903, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.31903])

                names.append("LWristYaw")
                times.append([0.733333, 1.06667, 1.33333, 1.6, 1.86667, 2.13333, 2.4, 2.66667, 2.93333, 3.2, 3.46667, 3.73333, 4, 4.8])
                keys.append([-1.00941, 0.254818, 0.254818, 0.254818, 0.254818, 0.254818, 0.254818, 0.254818, 0.254818, 0.254818, 0.254818, 0.254818, 0.254818, -1.00941])

                names.append("RAnklePitch")
                times.append([0.733333, 4.8])
                keys.append([-0.0889992, -0.0889992])

                names.append("RAnkleRoll")
                times.append([0.733333, 4.8])
                keys.append([-0.00149427, -0.00149427])

                names.append("RElbowRoll")
                times.append([0.733333, 1.06667, 1.2, 1.33333, 1.46667, 1.6, 1.73333, 1.86667, 2, 2.13333, 2.26667, 2.4, 2.53333, 2.66667, 2.8, 2.93333, 3.06667, 3.2, 3.33333, 3.46667, 3.6, 3.73333, 3.86667, 4, 4.13333, 4.8])
                keys.append([0.308375, 1.09956, 0.787143, 1.09956, 0.787143, 1.09956, 0.787143, 1.09956, 0.787143, 1.09956, 0.787143, 1.09956, 0.787143, 1.09956, 0.787143, 1.09956, 0.787143, 1.09956, 0.787143, 1.09956, 0.787143, 1.09956, 0.787143, 1.09956, 0.787143, 0.308375])

                names.append("RElbowYaw")
                times.append([0.733333, 1.06667, 1.2, 1.33333, 1.46667, 1.6, 1.73333, 1.86667, 2, 2.13333, 2.26667, 2.4, 2.53333, 2.66667, 2.8, 2.93333, 3.06667, 3.2, 3.33333, 3.46667, 3.6, 3.73333, 3.86667, 4, 4.13333, 4.8])
                keys.append([0.770025, 0.541052, 0.518363, 0.541052, 0.518363, 0.541052, 0.518363, 0.541052, 0.518363, 0.541052, 0.518363, 0.541052, 0.518363, 0.541052, 0.518363, 0.541052, 0.518363, 0.541052, 0.518363, 0.541052, 0.518363, 0.541052, 0.518363, 0.541052, 0.518363, 0.770025])

                names.append("RHand")
                times.append([0.733333,

                1.06667, 1.33333, 1.6, 1.86667, 2.13333, 2.4, 2.66667, 2.93333, 3.2, 3.46667, 3.73333, 4, 4.8])
                keys.append([0.917114, 0.945455, 0.945455, 0.945455, 0.945455, 0.945455, 0.945455, 0.945455, 0.945455, 0.945455, 0.945455, 0.945455, 0.945455, 0.917114])

                names.append("RHipPitch")
                times.append([0.733333, 4.8])
                keys.append([0.032157, 0.032157])

                names.append("RHipRoll")
                times.append([0.733333, 4.8])
                keys.append([0.0046224, 0.0046224])

                names.append("RKneePitch")
                times.append([0.733333, 4.8])
                keys.append([0.0624798, 0.0624798])

                names.append("RShoulderPitch")
                times.append([0.733333, 1.06667, 1.33333, 1.6, 1.86667, 2.13333, 2.4, 2.66667, 2.93333, 3.2, 3.46667, 3.73333, 4, 4.8])
                keys.append([1.57239, 1.09956, 1.09956, 1.09956, 1.09956, 1.09956, 1.09956, 1.09956, 1.09956, 1.09956, 1.09956, 1.09956, 1.09956, 1.57239])

                names.append("RShoulderRoll")
                times.append([0.733333, 1.06667, 1.33333, 1.6, 1.86667, 2.13333, 2.4, 2.66667, 2.93333, 3.2, 3.46667, 3.73333, 4, 4.8])
                keys.append([-0.309909, -0.0942478, -0.0942478, -0.0942478, -0.0942478, -0.0942478, -0.0942478, -0.0942478, -0.0942478, -0.0942478, -0.0942478, -0.0942478, -0.0942478, -0.309909])

                names.append("RWristYaw")
                times.append([0.733333, 1.06667, 1.33333, 1.6, 1.86667, 2.13333, 2.4, 2.66667, 2.93333, 3.2, 3.46667, 3.73333, 4, 4.8])
                keys.append([0.989389, 1.22173, 1.22173, 1.22173, 1.22173, 1.22173, 1.22173, 1.22173, 1.22173, 1.22173, 1.22173, 1.22173, 1.22173, 0.989389])

                motionProxy.angleInterpolationBezier(names, times, keys)

            pose = angles[-1]


        times = [0.11] * len(names)
        new_names = []
        new_angles = []
        new_times = []
        for i in range(len(names)):
            if names[i] == "HeadYaw" or names[i] == "HeadPitch":
                new_names.append(names[i])
                new_angles.append(angles[i])
                new_times.append(times[i])
    
