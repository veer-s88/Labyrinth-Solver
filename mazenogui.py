from threading import Thread
import cv2
import numpy as np
import imutils
import pyfirmata
from pyfirmata import Arduino# ArduinoMega has different pin arrangement
import time
import serial
import serial.tools.list_ports
from scipy import spatial
import math

I_terms = [0, 0]
previous_times = [time.time(), time.time()]
setPoints = [0, 0]
prev_errors = [0, 0]
prev_angles = [0, 0]
prev_positions = [0, 0]
angles = [0,0]

class WebcamVideoStream:
    def __init__(self, src=0):
        # define inital variables, assume frames from laptop webcam
        self.stream = cv2.VideoCapture(src)
        self.grabbed, self.frame = self.stream.read()
        self.stopped = False

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                self.stream.release()
                return
            self.grabbed, self.frame = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True


def draw_circle(mask, frame):
    # find contours in the masked image
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    centre = None

    if len(contours) > 0:
        # find the largest contour in the mask an assume that is the object we want to detect
        # use this to compute minumium closing circle and centroid
        max_contour = max(contours, key=cv2.contourArea)  # *inside contour area
        ((x, y), radius) = cv2.minEnclosingCircle(max_contour)
        M = cv2.moments(max_contour)
        if M["m00"] > 0:
            centre = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            print(centre)

        if radius > 10:
            # draw the circle and centroid on the frame
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, centre, 5, (0, 255, 255), -1)
    return frame, centre


def draw_circles(mask, frame):
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    centre = None
    contour_list = [None, None, None, None]

    if len(contours) > 0:
        sorted_contours = (sorted(contours, key=cv2.contourArea, reverse=True))[:4]

        for c in sorted_contours:
            M = cv2.moments(c)
            if M["m00"] > 0:
                centre = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                if centre[0] < 150 and centre[1] > 330:
                    contour_list[0] = centre
                    cv2.circle(frame, centre, 3, (0, 255, 255), -1)
                elif centre[0] < 150 and centre[1] < 150:
                    contour_list[1] = centre
                    cv2.circle(frame, centre, 3, (0, 255, 255), -1)
                elif centre[0] > 450 and centre[1] > 330:
                    contour_list[2] = centre
                    cv2.circle(frame, centre, 3, (0, 255, 255), -1)
                elif centre[0] > 450 and centre[1] < 150:
                    contour_list[3] = centre
                    cv2.circle(frame, centre, 3, (0, 255, 255), -1)

    return frame, contour_list


def draw_setpoints(frame, setpoints):
    cv2.circle(frame, (setpoints[0], setpoints[1]), 5, (0, 0, 0), -1)
    return frame


def ArduinoCheck():
    ports = list(serial.tools.list_ports.comports())
    i = 0
    for p in ports:
        i += 1
        if "Device" in p.description:
            autoCOM = p[0]
            return True, autoCOM
        elif i == (len(ports)):# and "Arduino" not in p.description:
            i = 0
            return False, '0'


def CloseCOMPort(COMport):
    ser = serial.Serial(COMport, 57600)
    ser.close()
    return 0


def ErrorMessage(messageCount):
    messageCount += 1
    print("Arduino not detected")
    print("Please reconnect Arduino")
    return messageCount


def Servos(boardName,servoNumb,servoPin,pin,refAngles):

    for i in range(servoNumb):
        pin.append(boardName.get_pin('d:' + str(servoPin[i]) + ':s'))
        pin[i].write(refAngles[i])
    return True


def AngleCalc(angles, servoNumb,pin,deltas,refAngles,newAngles):
    deltaX, deltaY = angles
    if deltaX != 0 or deltaY != 0:
        deltas[0] = deltaX
        deltas[1] = deltaY
        for i in range(servoNumb):
            # time.sleep(1)
            newAngles[i] = float(refAngles[i]) + float(deltas[i])
            pin[i].write(newAngles[i])
        # testsPin.write(1)
        # time.sleep(0.5)
        # testsPin.write(0)
        print("The new x,y coordinates are: " + str(newAngles[0]) + "," + str(newAngles[1]))


# main PID control function
def PIDcontrol(currentPos, setPoint, settlePoint, axis, gains):
    global I_terms
    global previous_times
    global prev_errors
    global prev_angles
    global prev_positions
    global angles

    sample_time = 0.01

    current_time = time.time()
    # select PID coefficients
    # tuned
    """Kp = 0.176
    Ki = 0.0052
    Kd = 0.8143"""

    # initial incorrect
    """Kp = 0.0078213
    Ki = 0.00005239
    Kd = 0.20372

    #pi
    Kp = 0.044508
    Ki = 0.01
    Kd = 0.58775"""

    Kp = 0.0005208
    Ki = 0.2
    Kd = 0.0272

    """Kp = 0.11208
    Ki = 0.0052
    Kd = 0.08072"""

    # assign different parameters for x and y controllers
    if axis == 'X':
        maxTilt, minTilt = 85,-85
        i = 0
        angleFactor = 11.9
        if settlePoint == 4:
            gains = [2.9, 0.2, 2.72]
    elif axis == 'Y':
        maxTilt, minTilt = 85, -85
        i = 1
        angleFactor = 15

    Kp = gains[0]
    Ki = gains[1]
    Kd = gains[2]

    # calculate values used PID equation
    delta_t = current_time - previous_times[i]
    error = float(setPoint[i]) - float(currentPos[i])
    delta_pv = currentPos[i] - prev_positions[i]

    # check if time elapsed since last PID output is longer than sample time
    if delta_t < sample_time:
        angles[i] = prev_angles[i]
    else:
        # calculate PID terms
        P_term = Kp * error
        #D_term = Kd * (error - prev_errors[i]) / delta_t
        D_term = -(Kd * (delta_pv) / delta_t)
        print(D_term, I_terms)
        I_terms[i] = I_terms[i] + Ki * error * delta_t

        # anti integral windup
        """if I_term >= 5:
            I_term = 5
        if I_term <= -5:
            I_term = -5"""

        # calculate controller output then scale up by the angle factor to move the motors the correct amount
        control_output = round((P_term + I_terms[i] + D_term) * angleFactor, 2)

        # because x orientation of motors and camera is inverse, flip controller x output
        if i == 0:
            angles[i] = - control_output
        else:
            angles[i] = control_output

        # clamp output of controller
        if angles[i] >= maxTilt:
            angles[i] = maxTilt
        if angles[i] <= minTilt:
            angles[i] = minTilt

        # assign current values to previous values for next pid computation
        prev_errors[i] = error
        prev_angles[i] = angles[i]
        previous_times[i] = current_time
        prev_positions[i] = currentPos[i]

        # print output
        if i == 1:
            print("ball moves from " + str(currentPos[0]) + "," + str(currentPos[1]) + " to " + str(setPoint[0]) + "," + str(setPoint[1]) + " by tilting x and y axis " + str(angles[0]) + " and " + str(angles[1]) + " degrees respectively")

    return angles


# function which obtains values from set point list file
def Getsetpoints():
    setPointlist = []
    settlePointList = []

    with open('pathfinder0.txt', 'r') as f:
        data = f.readlines()
    for line in data:
        nums = line.split()
        setX = int(nums[0])
        setY = int(nums[1])
        coords = [setX, setY]
        setPointlist.append(coords)
        isSettlePoint = int(nums[2])
        settlePointList.append(isSettlePoint)
    #print(setPointlist, settlePointList)
    return setPointlist, settlePointList


# function which runs the nearest neighbour algorithm to find the set point closest to the balls current position
def nearestneighbour(currentPos,tree):
    nearestNeighbour = tree.query([currentPos])
    newSetPointCounter = nearestNeighbour[1][0]
    #print(newSetPointCounter)
    return newSetPointCounter


# function which checks if the ball is close enough to the desired setpoint
"""def CheckTolerance(setPoints, settlePoint, ball_coords, i):
    setpointreachedX = False
    setpointreachedY = False
    setpointreached = False
    if settlePoint == 3:
        setpointreachedX = math.isclose(ball_coords[0], setPoints[0], 2.5*i)
        setpointreachedY = math.isclose(ball_coords[1], setPoints[1], 2.5*i)
    else:
        setpointreachedX = math.isclose(ball_coords[0], setPoints[0], i)
        setpointreachedY = math.isclose(ball_coords[1], setPoints[1], i)
    if


    return setpointreached"""
def CheckTolerance(setPoints, ball_coords, settlePoint, i):
    if settlePoint == 3:
        setpointreached = np.allclose(ball_coords, setPoints, atol=2.5*i)
    else:
        setpointreached = np.allclose(ball_coords, setPoints, atol=i)

    return setpointreached

def main():
    loop_start = time.time()
    video_stream = WebcamVideoStream(src=1).start()

    global setPoints
    global previous_times
    global I_terms
    global prev_positions
    global prev_angles
    global prev_errors
    global angles

    settle_time_start = time.time()
    init_timer_start = time.time()

    errorCounter = 0
    messageCounter = 0
    servosConnected = False
    servoNumb = 2  # change to vary number of servos
    servoPin = [10, 9]  # change/add for pin numbers servos connected to on Arduino
    pin = []
    refAngles = [90, 90]
    newAngles = [0, 0]
    deltas = [0, 0]

    setPointList, settlePointList = Getsetpoints()
    tree = spatial.KDTree(setPointList)
    setpointcounter = 0
    setPoints = setPointList[setpointcounter]
    settlePoint = settlePointList[setpointcounter]
    ball_coords = [0,0]
    startController = False

    while True:

        controllerGain = [0.005208, 0.040, 0.00272]

        settle_time_fin = time.time()
        settle_time = settle_time_fin - settle_time_start

        if startController and CheckTolerance(setPoints, ball_coords, settlePoint, 12):
            if settlePoint != 1:
                I_terms = [0, 0]
                setpointcounter += 1
                settle_time_start = time.time()
            else:
                if settle_time >= 1:
                    I_terms = [0, 0]
                    setpointcounter += 1
                    settle_time_start = time.time()
        if startController and settle_time >= 4:
            I_terms = [0, 0]
            setpointcounter = nearestneighbour(ball_coords, tree)
            settle_time_start = time.time()

        setPoints = setPointList[setpointcounter]
        settlePoint = settlePointList[setpointcounter]
        print(setpointcounter)

        if settlePoint == 2:
            print("I HAVE SOLVED THE MAZE")
            break

        frame = video_stream.read()
        frame = cv2.resize(frame, (600, 480))
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # define upper and lower bounds for the colour of the image
        blue_lower = np.array([54, 67, 58])
        blue_upper = np.array([142, 255, 255])
        mask = cv2.inRange(hsv, blue_lower, blue_upper)


        kernel = np.ones((2, 2), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)

        output_frame = draw_circles(mask, frame)

        if None in output_frame[1]:
            pass
        else:
            markers = output_frame[1]
            w = 780
            h = 660
            pts1 = np.float32([[markers[0][0], markers[0][1]], [markers[1][0], markers[1][1]], [markers[2][0], markers[2][1]], [markers[3][0], markers[3][1]]])
            pts2 = np.float32([[0, h], [0, 0], [w, h], [w, 0]])
            mx = cv2.getPerspectiveTransform(pts1, pts2)
            result = cv2.warpPerspective(frame, mx, (w, h))

            blurred = cv2.GaussianBlur(result, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # define upper and lower bounds for colour of image
            red_lower = np.array([170, 86, 47]) #(l_h, l_s, l_v)
            red_upper = np.array([179, 255, 255]) #(u_h, u_s, u_v)
            red_lower_2 = np.array([0, 86, 47]) #(l_h, l_s, l_v)
            red_upper_2 = np.array([10, 255, 255]) #(u_h, u_s, u_v)

            # erode and dilate masked image for more defined picture of object
            mask1 = cv2.inRange(hsv, red_lower, red_upper)
            mask2 = cv2.inRange(hsv, red_lower_2, red_upper_2)
            mask_ball = cv2.bitwise_or(mask1, mask2)

            kernel = np.ones((3, 3), np.uint8)
            mask_ball = cv2.erode(mask_ball, kernel, iterations=2)
            mask_ball = cv2.dilate(mask_ball, kernel, iterations=2)

            output = draw_circle(mask_ball, result)
            #cv2.imshow('Mask of flat w/ ball', mask_ball)

            output_setPoints = draw_setpoints(output[0], setPoints)
            cv2.imshow("Result", output_setPoints)

            ball_centre = output[1]
            init_timer_check = time.time()
            time_since_start = init_timer_check - init_timer_start

            if ball_centre == None:
                print("No ball seen on Maze, please add ball to the Maze")
            elif time_since_start <= 10:
                angles = [0,0]
                ball_coords = [ball_centre[0], ball_centre[1]]
                prev_positions = ball_coords
                settle_time_start = time.time()
            else:

                startController = True
                ball_coords = [ball_centre[0], ball_centre[1]]
                PIDcontrol(ball_coords, setPoints, settlePoint, 'Y', controllerGain)
                PIDcontrol(ball_coords, setPoints, settlePoint, 'X', controllerGain)

            boolie, returnBoard = ArduinoCheck()
            if boolie == True:
                if messageCounter == 0:
                    messageCounter += 1
                    print("Communication successfully started")
                    COMport = ''.join(returnBoard)
                    errorCounter = 0
                    if COMport:
                        CloseCOMPort(COMport)
                        board = Arduino(COMport)
                        print("Arduino is connected to: " + COMport)
                    # ensuring overflow cannot occur in serial buffer
                iter8 = pyfirmata.util.Iterator(board)
                iter8.start()
                if servosConnected == False:
                    servosConnected = Servos(board, servoNumb, servoPin, pin, refAngles)
                AngleCalc(angles, servoNumb, pin, deltas, refAngles, newAngles)

            elif boolie == False and errorCounter < 1:
                errorCounter = ErrorMessage(errorCounter)
                servosConnected = False
                messageCounter = 0

        #cv2.imshow("Mask", mask)
        cv2.imshow("OutputFrame", output_frame[0])

        loop_time = loop_start - time.time()
        #print(loop_time)

        if cv2.waitKey(1) == 27:
            break

    video_stream.stop()
    cv2.destroyAllWindows()
    return

main()
