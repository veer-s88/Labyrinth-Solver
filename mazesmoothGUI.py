# used for Python GUI mapping servo position
import threading
import tkinter as tk
from tkinter import Tk, Canvas, Frame, BOTH
import math

from threading import Thread
import cv2
import numpy as np
import imutils
import pyfirmata
from pyfirmata import Arduino# ArduinoMega has different pin arrangement
import time
import serial
import serial.tools.list_ports


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


class _App(threading.Thread):

    def __init__(self):
        self.servoNumb = 2
        xPos = [100, 300]
        yPos = 100
        _App.XPos = xPos
        _App.YPos = yPos
        radius = 50
        _App.Radius = radius
        self.lines = [0, 0]
        threading.Thread.__init__(self)
        self.start()

    def callback(self):
        self.root.quit()

    def _create_circle(self, x, y, r, **kwargs):
        ''' Function to create a circle object in
        the Tk window with parameters x,y,r. '''
        return self.create_oval(x-r, y-r, x+r, y+r, **kwargs)
    tk.Canvas.create_circle = _create_circle

    def run(self):
        self.root = tk.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.callback)

        _App.canvas = tk.Canvas(self.root, width=400, height=200,\
            borderwidth=0, highlightthickness=0, bg="black")
        _App.canvas.pack()

        self.root.wm_title("Real Time Servo Positions")

        print("Loading servo GUI")

        xPosArr = np.array(_App.XPos)
        yPos = np.array(_App.YPos)
        radius = np.array(_App.Radius)

        for i in range(self.servoNumb):
            self.canvas.create_circle(xPosArr[i], yPos,\
                radius, fill="white", outline="#DDD", width=4)
            self.lines[i] = self.canvas.create_line(xPosArr[i],\
                        (yPos - radius), xPosArr[i], yPos,\
                                        width=5, fill="red")

        _App.line1 = self.lines[0]
        _App.line2 = self.lines[1]

        self.initLabelText = '90 degrees'

        _App.label1Text = ('Servo 1: x-axis\n' + self.initLabelText)
        _App.label2Text = ('Servo 2: y-axis\n' + self.initLabelText)

        _App.label1 = tk.Label(self.root, text = self.label1Text)
        _App.label1.pack(padx=15, pady=5, side=tk.LEFT)
        _App.label1.config(width=20)
        _App.label1.config(font=("Courier", 15))
        _App.label2 = tk.Label(self.root, text = self.label2Text)
        _App.label2.pack(padx=25, pady=5, side=tk.LEFT)
        _App.label2.config(width=20)
        _App.label2.config(font=("Courier", 15))

        self.root.mainloop()

app = _App()


class _ConnectServos(object):

    def __init__(self, board):
        self.board = board
        self.refAngle = 90
        self.pin = []
        self.servoPin = [2, 3]
        self.servoNumb = _App.__init__
        servoNumb = np.array(self.servoNumb)

    def Servos(self, board):
        ''' Function takes Arduino location found in
        another function (ArduinoCheck) as an input
        argument. Using this, it attaches a specified
        number of servos to their associated pins. '''
        for i in range(self.servoNumb):
            ''' 'd.pinNumb.s' indicates a digital pin
            with servo output. '''
            self.pin.append(board.get_pin('d:' +\
                        str(self.servoPin[i]) + ':s'))
            self.pin[i].write(self.refAngle)
        _ConnectServos.pin = self.pin
        _ConnectServos.servoNumb = self.servoNumb
        return True, self.refAngle


# root.mainloop() keeps only runs event handlers.

class _Main(_App, _ConnectServos):

    def __init__(self):
        self.anglesRad = [0, 0]
        self.line1 = _App.line1
        self.line2 = _App.line2
        self.canvas = _App.canvas
        self.root = _App.run
        self.xPos = _App.XPos
        self.yPos = _App.YPos
        self.radius = _App.Radius
        self.label1 = _App.label1
        self.label2 = _App.label2
        self.pin = _ConnectServos.pin
        self.servoNumb = _ConnectServos.pin


    def NewAnglesGUI(self, newAngles):

        xPosArr = np.array(self.xPos)
        yPos = np.array(self.yPos)
        radius = np.array(self.radius)

        for i in range(self.servoNumb):
            self.anglesRad[i] = newAngles[i] * (math.pi/180)
        # TypeError: 'int' object does not support item assignment
        endX1 = xPosArr[0] + radius * math.cos(self.anglesRad[0])
        endY1 = yPos - radius * math.sin(self.anglesRad[0])
        endX2 = xPosArr[1] + radius * math.cos(self.anglesRad[1])
        endY2 = yPos - radius * math.sin(self.anglesRad[1])

        # attribute error: type object has no attribute
        # not creating an instance, but instead referencing
        # the class itself

        self.canvas.coords(self.line1, xPosArr[0],\
                           yPos, endX1, endY1)
        self.canvas.coords(self.line2, xPosArr[1],\
                           yPos, endX2, endY2)

        self.servo1AngleStr = (str(newAngles[0]) + ' degrees')
        self.servo2AngleStr = (str(newAngles[1]) + ' degrees')

        self.label1['text'] = ('Servo 1: x-axis\n' + self.servo1AngleStr)
        self.label2['text'] = ('Servo 2: y-axis\n' + self.servo2AngleStr)

        # self.root.after(100, self.NewAnglesGUI)

    def AngleCalc(self, deltaX, deltaY, refAngle):
        ''' Reads in x,y angles board to be moved to
        with 0 as reference. Adds 90 degrees as required
        by servos and writes new angles to servos. '''
        deltas = [0, 0]
        newAngles = [0, 0]
        # Requires some change in angle to operate.
        if deltaX != 0 or deltaY != 0:
            deltas[0] = deltaX
            deltas[1] = deltaY
            for i in range(self.servoNumb):
                newAngles[i] = float(refAngle) +\
                               float(deltas[i])
                self.pin[i].write(newAngles[i])
            # testsPin.write(1)
            # time.sleep(0.5)
            # testsPin.write(0)

            print("The new x,y coordinates are: " +\
                  str(newAngles[0]) + ","\
                  + str(newAngles[1]))

            self.NewAnglesGUI(newAngles)



'''
Functions called in code defined below:
'''

def ArduinoCheck():
    ''' Reads in and stores as list the COM port
    connections currently available on PC. Iterates
    through list to check if "Arduino" appears. '''
    ports = list(serial.tools.list_ports.comports())
    i = 0
    for p in ports:
        i += 1
        if "Arduino" in p.description:
            autoCOM = p[0]
            # Returns Boolean and string
            return True, autoCOM
            ''' Can only assume no connection if the array
            has been checked until final element. '''
        elif i == (len(ports)) and "Arduino" not in \
            p.description or p == None:
            i = 0
            ''' Returns Boolean and empty string so that
            format of returned variables alligns, avoiding
            error. '''
            return False, '0'

def CloseCOMPort(COMport):
    ''' Used to close COM port opened with Arduino so
    if disconnected and reconnected, it resets,
    avoiding PinAlreadyTakenError otherwise triggered. '''
    ser = serial.Serial(COMport,57600)
    ser.close()
    # Returns none to terminate function.
    return 0

def ErrorMessage(messageCount):
    ''' Prints error message if Arduino not detected.
    Message count returned so it is only displayed
    once and prevents message being output infinitely
    to user. '''
    messageCount +=1
    print("Arduino not detected")
    print("Please reconnect Arduino")
    return messageCount


''' testsPin used as input to AngleCalc when testing
as the location is required by the function to make
it flash. '''

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

#main PID control function
def PIDcontrol(currentPos, setPoint, axis,gains):
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
        if setPoints[2] == 4:
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

#function which obtains values from setpoint list file
def Getsetpoints(i):
    with open('pathfinder0.txt', 'r') as f:
        data = f.readlines()
    line = data[i]
    coords = line.split()
    setX = int(coords[0])
    setY = int(coords[1])
    isSettlePoint = int(coords[2])

    return setX, setY, isSettlePoint


def CheckTolerance(setPoints,ball_coords,i):
    setpointreached = False
    if ((float(setPoints[0]) - i) <= ball_coords[0] <= (float(setPoints[0]) + i) and (
            float(setPoints[1]) - i) <= ball_coords[1] <= (float(setPoints[1]) + i)):
        setpointreached = True
    elif ((float(setPoints[0]) - 2.5*i) <= ball_coords[0] <= (float(setPoints[0]) + 2.5*i) and (
            float(setPoints[1]) - 2.5*i) <= ball_coords[1] <= (float(setPoints[1]) + 2.5*i) and (setPoints[2] == 3)):
        setpointreached = True
    else:
        setpointreached = False

    return setpointreached


def main():
    video_stream = WebcamVideoStream(src=1).start()

    global setPoints
    setpointcounter = 0

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



    while True:
        controllerGain = [0.005208, 0.03, 0.00272]
        setPoints = Getsetpoints(setpointcounter)
        print(setpointcounter)
        frame = video_stream.read()
        frame = cv2.resize(frame, (600, 480))
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # define upper and lower bounds for the colour of the image
        blue_lower = np.array([54, 37, 70])
        blue_upper = np.array([157, 255, 255])
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
            red_lower = np.array([170, 95, 58]) #(l_h, l_s, l_v)
            red_upper = np.array([179, 255, 255]) #(u_h, u_s, u_v)
            red_lower_2 = np.array([0, 95, 58]) #(l_h, l_s, l_v)
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

            output_setpoints = draw_setpoints(output[0], setPoints)
            cv2.imshow("Result", output_setpoints)

            ball_centre = output[1]
            init_timer_check = time.time()
            time_since_start = init_timer_check - init_timer_start

            if ball_centre == None:
                print("No ball seen on Maze, please add ball to the Maze")
            elif time_since_start <= 10:
                angles = [0,0]
                ball_coords = [ball_centre[0], ball_centre[1]]
                prev_positions = ball_coords
            else:
                ball_coords = [ball_centre[0], ball_centre[1]]
                """ball_coords = (round(ball_centre[0]/30,2), round(ball_centre[1]/30,2))
                setPoints2 = [0, 0, 0]
                setPoints2[0] = round(int(setPoints[0]) / 30, 1)
                setPoints2[1] = round(int(setPoints[1]) / 30, 1)
                setPoints2[2] = setPoints[2]"""
                if setPoints[2] == 2:
                    print("I HAVE SOLVED THE MAZE")
                    break
                elif CheckTolerance(setPoints, ball_coords, 12):
                    if setPoints[2] == 0 or 3 or 4:
                        I_terms = [0, 0]
                        setpointcounter += 1
                        settle_time_start = time.time()
                    elif setPoints[2] == 1:
                        settle_time_fin = time.time()
                        delta_time = settle_time_fin - settle_time_start
                        if delta_time >= 1:
                            I_terms = [0, 0]
                            setpointcounter += 1
                            settle_time_start = time.time()
                else:
                    PIDcontrol(ball_coords, setPoints, 'Y', controllerGain)
                    PIDcontrol(ball_coords, setPoints, 'X', controllerGain)

            '''
            As located inside main 'while True' loop, program
            constantly monitors if Arduino
            connected/disconnected. '''

            # Stores returned values from ArduinoCheck()
            boolie, returnBoard = ArduinoCheck()

            # If Arduino is connected:
            if boolie == True:

                # Checks message not yet output to user.
                if messageCounter == 0:
                    # Ensures message output only single time.
                    messageCounter += 1
                    print("Communication successfully started")
                    # returnBoard concatenated to string.
                    COMport = ''.join(returnBoard)
                    errorCounter = 0

                    # if True (COMport is non-zero value)
                    if COMport:
                        ''' Close COM port in case is already
                        open in program. '''
                        CloseCOMPort(COMport)
                        ''' board stores pin arrangement of
                        Arduino Mega on COM port location. '''
                        #board = ArduinoMega(COMport)
                        boardN = Arduino(COMport)
                        print("Arduino is connected to: " + COMport)

                # Ensures overflow cannot occur in serial buffer
                iter8 = pyfirmata.util.Iterator(boardN)
                iter8.start()
                ''' As COM port closed and re-opened upon
                connecting to Arduino, servos must be
                re-attached. '''
                # Checking if servos are connected:
                if servosConnected == False:
                    # instantiating class instance
                    _c = _ConnectServos(boardN)
                    # Connecting servos to Arduino currently attached.
                    servosConnected, refAngle = _c.Servos(boardN)

                ''' If board connected, runs AngleCalc function
                in loop. '''
                deltaX, deltaY = angles
                _m = _Main()
                _m.AngleCalc(deltaX, deltaY, refAngle)


                ''' If Arduino not connected, displays error message

                once. '''
            elif boolie == False and errorCounter < 1:
                errorCounter = ErrorMessage(errorCounter)
                servosConnected = False
                messageCounter = 0

        #cv2.imshow("Mask", mask)
        cv2.imshow("OutputFrame", output_frame[0])


        if cv2.waitKey(1) == 27:
            break

    video_stream.stop()
    cv2.destroyAllWindows()
    return

main()
