from threading import Thread
import cv2
import numpy as np
import imutils
import pyfirmata
# from pyfirmata import ArduinoMega # ArduinoMega has different pin arrangement
from pyfirmata import Arduino
import time
import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import Tk, Canvas, Frame, BOTH
import math

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
    cv2.circle(frame, (setpoints[0], setpoints[1]), 3, (0, 255, 0), -1)
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


def AngleCalc(angles,servoNumb,pin,deltas,refAngles,newAngles,canvas, line1, line2):
    ''' Reads in x,y angles board to be moved to
    with 0 as reference. Adds 90 degrees as required
    by servos and writes new angles to servos. '''
    deltaX, deltaY = angles
    # Requires some change in angle to operate.
    if deltaX != 0 or deltaY != 0:
        deltas[0] = deltaX
        deltas[1] = deltaY
        for i in range(servoNumb):
            newAngles[i] = float(refAngles[i])\
                + float(deltas[i])
            pin[i].write(newAngles[i])  
        # testsPin.write(1)
        # time.sleep(0.5)
        # testsPin.write(0)
        NewAnglesGUI(canvas, line1, line2)

def _create_circle(self, x, y, r, **kwargs):
    ''' Function to create a circle object in
    the Tk window with parameters x,y,r. '''
    return self.create_oval(x-r, y-r, x+r, y+r, **kwargs)
tk.Canvas.create_circle = _create_circle

# ISSUE WITH GUI NOT OPENING - MAINLOOP BLOCKS CODE AS WAITS FOR CHANGE

def DrawServoGUI(servoNumb):
    root = tk.Tk()
    canvas = tk.Canvas(root, width=600, height=600,\
        borderwidth=0, highlightthickness=0, bg="black")
    # canvas.grid()
    canvas.pack()
    for i in range(servoNumb):
        canvas.create_circle(xPos[i], yPos, radius,\
               fill="white", outline="#DDD", width=4)
    line1 = canvas.create_line(xPos[0], yPos, endX[0], endY, width=5, fill="red")
    line2 = canvas.create_line(xPos[1], yPos, endX[1], endY, width=5, fill="red")
    root.wm_title("Real Time Servo Positions")
    NewAnglesGUI(canvas, line1, line2)
    # root.mainloop()
    return canvas,line1,line2
    

def NewAnglesGUI(canvas, line1, line2):
    for i in range(servoNumb):
        anglesRad[i] = newAngles[i] * (math.pi/180)
    # TypeError: 'int' object does not support item assignment
    endX1 = xPos[0] + radius * math.cos(anglesRad[0])
    endY1 = yPos + radius * math.sin(anglesRad[0])
    endX2 = xPos[1] + radius * math.cos(anglesRad[1])
    endY2 = yPos + radius * math.sin(anglesRad[1])
    
    canvas.coords(line1, xPos[0], yPos, endX1, endY1)
    canvas.coords(line2, xPos[1], yPos, endX2, endY2)
    

def PIDcontrol(currentPos, setPoint, prev_angle, prev_error, previous_time):
    global I_termX, I_termY

    sample_time = 0.1

    current_time = time.time()
    delta_t = current_time - previous_time

    #Kp = 0.176
    #Ki = 0.0052
    #Kd = 0.8143

    Kp = 0.0078213
    Ki = 0.00005239
    Kd = 0.20372

    maxTilt = 90
    minTilt = -90

    errorX = float(setPoint[0]) - float(currentPos[0])
    errorY = float(setPoint[1]) - float(currentPos[1])

    if delta_t < sample_time:
        angleX = prev_angle[0]
        angleY = prev_angle[1]
    else:

        P_termX = Kp * errorX
        D_termX = Kd * (errorX - prev_error[0]) / delta_t
        I_termX = I_termX + Ki * errorX * delta_t
        """if I_termX >= 1:
            I_termX = 1
        if I_termX <= -1:
            I_termX = -1"""
        angleX = -round(P_termX + I_termX + D_termX, 2) * 20
        prev_angle[0] = angleX
        prev_error[0] = errorX
        angleX = round(angleX, 2)
        if angleX > maxTilt:
            angleX = maxTilt
        if angleX < minTilt:
            angleX = minTilt

        P_termY = Kp * errorY
        D_termY = Kd * (errorY - prev_error[1]) / delta_t
        I_termY = I_termY + Ki * errorY * delta_t
        """if I_termY >= 1:
            I_termY = 1
        if I_termY <= -1:
            I_termY = -1"""
        angleY = round(P_termY + I_termY + D_termY, 2) * 30
        prev_angle[1] = angleY
        prev_error[1] = errorY
        angleY = round(angleY, 2)
        if angleY > maxTilt:
            angleY = maxTilt
        if angleY < minTilt:
            angleY = minTilt

    previous_time = current_time
    print("ball moves from " + str(currentPos[0]) + "," + str(currentPos[1]) + " to " + str(setPoints[0]) + "," + str(setPoints[1]) + " by tilting x and y axis " + str(angleX) + " and " + str(angleY) + " degrees respectively")
    angles = ('<' + '%+.2f' % float(angleX)) + ('%+.2f' % float(angleY) + '>')

    return angleX, angleY


def Getsetpoints(i):
    with open('pathfinder0.txt', 'r') as f:
        data = f.readlines()
    line = data[i]
    coords = line.split()
    setX = int(coords[0])
    setY = int(coords[1])
    isSettlePoint = int(coords[2])
    return setX, setY, isSettlePoint


def main():
    video_stream = WebcamVideoStream(src=1).start()

    setPointX = 0
    setPointY = 0
    global setPoints
    setPoints = [setPointX, setPointY]
    previous_time = time.time()
    prev_angleX = 0
    prev_angleY = 0
    prev_angles = [prev_angleX, prev_angleY]
    prev_errorX = 0
    prev_errorY = 0
    prev_errors = [prev_errorX, prev_errorY]
    global I_termX
    global I_termY
    I_termX = 0
    I_termY = 0
    setpointcounter = 0
    settle_time_start = time.time()

    errorCounter = 0
    messageCounter = 0
    servosConnected = False
    servoNumb = 2  # change to vary number of servos
    servoPin = [9, 10]  # change/add for pin numbers servos connected to on Arduino
    pin = []
    refAngles = [90, 90]
    newAngles = [0, 0]
    deltas = [0,0]
    angles = [0,0]


    while True:
        setPoints = Getsetpoints(setpointcounter)

        frame = video_stream.read()
        frame = cv2.resize(frame, (600, 480))
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # define upper and lower bounds for the colour of the image
        blue_lower = np.array([54, 44, 70])
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
            red_lower = np.array([170, 120, 23])
            red_upper = np.array([179, 255, 255])
            red_lower_2 = np.array([0, 120, 23])
            red_upper_2 = np.array([10, 255, 255])

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
            if ball_centre == None:
                print("No ball seen on Maze, please add ball to the Maze")

            else:
                ball_coords = (ball_centre[0], ball_centre[1])
                if (float(setPoints[0]) - 10) <= ball_coords[0] <= (float(setPoints[0]) + 10) and (
                        float(setPoints[1]) - 10) <= ball_coords[1] <= (float(setPoints[1]) + 10):
                    if setPoints[2] == 0:
                        I_termX = 0
                        I_termY = 0
                        setpointcounter += 1
                        settle_time_start = time.time()
                    elif setPoints[2] == 1:
                        settle_time_fin = time.time()
                        if settle_time_fin - settle_time_start >= 1:
                            I_termX = 0
                            I_termY = 0
                            setpointcounter += 1
                            settle_time_start = time.time()

                else:
                    angles = (PIDcontrol(ball_coords, setPoints, prev_angles, prev_errors, previous_time))
                    print(I_termX)
            boolie, returnBoard = ArduinoCheck()
            if boolie == True:
                if messageCounter == 0:
                    messageCounter += 1
                    print("Communication successfully started")
                    COMport = ''.join(returnBoard)
                    errorCounter = 0
                    if COMport:
                        CloseCOMPort(COMport)
                        #board = ArduinoMega(COMport)
                        board = Arduino(COMport)
                        print("Arduino is connected to: " + COMport)
                    # ensuring overflow cannot occur in serial buffer
                iter8 = pyfirmata.util.Iterator(board)
                iter8.start()
                if servosConnected == False:
                    servosConnected = Servos(board, servoNumb, servoPin, pin, refAngles)
                canvas, line1, line2 = DrawServoGUI(servoNumb)
                AngleCalc(angles, servoNumb, pin, deltas, refAngles, newAngles, canvas, line1, line2)

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
