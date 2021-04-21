import numpy as np
import serial
import serial.tools.list_ports
import csv
import time


currentX = 0
currentY = 0

setPointX = 0
setPointY = 0

l = 0

previous_timeX = time.time()
previous_timeY = time.time()

prev_angleX = 0
prev_angleY = 0

prev_errorX = 0
prev_errorY = 0

I_termX = 0
I_termY = 0
XorY = 0


def PIDcontrol(currentPos, setPoint, prev_angle, prev_error, previous_time):
    global prev_errorX, prev_errorY
    global prev_angleX, prev_angleY
    global previous_timeX, previous_timeY
    global I_termX, I_termY
    global XorY

    sample_time = 0.001

    current_time = time.time()
    delta_t = current_time - previous_time

    Kp = 0.176
    Ki = 0.0052
    Kd = 0.8143

    maxTilt = 5
    minTilt = -5
    error = setPoint-currentPos

    if delta_t < sample_time:
        angle = prev_angle
    else:

        P_term = Kp * error
        D_term = Kd * (error - prev_error) / delta_t

        if XorY == 0:
            I_termX = I_termX + Ki * error * delta_t
            angle = round(P_term + I_termX + D_term, 2)
            previous_timeX = current_time
            prev_angleX = angle
            prev_errorX = error
            angle = round(angle, 2)
            if angle == 0.0:
                angle = 0.00



        elif XorY == 1:
            I_termY = I_termY + Ki * error * delta_t
            angle = round(P_term + I_termY + D_term, 2)
            previous_timeY = current_time
            prev_angleY = angle
            prev_errorY = error
            angle = round(angle, 2)
            if angle == 0.0:
                angle = 0.00


    if angle > maxTilt:
        angle = maxTilt
    if angle < minTilt:
        angle = minTilt

    XorY = 1 - XorY

    return angle


def Getsetpoints(i):
    with open('pathfinder0.txt','r') as f:
        data = f.readlines()
    line = data[i]
    coords = line.split()
    setX = coords[0]
    setY = coords[1]
    return setX, setY


##arduino = serial.Serial('COM5', 115200, timeout=.1)

while True:

    if (setPointX - 0.05) <= currentX <= (setPointX + 0.05) and (setPointY - 0.05) <= currentY <= (setPointY + 0.05):
        l += 1
        I_termX = 0
        I_termY = 0

    setPoints = Getsetpoints(l)
    setPointX = int(setPoints[0])
    setPointY = int(setPoints[1])

    angleX = (PIDcontrol(currentX, setPointX, prev_angleX, prev_errorX, previous_timeX))
    angleY = (PIDcontrol(currentY, setPointY, prev_angleY, prev_errorY, previous_timeY))

    angl = ('%+.2f' % float(angleX))+('%+.2f' % float(angleY))
    angles = angl.encode()


   ## arduino.write(angles)
    print(angles)
    print("ball moves from " + str(currentX) + "," + str(currentY) + " to " + str(setPointX) + "," + str(setPointY) + " by tilting x and y axis " + str(angleX) + " and " + str(angleY) + " degrees respectively")

    prevPosX = currentX
    prevPosY = currentY

#add plus to the front
#0.0 to 0.00

