import numpy as np
import serial
import serial.tools.list_ports
import csv
    import time


currentX = 0
currentY = 0
currentPosition = [currentX,currentY]

setPointX = 0
setPointY = 0
setPoints = [setPointX,setPointY]

previous_time = time.time()

prev_angleX = 0
prev_angleY = 0
prev_angles = [prev_angleX,prev_angleY]

prev_errorX = 0
prev_errorY = 0
prev_errors = [prev_errorX,prev_errorY]

I_termX = 0
I_termY = 0

l = 1

def PIDcontrol(currentPos, setPoint, prev_angle, prev_error, previous_time):
    global I_termX, I_termY

    sample_time = 0.001

    current_time = time.time()
    delta_t = current_time - previous_time

    Kp = 0.176
    Ki = 0.0052
    Kd = 0.8143

    maxTilt = 5
    minTilt = -5

    errorX = int(setPoint[0]) - int(currentPos[0])
    errorY = int(setPoint[1]) - int(currentPos[1])

    if delta_t < sample_time:
        angleX = prev_angle[0]
        angleY = prev_angle[1]
    else:

        P_termX = Kp * errorX
        D_termX = Kd * (errorX - prev_error[0]) / delta_t
        I_termX = I_termX + Ki * errorX * delta_t
        angleX = round(P_termX + I_termX + D_termX, 2)
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
        angleY = round(P_termY + I_termY + D_termY, 2)
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

    return angles


def Getsetpoints(i):
    with open('pathfinder0.txt','r') as f:
        data = f.readlines()
    line = data[i]
    coords = line.split()
    setX = coords[0]
    setY = coords[1]
    return setX, setY


#arduino = serial.Serial('COM7', 115200, timeout=.1)

while True:

    if (float(setPoints[0]) - 0.05) <= currentPosition[0] <= (float(setPoints[0]) + 0.05) and (float(setPoints[1]) - 0.05) <= currentPosition[1] <= (float(setPoints[1]) + 0.05):
        I_termX = 0
        I_termY = 0
        l += 1

    setPoints = Getsetpoints(l)

    angles = (PIDcontrol(currentPosition, setPoints, prev_angles, prev_errors, previous_time))

    #arduino.write(angles)
    print(angles)
    print(I_termY)



