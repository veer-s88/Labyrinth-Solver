import pyfirmata
from pyfirmata import ArduinoMega # ArduinoMega has different pin arrangement
import time
import serial
import serial.tools.list_ports

currentX = None
currentY = None
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

errorCounter = 0
messageCounter = 0
servosConnected = False
servoNumb = 2 # change to vary number of servos
servoPin = [2, 3] # change/add for pin numbers servos connected to on Arduino
pin = []
refAngles = [90,90]
newAngles = [0,0]
deltas = []

def ArduinoCheck():
    ports = list(serial.tools.list_ports.comports())
    i = 0
    for p in ports:
        if "Arduino" in p.description:
            autoCOM = p[0]
            return True, autoCOM 
        elif i == (len(ports)) and "Arduino" not in p.description:
            i = 0
            return False, '0'
        i += 1

def CloseCOMPort():
    ser = serial.Serial(COMport,57600)
    ser.close()
    return 0

def ErrorMessage(messageCount):
    messageCount +=1
    print("Arduino not detected")
    print("Please reconnect Arduino")
    return messageCount

def Servos(boardName):
    for i in range(servoNumb):
        pin.append(board.get_pin('d:'+str(servoPin[i])+':s'))
        pin[i].write(refAngles[i])
    return True

def AngleCalc():
    deltaX, deltaY = angles
    deltas.append(deltaX)
    deltas.append(deltaY)
    print("DELTA Y: " + str(deltaY))
    for i in range(servoNumb):
        newAngles[i] = refAngles[i] + deltas[i]
        pin[i].write(newAngles[i])
    print("The new angle is: " + str(newAngles[0]) + "," + str(newAngles[1]))

def PIDcontrol(currentPos, setPoint, prev_angle, prev_error, previous_time):
    global I_termX, I_termY

    sample_time = 0.1

    current_time = time.time()
    delta_t = current_time - previous_time

    Kp = 0.176
    Ki = 0.0052
    Kd = 0.8143

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
        if I_termX >= 0.03:
            I_termX = 0.03
        angleX = round(P_termX + I_termX + D_termX, 2)*20
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
        if I_termY >= 0.03:
            I_termY = 0.03
        angleY = round(P_termY + I_termY + D_termY, 2)*30
        prev_angle[1] = angleY
        prev_error[1] = errorY
        angleY = round(angleY, 2)
        if angleY > maxTilt:
            angleY = maxTilt
        if angleY < minTilt:
            angleY = minTilt

    previous_time = current_time
    #print("ball moves from " + str(currentPos[0]) + "," + str(currentPos[1]) + " to " + str(setPoints[0]) + "," + str(setPoints[1]) + " by tilting x and y axis " + str(angleX) + " and " + str(angleY) + " degrees respectively")
    angles = ('<' + '%+.2f' % float(angleX)) + ('%+.2f' % float(angleY) + '>')

    return angleX, angleY


def Getsetpoints(i):
    with open('pathfinder0.txt','r') as f:
        data = f.readlines()
    line = data[i]
    coords = line.split()
    setX = round(int(coords[0]) / 3000, 2)
    setY = round(int(coords[1]) / 3000, 2)
    return setX, setY

                    
while True:
    setPoints = Getsetpoints(l)
    if (float(setPoints[0]) - 0.05) <= currentPosition[0] <= (float(setPoints[0]) + 0.05) and (
            float(setPoints[1]) - 0.05) <= currentPosition[1] <= (float(setPoints[1]) + 0.05):
        I_termX = 0
        I_termY = 0
        l += 1
    else:
        angles = (PIDcontrol(currentPosition, setPoints, prev_angles, prev_errors, previous_time))

    boolie, returnBoard = ArduinoCheck()
    if boolie == True:
        if messageCounter == 0:
            messageCounter += 1
            print("Communication successfully started")  
            COMport = ''.join(returnBoard)
            errorCounter = 0
            if COMport:
                CloseCOMPort()
                board = ArduinoMega(COMport)
                print("Arduino is connected to: " + COMport)
            # ensuring overflow cannot occur in serial buffer
        iter8 = pyfirmata.util.Iterator(board)
        iter8.start()
        if servosConnected == False:
            servosConnected = Servos(board)
        AngleCalc()
            
    elif boolie == False and errorCounter < 1:
        errorCounter = ErrorMessage(errorCounter)
        servosConnected = False
        messageCounter = 0