'''
Importing modules used in operation of code:
'''

import pyfirmata
''' ArduinoMega has non-standard pin arrangement.
Requires unique application of Pyfirmata.'''
from pyfirmata import ArduinoMega

# used for Python GUI mapping servo position
import tkinter as tk

import time

import serial.tools.list_ports

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

before = 0



'''
Variables set to default states here:
'''

''' Used to restrict number of times messages displayed to 
user. Required as function checking for connection between
Arduino and PC ran in continuous 'while True' loop.'''
errorCounter = 0
messageCounter = 0

servosConnected = False
# Change to vary number of servos
servoNumb = 2
# Change/add for pin numbers servos connected to on Arduino
servoPin = [8,9]
''' Can be left empty as servo pin positions appended to
array only once. '''
pin = []

''' Empty arrays must be assigned as zero arrays as the
index placeholders are required to change the values in
a loop. '''
refAngles = [90,90]
newAngles = [0,0]
deltas = [0,0]

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

def CloseCOMPort():
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

def Servos(boardName):
    ''' Function takes Arduino location found in another
    function (ArduinoCheck) as an input argument. Using
    this it attaches a specified number of servos to
    their associated pins. '''
    for i in range(servoNumb):
        ''' 'd.pinNumb.s' indicates a digital pin with
        servo output. '''
        pin.append(boardName.get_pin\
            ('d:'+str(servoPin[i])+':s'))
        pin[i].write(refAngles[i])
    ''' Used to test connection using Arduino's built
    -in LED. '''
    # testingPin = boardName.digital[13]
    # return True, testingPin
    return True

''' testsPin used as input to AngleCalc when testing
as the location is required by the function to make
it flash. '''
# def AngleCalc(testsPin):

def AngleCalc():
    ''' Reads in x,y angles board to be moved to
    with 0 as reference. Adds 90 degrees as required
    by servos and writes new angles to servos. '''
    deltaX, deltaY = angles
    # Requires some change in angle to operate.
    if deltaX != 0 or deltaY != 0:
        deltas[0] = deltaX
        deltas[1] = deltaY
        for i in range(servoNumb):
            newAngles[i] = float(refAngles[i]) + float(deltas[i])
            pin[i].write(newAngles[i])
            
        # testsPin.write(1)
        # time.sleep(0.5)
        # testsPin.write(0)
        
        print("The new x,y coordinates are: " + str(newAngles[0]) + "," + str(newAngles[1]))

def PIDcontrol(currentPos, setPoint, prev_angle, prev_error, previous_time):
    global I_termX, I_termY

    sample_time = 0.1

    current_time = time.time()
    delta_t = current_time - previous_time

    Kp = 0.44508
    Ki = 0.023681
    Kd = 1.8586

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
    setX = round(int(coords[0]) / 300, 2)
    setY = round(int(coords[1]) / 300, 2)
    return setX, setY

                    
while True:

    setPoints = Getsetpoints(l)

    now = time.time()

    if (int(now) - int(before)) >= 2:
        l+= 1
        now = before
    
    if (float(setPoints[0]) - 0.05) <= currentPosition[0] <= (float(setPoints[0]) + 0.05) and (
            float(setPoints[1]) - 0.05) <= currentPosition[1] <= (float(setPoints[1]) + 0.05):
        I_termX = 0
        I_termY = 0
        l += 1
    else:
        angles = (PIDcontrol(currentPosition, setPoints, prev_angles, prev_errors, previous_time))


    angles = (PIDcontrol(currentPosition, setPoints, prev_angles, prev_errors, previous_time))
    print(angles)
    time.sleep(2)


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
                CloseCOMPort()
                ''' board stores pin arrangement of
                Arduino Mega on COM port location. '''
                board = ArduinoMega(COMport)
                print("Arduino is connected to: " + COMport)
                
        # Ensures overflow cannot occur in serial buffer
        iter8 = pyfirmata.util.Iterator(board)
        iter8.start()
        ''' As COM port closed and re-opened upon
        connecting to Arduino, servos must be
        re-attached. '''
        # Checking if servos are connected:
        if servosConnected == False:
            # Connecting servos to Arduino currently attached.
            servosConnected = Servos(board)

            # servosConnected, testPin = Servos(board)

            
        ''' If board connected, runs AngleCalc function
        in loop. '''
        AngleCalc()

        ''' If Arduino not connected, displays error message
        once. '''       
    elif boolie == False and errorCounter < 1:
        errorCounter = ErrorMessage(errorCounter)
        servosConnected = False
        messageCounter = 0
