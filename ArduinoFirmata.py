import pyfirmata
from pyfirmata import ArduinoMega # ArduinoMega has different pin arrangement
import time
import serial.tools.list_ports

servoNumb = 2 # change to vary number of servos
servoPin = [2, 3] # change/add for pin numbers servos connected to on Arduino
pin = []
startAngle = 90

def ArduinoCheck():
    ports = list(serial.tools.list_ports.comports())
    i = 0
    for p in ports:
        i += 1
        if "Arduino" in p.description:
            autoCOM = p[0]
            return True,autoCOM 
        elif i == (len(ports)) and "Arduino" not in p.description:
            i = 0
            return False, ''

def ErrorMessage(messageCount):
    messageCount +=1
    print("Arduino not detected")
    print("Please reconnect Arduino")
    return messageCount

ArduinoCheck()

if __name__ == '__main__':
    if ArduinoCheck == True:
        COMport = ''.join(ArduinoCheck()[1])
        if COMport:
            board = ArduinoMega(COMport)
            # ensuring overflow cannot occur in serial buffer
            iter8 = pyfirmata.util.Iterator(board)
            iter8.start()
            for i in range(servoNumb):
                pin.append(board.get_pin('d:'+str(servoPin[i])+':s'))
                pin[i].write(startAngle)
                currAngleX,currAngleY = startAngle
            #try:
            #    while True:
            #       break
                    #newAngleX,newAngleY = AngleCalc(currAngleX, currAngleY)
                
errorCounter = 0
messageCounter = 0
while True:
    boolie = ArduinoCheck()
    if boolie[0] == True and messageCounter == 0:
        errorCounter = 0
        messageCounter = 1
        print("Communication Successfully started")
    if boolie[0] == False and errorCounter < 1:
        errorCounter = ErrorMessage(errorCounter)
        messageCounter = 0


        
        
    
        
