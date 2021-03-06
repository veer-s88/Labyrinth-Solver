import tkinter
import cv2
import PIL.Image
import PIL.ImageTk
import numpy as np
import imutils
import pyfirmata
from pyfirmata import Arduino# ArduinoMega has different pin arrangement
import time
import serial
import serial.tools.list_ports


class App:
    def __init__(self, window, window_title, video_source=1):
        self.window = window
        self.window.title(window_title)
        self.video_source = video_source

        self.start_time = time.time()

        # open video source (by default this will try to open the computer webcam)
        self.vid = MyVideoCapture(self.video_source)

        # Create a canvas that can fit the above video source size
        self.canvas = tkinter.Canvas(window, width=self.vid.width, height=self.vid.height)
        self.canvas.pack()

        self.maze_solved_lbl = tkinter.Label(window, width=50)
        self.maze_solved_lbl.pack(anchor=tkinter.S, expand=True)
        self.maze_solved_lbl.configure(text='', font=("Calibri", 12))

        self.no_ball_lbl = tkinter.Label(window, width=50)
        self.no_ball_lbl.pack(anchor=tkinter.CENTER, expand=True)
        self.no_ball_lbl.configure(text='No ball')

        self.lbl = tkinter.Label(window, width=50)
        self.lbl.pack(anchor=tkinter.S, expand=True)
        self.lbl.configure(text='nothing')

        self.sp_label = tkinter.Label(window, width=50)
        self.sp_label.pack(anchor=tkinter.S, expand=True)
        self.sp_label.configure(text='Nothing')

        self.angles_lbl = tkinter.Label(window, width=50)
        self.angles_lbl.pack(anchor=tkinter.S, expand=True)
        self.angles_lbl.configure(text='Nothing')

        self.btn_quit = tkinter.Button(window, text='Quit Application', width=50, command=quit)
        self.btn_quit.pack(anchor=tkinter.CENTER, expand=True)

        # After it is called once, the update method will be automatically called every delay milliseconds
        self.delay = 50
        self.update()

        self.window.mainloop()

    def update(self):
        # Get a frame from the video source
        ret, frame = self.vid.get_frame()

        if ret:
            self.photo = PIL.ImageTk.PhotoImage(image=PIL.Image.fromarray(frame))
            self.canvas.create_image(0, 0, image=self.photo, anchor=tkinter.NW)

        if self.vid.maze_solved:
            self.maze_solved_lbl.configure(text='I HAVE SOLVED THE MAZE')

        self.lbl.configure(text='Current Ball Coordinates: ' + str(self.vid.centre))
        self.sp_label.configure(text='Desired Ball Coordinates: ' + str(self.vid.setPoints))
        self.angles_lbl.configure(text='Board angle tilt [x]: ' + str(self.vid.angles[0]) + '    Board angle tilt [y]: '
                                       + str(self.vid.angles[1]))
        if self.vid.no_ball:
            self.no_ball_lbl.configure(text='Ball Seen on Maze: NO')
        else:
            self.no_ball_lbl.configure(text='Ball Seen on Maze: YES')

        self.window.after(self.delay, self.update)


class MyVideoCapture:
    def __init__(self, video_source=0):
        self.centre = None
        # Open the video source
        self.vid = cv2.VideoCapture(video_source)
        if not self.vid.isOpened():
            raise ValueError("Unable to open video source", video_source)

        # Get video source width and height
        self.width = 780
        self.height = 660

        # other variables
        self.settle_time_start = time.time()
        self.init_timer_start = time.time()

        self.errorCounter = 0
        self.messageCounter = 0
        self.servosConnected = False
        self.servoNumb = 2  # change to vary number of servos
        self.servoPin = [10, 9]  # change/add for pin numbers servos connected to on Arduino
        self.pin = []
        self.refAngles = [90, 90]
        self.newAngles = [0, 0]
        self.deltas = [0, 0]

        self.setPoints = [0, 0]
        self.setpointcounter = 0
        self.setpointreached = False

        self.I_terms = [0, 0]
        self.previous_times = [time.time(), time.time()]
        self.prev_errors = [0, 0]
        self.prev_angles = [0, 0]
        self.prev_positions = [0, 0]
        self.angles = [0, 0]

        self.no_ball = False
        self.maze_solved = False

    def draw_circles(self, mask, frame):
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

    def draw_circle(self, mask, frame, setpoints):
        # find contours in the masked image
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        self.centre = None

        if len(contours) > 0:
            # find the largest contour in the mask an assume that is the object we want to detect
            # use this to compute minumium closing circle and centroid
            max_contour = max(contours, key=cv2.contourArea)  # *inside contour area
            ((x, y), radius) = cv2.minEnclosingCircle(max_contour)
            M = cv2.moments(max_contour)
            if M["m00"] > 0:
                self.centre = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius > 10:
                # draw the circle and centroid on the frame
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, self.centre, 2, (0, 255, 255), -1)

            # draw set point circle on frame
            cv2.circle(frame, (setpoints[0], setpoints[1]), 5, (0, 0, 0), -1)
        return frame

    def ArduinoCheck(self):
        ports = list(serial.tools.list_ports.comports())
        i = 0
        for p in ports:
            i += 1
            if "Arduino" in p.description:
                autoCOM = p[0]
                return True, autoCOM
            elif i == (len(ports)):  # and "Arduino" not in p.description:
                i = 0
                return False, '0'

    def CloseCOMPort(self, COMport):
        ser = serial.Serial(COMport, 57600)
        ser.close()
        return 0

    def ErrorMessage(self, messageCount):
        messageCount += 1
        print("Arduino not detected")
        print("Please reconnect Arduino")
        return messageCount

    def Servos(self, boardName, servoNumb, servoPin, pin, refAngles):

        for i in range(servoNumb):
            pin.append(boardName.get_pin('d:' + str(servoPin[i]) + ':s'))
            pin[i].write(refAngles[i])
        return True

    def AngleCalc(self, angles, servoNumb, pin, deltas, refAngles, newAngles):
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
            #print("The new x,y coordinates are: " + str(newAngles[0]) + "," + str(newAngles[1]))

    # main PID control function
    def PIDcontrol(self, currentPos, setPoint, axis, gains):

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
            maxTilt, minTilt = 85, -85
            i = 0
            angleFactor = 11.9
            if self.setPoints[2] == 4:
                gains = [2.9, 0.2, 2.72]
        elif axis == 'Y':
            maxTilt, minTilt = 85, -85
            i = 1
            angleFactor = 15

        Kp = gains[0]
        Ki = gains[1]
        Kd = gains[2]

        # calculate values used PID equation
        delta_t = current_time - self.previous_times[i]
        error = float(setPoint[i]) - float(currentPos[i])
        delta_pv = currentPos[i] - self.prev_positions[i]

        # check if time elapsed since last PID output is longer than sample time
        if delta_t < sample_time:
            self.angles[i] = self.prev_angles[i]
        else:
            # calculate PID terms
            P_term = Kp * error
            # D_term = Kd * (error - prev_errors[i]) / delta_t
            D_term = -(Kd * (delta_pv) / delta_t)
            #print(D_term, self.I_terms)
            self.I_terms[i] = self.I_terms[i] + Ki * error * delta_t

            # anti integral windup
            """if I_term >= 5:
                I_term = 5
            if I_term <= -5:
                I_term = -5"""

            # calculate controller output then scale up by the angle factor to move the motors the correct amount
            control_output = round((P_term + self.I_terms[i] + D_term) * angleFactor, 2)

            # because x orientation of motors and camera is inverse, flip controller x output
            if i == 0:
                self.angles[i] = - control_output
            else:
                self.angles[i] = control_output

            # clamp output of controller
            if self.angles[i] >= maxTilt:
                self.angles[i] = maxTilt
            if self.angles[i] <= minTilt:
                self.angles[i] = minTilt

            # assign current values to previous values for next pid computation
            self.prev_errors[i] = error
            self.prev_angles[i] = self.angles[i]
            self.previous_times[i] = current_time
            self.prev_positions[i] = currentPos[i]

            # print output
            #if i == 1:
                # print("ball moves from " + str(currentPos[0]) + "," + str(currentPos[1]) + " to " + str(
                #    setPoint[0]) + "," + str(setPoint[1]) + " by tilting x and y axis " + str(
                #   self.angles[0]) + " and " + str(self.angles[1]) + " degrees respectively")

        return self.angles

    def Getsetpoints(self, i):
        with open('pathfinder0.txt', 'r') as f:
            data = f.readlines()
        line = data[i]
        coords = line.split()
        setX = int(coords[0])
        setY = int(coords[1])
        isSettlePoint = int(coords[2])

        return setX, setY, isSettlePoint

    def CheckTolerance(self, setPoints, ball_coords, i):
        if ((float(setPoints[0]) - i) <= ball_coords[0] <= (float(setPoints[0]) + i) and (
                float(setPoints[1]) - i) <= ball_coords[1] <= (float(setPoints[1]) + i)):
            self.setpointreached = True
        elif ((float(setPoints[0]) - 2.5 * i) <= ball_coords[0] <= (float(setPoints[0]) + 2.5 * i) and (
                float(setPoints[1]) - 2.5 * i) <= ball_coords[1] <= (float(setPoints[1]) + 2.5 * i) and (
                      setPoints[2] == 3)):
            self.setpointreached = True
        else:
            self.setpointreached = False

        return self.setpointreached

    def get_frame(self):

        if self.vid.isOpened():
            controllerGain = [0.005208, 0.03, 0.00272]
            self.setPoints = self.Getsetpoints(self.setpointcounter)
            #print(self.setpointcounter)

            ret, frame = self.vid.read()
            if ret:
                blurred = cv2.GaussianBlur(frame, (11, 11), 0)
                hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                # define upper and lower bounds for the colour of the image
                blue_lower = np.array([54, 37, 70])
                blue_upper = np.array([157, 255, 255])
                mask = cv2.inRange(hsv, blue_lower, blue_upper)

                kernel = np.ones((2, 2), np.uint8)
                mask = cv2.erode(mask, kernel, iterations=2)
                mask = cv2.dilate(mask, kernel, iterations=2)

                output_frame = self.draw_circles(mask, frame)

                if None in output_frame[1]:
                    return ret, cv2.cvtColor(output_frame[0], cv2.COLOR_BGR2RGB)
                else:
                    markers = output_frame[1]
                    w = 780
                    h = 660
                    pts1 = np.float32(
                        [[markers[0][0], markers[0][1]], [markers[1][0], markers[1][1]], [markers[2][0], markers[2][1]],
                         [markers[3][0], markers[3][1]]])
                    pts2 = np.float32([[0, h], [0, 0], [w, h], [w, 0]])
                    mx = cv2.getPerspectiveTransform(pts1, pts2)
                    result = cv2.warpPerspective(frame, mx, (w, h))

                    blurred = cv2.GaussianBlur(result, (11, 11), 0)
                    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                    # define upper and lower bounds for colour of image
                    red_lower = np.array([170, 95, 58])  # (l_h, l_s, l_v)
                    red_upper = np.array([179, 255, 255])  # (u_h, u_s, u_v)
                    red_lower_2 = np.array([0, 95, 58])  # (l_h, l_s, l_v)
                    red_upper_2 = np.array([10, 255, 255])  # (u_h, u_s, u_v)

                    # erode and dilate masked image for more defined picture of object
                    mask1 = cv2.inRange(hsv, red_lower, red_upper)
                    mask2 = cv2.inRange(hsv, red_lower_2, red_upper_2)
                    mask_ball = cv2.bitwise_or(mask1, mask2)

                    kernel = np.ones((3, 3), np.uint8)
                    mask_ball = cv2.erode(mask_ball, kernel, iterations=2)
                    mask_ball = cv2.dilate(mask_ball, kernel, iterations=2)

                    output = self.draw_circle(mask_ball, result, self.setPoints)

                    ball_centre = self.centre
                    init_timer_check = time.time()
                    time_since_start = init_timer_check - self.init_timer_start

                    if ball_centre == None:
                        self.no_ball = True
                        print("No ball seen on Maze, please add ball to the Maze")
                    elif time_since_start <= 10:
                        self.no_ball = False
                        self.angles = [0, 0]
                        ball_coords = [ball_centre[0], ball_centre[1]]
                        self.prev_positions = ball_coords
                    else:
                        self.no_ball = False
                        ball_coords = [ball_centre[0], ball_centre[1]]
                        """ball_coords = (round(ball_centre[0]/30,2), round(ball_centre[1]/30,2))
                        setPoints2 = [0, 0, 0]
                        setPoints2[0] = round(int(setPoints[0]) / 30, 1)
                        setPoints2[1] = round(int(setPoints[1]) / 30, 1)
                        setPoints2[2] = setPoints[2]"""
                        if self.setPoints[2] == 2:
                            self.maze_solved = True
                            print("I HAVE SOLVED THE MAZE")
                            #break
                        elif self.CheckTolerance(self.setPoints, ball_coords, 12):
                            if self.setPoints[2] == 0 or 3 or 4:
                                self.I_terms = [0, 0]
                                self.setpointcounter += 1
                                self.settle_time_start = time.time()
                            elif self.setPoints[2] == 1:
                                settle_time_fin = time.time()
                                delta_time = settle_time_fin - self.settle_time_start
                                if delta_time >= 1:
                                    self.I_terms = [0, 0]
                                    self.setpointcounter += 1
                                    self.settle_time_start = time.time()
                        else:
                            self.PIDcontrol(ball_coords, self.setPoints, 'Y', controllerGain)
                            self.PIDcontrol(ball_coords, self.setPoints, 'X', controllerGain)

                    boolie, returnBoard = self.ArduinoCheck()
                    if boolie == True:
                        if self.messageCounter == 0:
                            self.messageCounter += 1
                            print("Communication successfully started")
                            COMport = ''.join(returnBoard)
                            self.errorCounter = 0
                            if COMport:
                                self.CloseCOMPort(COMport)
                                self.board = Arduino(COMport)
                                print("Arduino is connected to: " + COMport)
                            # ensuring overflow cannot occur in serial buffer
                        iter8 = pyfirmata.util.Iterator(self.board)
                        iter8.start()
                        if self.servosConnected == False:
                            self.servosConnected = self.Servos(self.board, self.servoNumb, self.servoPin, self.pin, self.refAngles)
                        self.AngleCalc(self.angles, self.servoNumb, self.pin, self.deltas, self.refAngles, self.newAngles)

                    elif boolie == False and self.errorCounter < 1:
                        self.errorCounter = self.ErrorMessage(self.errorCounter)
                        self.servosConnected = False
                        self.messageCounter = 0

                    # Return a boolean success flag and the current frame converted to BGR
                    return ret, cv2.cvtColor(output, cv2.COLOR_BGR2RGB)


    # Release the video source when the object is destroyed
    def __del__(self):
        if self.vid.isOpened():
            self.vid.release()


# Create a window and pass it to the Application object
App(tkinter.Tk(), "Tkinter and OpenCV")
