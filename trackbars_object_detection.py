import argparse
import time
import cv2
import numpy as np
import imutils
import requests
import pygame

# define colours as global variables
white = (255, 255, 255)
red = (255, 0, 0)


def run_pg():
    p_width = 600
    p_height = 450
    window = pygame.display.set_mode((p_width, p_height))
    pygame.display.set_caption("Virtual Image")
    window.fill(white)
    return window


def control_pg(virtual_image):
    if virtual_image:
        for event in pygame.event.get():
            if event == pygame.QUIT:
                pygame.quit()


def pygame_virtual_image(radius, centre, virtual_image, window):
    if virtual_image:
        window.fill(white)
        pygame.draw.circle(window, red, (centre[0], centre[1]), radius)
        pygame.display.update()
        return
    return


def draw_circle(mask, frame, virtual_image, window):
    # find contours in mask image
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    centre = None

    # only proceed if at least one contour was found
    if len(contours) > 0:
        # find the largest contour in the mask an assume that is the object we want to detect
        # use this to compute minumium closing circle and centroid
        max_contour = max(contours, key=cv2.contourArea)  # *inside contour area
        ((x, y), radius) = cv2.minEnclosingCircle(max_contour)
        M = cv2.moments(max_contour)
        if M["m00"] > 0:
            centre = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if radius > 10:
            # draw the circle and centroid on the frame
            #cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            #cv2.circle(frame, centre, 5, (0, 255, 255), -1)
            # function creates virtual image of circle on pygame
            pygame_virtual_image(radius, centre, virtual_image, window)
    return frame


def find_object(virtual_image, window, phone_cam):
    url = "http://192.168.0.49:8080/shot.jpg"
    video_stream = None
    if not phone_cam:
        video_stream = cv2.VideoCapture(1)



    #create trackbars to work out the best hsv code for an object

    def nothing(x):
        pass
    cv2.namedWindow("Trackbars")
    cv2.createTrackbar("L-H", "Trackbars", 0, 179, nothing)
    cv2.createTrackbar("L-S", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("L-V", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("U-H", "Trackbars", 179, 179, nothing)
    cv2.createTrackbar("U-S", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("U-V", "Trackbars", 255, 255, nothing)


    time.sleep(1)

    while True:

        # if anything pressed to control pygame GUI
        control_pg(virtual_image)

        # read current frame, resize, blur and convert to hsv colour
        if phone_cam:
            img_resp = requests.get(url)
            img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
            frame = cv2.imdecode(img_arr, -1)
        else:
            frame = video_stream.read()[1]

        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)



        l_h = cv2.getTrackbarPos("L-H", "Trackbars")
        l_s = cv2.getTrackbarPos("L-S", "Trackbars")
        l_v = cv2.getTrackbarPos("L-V", "Trackbars")
        u_h = cv2.getTrackbarPos("U-H", "Trackbars")
        u_s = cv2.getTrackbarPos("U-S", "Trackbars")
        u_v = cv2.getTrackbarPos("U-V", "Trackbars")
        # put these variables into the numpy array


        # define upper and lower bounds for colour of image
        red_lower = np.array([l_h, l_s, l_v])
        red_upper = np.array([u_h, u_s, u_v])
        red_lower_2 = np.array([0, 141, 77])
        red_upper_2 = np.array([10, 255, 255])

        # erode and dilate masked image for more defined picture of object
        mask = cv2.inRange(hsv, red_lower, red_upper)
        kernel = np.ones((2, 2), np.uint8)
        #mask2 = cv2.inRange(hsv, red_lower_2, red_upper_2)
        #mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # output frame with circle drawn around object
        #output_frame = draw_circle(mask, frame, virtual_image, window)

        # show masked image and output frame
        cv2.imshow("mask", mask)
        cv2.imshow("Frame", frame)
        #cv2.imshow("output_frame", output_frame)

        # if q is pressed exit frames and stop running
        if ((cv2.waitKey(1) & 0xFF) == ord("q")) or (cv2.waitKey(1) == 27):
            break

    pygame.quit()

    if not phone_cam:
        video_stream.release()

    return


def main():
    window = None
    virtual_image = False
    phone_cam = False
    if virtual_image:
        window = run_pg()
    find_object(virtual_image, window, phone_cam)

    cv2.destroyAllWindows()


main()
