import time
import cv2
import numpy as np
import imutils
import requests

# define colours as global variables
white = (255, 255, 255)
red = (255, 0, 0)

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

        if radius > 10:
            # draw the circle and centroid on the frame
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, centre, 5, (0, 255, 255), -1)
    return frame

def find_object(url, phone_cam):
    video_stream = None
    if not phone_cam:
        video_stream = cv2.VideoCapture(0)

    while True:
        if phone_cam:
            img_resp = requests.get(url)
            img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
            frame = cv2.imdecode(img_arr, -1)
        else:
            frame = video_stream.read()[1]

        frame = cv2.resize(frame, (600, 480))
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # define upper and lower bounds for colour of image
        red_lower = np.array([170, 170, 70])
        red_upper = np.array([179, 255, 255])
        red_lower_2 = np.array([0, 170, 70])
        red_upper_2 = np.array([5, 255, 255])

        # erode and dilate masked image for more defined picture of object
        mask1 = cv2.inRange(hsv, red_lower, red_upper)
        mask2 = cv2.inRange(hsv, red_lower_2, red_upper_2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)


        # output frame with circle drawn around object
        output_frame = draw_circle(mask, frame)

        cv2.imshow("mask", mask)
        cv2.imshow("output_frame", output_frame)

        # if q is pressed exit frames and stop running
        if ((cv2.waitKey(1) & 0xFF) == ord("q")) or (cv2.waitKey(1) == 27):
            break

    video_stream.release()
    return


def main():
    phone_cam = False
    url = "http://xxxxxxxxx/shot.jpg"
    find_object(url, phone_cam)
    cv2.destroyAllWindows()

main()