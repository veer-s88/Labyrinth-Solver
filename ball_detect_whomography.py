from threading import Thread
import cv2
import numpy as np
import imutils
import time


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

def main():
    video_stream = WebcamVideoStream(src=1).start()

    while True:
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

            output = draw_circle(mask_ball, result)
            cv2.imshow('Mask of flat w/ ball', mask_ball)

            cv2.imshow("Result", output[0])

            ball_centre = output[1]
            print(ball_centre)

        cv2.imshow("Mask", mask)
        cv2.imshow("OutputFrame", output_frame[0])



        if cv2.waitKey(1) == 27:
            break

    video_stream.stop()
    cv2.destroyAllWindows()
    return

main()




