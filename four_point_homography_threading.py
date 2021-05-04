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

    start = time.time()
    cnt = 0

    while True:
        frame = video_stream.read()
        frame = cv2.resize(frame, (600, 480))
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # define upper and lower bounds for the colour of the image
        blue_lower = np.array([91, 40, 40])
        blue_upper = np.array([135, 211, 210])
        mask = cv2.inRange(hsv, blue_lower, blue_upper)

        kernel = np.ones((2, 2), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)

        output_frame = draw_circles(mask, frame)

        if None in output_frame[1]:
            pass
        else:
            markers = output_frame[1]
            w = 450
            h = 600
            pts1 = np.float32([[markers[0][0], markers[0][1]], [markers[1][0], markers[1][1]], [markers[2][0], markers[2][1]], [markers[3][0], markers[3][1]]])
            pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
            mx = cv2.getPerspectiveTransform(pts1, pts2)
            result = cv2.warpPerspective(frame, mx, (w, h))
            cv2.imshow("Result", result)

        cv2.imshow("Mask", mask)
        cv2.imshow("OutputFrame", output_frame[0])

        cnt += 1

        if cv2.waitKey(1) == 27:
            end = time.time()
            seconds = end - start
            fps = cnt/seconds
            print(fps)
            break

    video_stream.stop()
    cv2.destroyAllWindows()
    return

main()




