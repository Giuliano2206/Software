import math as np
import Lib\opencv\build\python\2.7\x64\cv2


class LineDetector():

    def __init__(self):
        self.image_1 = cv2.imread('in_1.jpg')
        self.image_2 = cv2.imread('in_2.jpg')

    def _process_image(self):
        image_1 = self.image_1
        image_2 = self.image_2

        gray_1 = cv2.cvtColor(image_1, cv2.COLOR_BGR2GRAY)
        gray_2 = cv2.cvtColor(image_2, cv2.COLOR_BGR2GRAY)

        edges_1 = cv2.Canny(gray_1, 50, 150, apertureSize=3)
        edges_2 = cv2.Canny(gray_2, 50, 150, apertureSize=3)

        min_line_length = 100
        max_line_gap = 10

        lines_1 = cv2.HoughLinesP(edges_1, 1, np.pi / 180, 100, min_line_length, max_line_gap)
        lines_2 = cv2.HoughLinesP(edges_2, 1, np.pi / 180, 100, min_line_length, max_line_gap)

        for x1, y1, x2, y2 in lines_1[0]:
            cv2.line(image_1, (x1, y1), (x2, y2), (0, 255, 0), 2)
        for x1, y1, x2, y2 in lines_2[0]:
            cv2.line(image_2, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imwrite('lines_1.jpg', image_1)
        cv2.imwrite('lines_2.jpg', image_2)

LineDetector()
