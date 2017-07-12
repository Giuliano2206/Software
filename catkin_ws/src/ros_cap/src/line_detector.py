#!/usr/bin/env python

import math
import rospy
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse

from cv_bridge import CvBridge, CvBridgeError

import numpy as np

# Define range of color in HSV
lower_white = np.array([0,0,155])
upper_white = np.array([255,55,255])
lower_yellow = np.array([20,110,170])
upper_yellow = np.array([40,255,255])

# Morfologias
kernel_dimentions = 4 # 5
erode_iterations = 1 # 1
dilate_iterations = 1 # 1

# Dibujo
yellow_figure_color = (255,0,255)
yellow_figure_thickness = 2
white_figure_color = (255,0,255)
white_figure_thickness = 2
show_centers_white = False
show_centers_yellow = False

# Deteccion
profundidad_min = 125
ratio_min = 1.5    # 1.71

class LineDetector():

    def __init__(self):


        # Subscribirce al topico "/duckiebot/camera_node/image/raw"
        self.image_subscriber = rospy.Subscriber("/duckiebot/camera_node/image/rect",Image,self._process_image)

        # Publicar imagen con lineas al topico "duckiebot/lineafiltrada"
        self.pub_1 = rospy.Publisher("/duckiebot/lineafiltrada",Image,queue_size=1)

        # Publicar datos de lineas al topico "/duckiebot/datoslineas"
        self.pub_2 = rospy.Publisher("/duckiebot/datoslineas",Point,queue_size=1)

        # Publicar imagen en matiz de grises al topico "/duckiebot/gray"
        self.pub_gray = rospy.Publisher("/duckiebot/gray",Image,queue_size=1)

        # Publicar imagen el sketch al topico "/duckiebot/sketch"
        self.pub_sketch = rospy.Publisher("/duckiebot/sketch",Image,queue_size=1)

        # Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        # Ultima imagen adquirida
        self.cv_image = Image()

        # Chiste
        print("encontrando lineas en 3, 2, 1...") 

    def morfologies(self,mask):
        kernel = np.ones((kernel_dimentions,kernel_dimentions),np.uint8)
        mask = cv2.erode(mask, kernel, iterations = erode_iterations)
        mask = cv2.dilate(mask, kernel, iterations = dilate_iterations)
        return mask

    def center(self,rect):
        x = rect[0][0]
        y = rect[0][1]
        center = Point()
        center.x = x
        center.y = y
        return center

    def ratio(self,rect):
        ratios = Point()
        w = float(rect[1][0])
        h = float(rect[1][1])
        ratios.x = h/w
        ratios.y = w/h
        return ratios

    def _process_image(self,image):

        # Se cambia mensage tipo ros a imagen opencv
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Se deja en frame la imagen actual
        frame = self.cv_image

        # Creamos los espacios de color
        gray_space = cv2.COLOR_BGR2GRAY
        color_space = cv2.COLOR_BGR2HSV

        # Encontramos las mascaras de colores blanco y amarillo
        frame_hsv = cv2.cvtColor(frame, color_space)
        mask_yellow = cv2.inRange(frame_hsv, lower_yellow, upper_yellow)
        mask_white = cv2.inRange(frame_hsv, lower_white, upper_white)

        # Realizamos las operaciones morfologicas para borrar manchas pequenas
        mask_yellow = self.morfologies(mask_yellow)
        mask_white = self.morfologies(mask_white)

        # Filtramos la imagen con esos colores
        frame_yellow = cv2.bitwise_and(frame,frame, mask= mask_yellow)
        frame_white = cv2.bitwise_and(frame,frame, mask= mask_white)

        # Cambio de tipo de imagen
        frame_yellow = cv2.cvtColor(frame_yellow, gray_space)
        frame_white = cv2.cvtColor(frame_white, gray_space)

        # Deteccion de contornos
        image_1, contours_1, hierarchy_1 = cv2.findContours(frame_yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        image_2, contours_2, hierarchy_2 = cv2.findContours(frame_white,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        # Dibujo de rectangulos
        for cnt_1 in contours_1:
            rect_1 = cv2.minAreaRect(cnt_1)
            center_1 = self.center(rect_1)
            if show_centers_yellow:
                frame = cv2.circle(frame, (int(center_1.x),int(center_1.y)), 5, (0,0,255),5)
            ratio_1 = self.ratio(rect_1)
            t_1 = ratio_1.x >= ratio_min or ratio_1.y >= ratio_min
            if center_1.y >= profundidad_min and t_1:
                box_1 = np.int0(cv2.boxPoints(rect_1))
                frame = cv2.drawContours(frame, [box_1], 0, yellow_figure_color, yellow_figure_thickness)

        for cnt_2 in contours_2:
            rect_2 = cv2.minAreaRect(cnt_2)
            center_2 = self.center(rect_2)
            if show_centers_white:
                frame = cv2.circle(frame, (int(center_2.x),int(center_2.y)), 5, (0,0,255),5)
            ratio_2 = self.ratio(rect_2)
            t_2 = ratio_2.x >= ratio_min or ratio_2.y >= ratio_min
            if center_2.y >= profundidad_min and t_2:
                box_2 = np.int0(cv2.boxPoints(rect_2))
                frame = cv2.drawContours(frame, [box_2], 0, white_figure_color, white_figure_thickness)

        # Volver al formato imgmsg
        final_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")

        # Publicar la imagen
        self.pub_1.publish(final_image)


def main():

    rospy.init_node('LineDetector')

    LineDetector()

    rospy.spin()

if __name__ == '__main__':
    main()
