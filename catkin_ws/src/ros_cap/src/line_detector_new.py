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
lower_white = np.array([0, 0, 155])
upper_white = np.array([255, 55, 255])
lower_yellow = np.array([20, 110, 170])
upper_yellow = np.array([40, 255, 255])

# Morfologias
kernel_dimensions = 4    # 5
erode_iterations = 1    # 1
dilate_iterations = 1    # 1

# Dibujo
yellow_figure_color = (255, 0, 255)
yellow_figure_thickness = 2
white_figure_color = (255, 0, 255)
white_figure_thickness = 2

# Dibujo centros de blobs detectados
show_centers_yellow = False
show_centers_white = False
yellow_centers_color = (0, 0, 255)
white_centers_color = (0, 0, 255)
centers_radius = 5
centers_thickness = 5

# Deteccion
minimum_deepness = 125
minimum_ratio_yellow = 1.5
minimum_ratio_white = 3.0    # 1.71


class LineDetector():

    def __init__(self):

        # Subscribirce al topico "/duckiebot/camera_node/image/rect"
        self.image_subscriber = rospy.Subscriber("/duckiebot/camera_node/image/rect", Image, self._process_image)

        # Publicar imagen con lineas al topico "duckiebot/camera_node/image/rect/streets"
        self.image_publisher = rospy.Publisher("duckiebot/camera_node/image/rect/streets", Image, queue_size=1)

        # Publicar datos de lineas al topico "/duckiebot/street_data"
        self.data_publisher = rospy.Publisher("/duckiebot/street_data", dict, queue_size=1)

        # Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        # Ultima imagen adquirida
        self.cv_image = Image()

        # Chiste
        print("encontrando lineas de calzada en 3, 2, 1...")

    def morfologies(self, mask):
        kernel = np.ones((kernel_dimensions, kernel_dimensions), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=erode_iterations)
        mask = cv2.dilate(mask, kernel, iterations=dilate_iterations)
        return mask

    def center(self, rect):
        x = rect[0][0]
        y = rect[0][1]
        center = Point()
        center.x = x
        center.y = y
        return center

    def ratio(self, rect):
        ratios = Point()
        w = float(rect[1][0])
        h = float(rect[1][1])
        ratios.x = h/w
        ratios.y = w/h
        return ratios

    def _process_image(self, image):

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
        frame_yellow = cv2.bitwise_and(frame, frame, mask=mask_yellow)
        frame_white = cv2.bitwise_and(frame, frame, mask=mask_white)

        # Cambio a espacio de color en blanco y negro
        frame_yellow = cv2.cvtColor(frame_yellow, gray_space)
        frame_white = cv2.cvtColor(frame_white, gray_space)

        # Deteccion de contornos
        image__yellow, contours_yellow, hierarchy_yellow = cv2.findContours(frame_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        image_white, contours_white, hierarchy_white = cv2.findContours(frame_white, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Creamos el diccionario con los datos
        data = {'yellow': 0, 'white': 0, 'yellow_data': [], 'white_data': []}

        # Manejo de datos amarillos
        for cnt_yellow in contours_yellow:

            # Encontrar rectangulos rotados y sus puntos centro
            rect_yellow = cv2.minAreaRect(cnt_yellow)
            center_yellow = self.center(rect_yellow)

            # Dibujar puntos centro
            if show_centers_yellow:
                center_yellow_coordinates = (int(center_yellow.x), int(center_yellow.y))
                frame = cv2.circle(frame, center_yellow_coordinates, centers_radius, yellow_centers_color, centers_thickness)

            # Condiciones para ser calzada amarilla
            ratio_yellow = self.ratio(rect_yellow)
            condition_yellow = center_yellow.y >= minimum_deepness and (ratio_yellow.x >= minimum_ratio_yellow or ratio_yellow.y >= minimum_ratio_yellow)

            if condition_yellow:
                # Dibujo de rectangulos en la imagen
                box__yellow = np.int0(cv2.boxPoints(rect_yellow))
                frame = cv2.drawContours(frame, [box__yellow], 0, yellow_figure_color, yellow_figure_thickness)

                # Extraccion de datos amarillos
                data['yellow'] += 1
                data['yellow_data'].append(rect_yellow)

        # Manejo de datos blancos
        for cnt_white in contours_white:

            # Encontrar rectangulos rotados y sus puntos centro
            rect_white = cv2.minAreaRect(cnt_white)
            center_white = self.center(rect_white)

            # Dibujar puntos centro
            if show_centers_white:
                center_white_coordinates = (int(center_white.x), int(center_white.y))
                frame = cv2.circle(frame, center_white_coordinates, centers_radius, white_centers_color, centers_thickness)

            # Condiciones para ser calzada blanca
            ratio_white = self.ratio(rect_white)
            condition_white = center_white.y >= minimum_deepness and (ratio_white.x >= minimum_ratio_white or ratio_white.y >= minimum_ratio_white)

            if condition_white:
                # Dibujo de rectangulos en la imagen
                box_white = np.int0(cv2.boxPoints(rect_white))
                frame = cv2.drawContours(frame, [box_white], 0, white_figure_color, white_figure_thickness)

                # Extraccion de datos blancos
                data['white'] += 1
                data['white_data'].append(rect_white)

        # Volver imagen al formato msg
        final_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")

        # Publicar la imagen
        self.image_publisher.publish(final_image)

        # Publicar datos
        self.data_publisher.publish(data)


def main():

    rospy.init_node('LineDetector')

    rospy.loginfo('LineDetector')

    LineDetector()

    rospy.spin()

if __name__ == '__main__':
    main()
