#!/usr/bin/env python

import math
import rospy
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse

from cv_bridge import CvBridge, CvBridgeError

import numpy as np

# Minimo largo de una linea
min_length = 25

# Minima distancia entre lineas
min_gap = 10

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

        # Encontramos las imagenes en los nuevos espacios
        gray = cv2.cvtColor(frame, gray_space)

        # Filtramos bordes de la imagen GRAY utilizada
        sketch = cv2.Canny(gray,20,50,apertureSize=5)

        # Deteccion de lineas
        lines = cv2.HoughLinesP(sketch,1,np.pi/180,1,min_length,min_gap)

        # Dibujo de lineas
        for l in lines:
            for x1, y1, x2, y2 in l:
                if (abs(x1-x2)^2 + abs(y1-y2)^2) >= min_length^2:
                    cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        print(lines)

        # Volver al formato imgmsg
        final_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")

        final_gray = self.bridge.cv2_to_imgmsg(gray, "8UC1")

        # final_sketch = self.bridge.cv2_to_imgmsg(sketch, "8UC1")


        # Publicar la imagen
        self.pub_1.publish(final_image)

        self.pub_gray.publish(final_gray)

        # self.pub_sketch.publish(final_sketch)
       
def main():

    rospy.init_node('LineDetector')

    LineDetector()

    rospy.spin()

if __name__ == '__main__':
    main()
