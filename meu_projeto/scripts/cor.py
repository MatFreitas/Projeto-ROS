#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cormodule


bridge = CvBridge()
dadodist = -10
cv_image = None
media = []
centro = []
maior_area = 0
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

# Função do scan.
def scaneou(dado):
	global dadodist
	dadodist = dado.ranges[0]
	x = 0

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
	#print("frame")
	global cv_image
	global maior_area
	global media
	global centro

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	#print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		# cv_image = cv2.flip(cv_image, -1)
		media, centro, maior_area =  cormodule.identifica_cor(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)


if __name__=="__main__":
	rospy.init_node("cor")

	# topico_imagem = "/kamera"
	topico_imagem = "/camera/rgb/image_raw/compressed"
	
	# Para renomear a *webcam*
	#   Primeiro instale o suporte https://github.com/Insper/robot19/blob/master/guides/debugar_sem_robo_opencv_melodic.md
	#
	#	Depois faça:
	#	
	#	rosrun cv_camera cv_camera_node
	#
	# 	rosrun topic_tools relay  /cv_camera/image_raw/compressed /kamera
	#
	# 
	# Para renomear a câmera simulada do Gazebo
	# 
	# 	rosrun topic_tools relay  /camera/rgb/image_raw/compressed /kamera
	# 
	# Para renomear a câmera da Raspberry
	# 
	# 	rosrun topic_tools relay /raspicam_node/image/compressed /kamera
	# 

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

	# Subscriber do laser que devolve array com as distÂncias para o giro de 360°.
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	print("Usando ", topico_imagem)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	try:
		veloc = 0
		calc = True
		stop = False
		while not rospy.is_shutdown():


			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

			if stop != True:
				if len(media) != 0 and len(centro) != 0:
					#print("Média dos vermelhos: {0}, {1}".format(media[0], media[1]))
					#print("Centro dos vermelhos: {0}, {1}".format(centro[0], centro[1]))

					if maior_area > 600:
						if (calc == True):
							dif = centro[0] - media[0]
						else:
							dif = 0

						print("Creeper encontrado em {0} metros!".format(dadodist))
						print("Dif: {0}!".format(dif))
						if (media[0] > centro[0]):
							if (dif > 100):
								vel = Twist(Vector3(veloc,0,0), Vector3(0,0,-0.1))
							elif (dif < 100):
								vel = Twist(Vector3(veloc,0,0), Vector3(0,0,-0.05))
							elif (dif < 50):
								vel = Twist(Vector3(veloc,0,0), Vector3(0,0,-0.01))
							elif (dif < 25):
								vel = Twist(Vector3(veloc,0,0), Vector3(0,0,-0.0005))
							"""
							elif (dif < 10):
								vel = Twist(Vector3(0.2,0,0), Vector3(0,0,-0.0001))
							elif (dif < 5):
								vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))
							"""

						elif (media[0] < centro[0]):
							if (dif > -100):
								vel = Twist(Vector3(veloc,0,0), Vector3(0,0,0.1))
							elif (dif > -100):
								vel = Twist(Vector3(veloc,0,0), Vector3(0,0,0.05))
							elif (dif > -50):
								vel = Twist(Vector3(veloc,0,0), Vector3(0,0,0.01))
							elif (dif > -25):
								vel = Twist(Vector3(veloc,0,0), Vector3(0,0,0.0005))
							"""
							elif (dif > -10):
								vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0.0001))
							elif (dif > -5):
								vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))
							"""
						if (dif == 0):
							veloc = 0.2
						if dadodist < 0.8:
							calc = False
							veloc = 0.1
						if dadodist < 0.3:
							calc = False
							veloc = 0
							stop = True



					else:
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0.5))
						print("Procurando creeper...")
						veloc = 0
			else:
				vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
				print("Stop my comrade!")

			#print("área: {}".format(maior_area))
			#print("distância frontal: {}".format(dadodist))
			velocidade_saida.publish(vel)
			rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("O ROSPY FECHOU!")
