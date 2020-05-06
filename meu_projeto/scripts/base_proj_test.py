#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


import visao_module


bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos


area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0
# teste frame error no lookup
frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()

def recebe(msg):
	global x # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
	global y
	global z
	global id
	for marker in msg.markers:
		id = marker.id
		marcador = "ar_marker_" + str(id)

		print(tf_buffer.can_transform(frame, marcador, rospy.Time(0)))
		header = Header(frame_id=marcador)
		# Procura a transformacao em sistema de coordenadas entre a base do robo e o marcador numero 100
		# Note que para seu projeto 1 voce nao vai precisar de nada que tem abaixo, a 
		# Nao ser que queira levar angulos em conta
		trans = tf_buffer.lookup_transform(frame, marcador, rospy.Time(0))
		
		# Separa as translacoes das rotacoes
		x = trans.transform.translation.x
		y = trans.transform.translation.y
		z = trans.transform.translation.z
		# ATENCAO: tudo o que vem a seguir e'  so para calcular um angulo
		# Para medirmos o angulo entre marcador e robo vamos projetar o eixo Z do marcador (perpendicular) 
		# no eixo X do robo (que e'  a direcao para a frente)
		t = transformations.translation_matrix([x, y, z])
		# Encontra as rotacoes e cria uma matriz de rotacao a partir dos quaternions
		r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, 
                                              trans.transform.rotation.z, trans.transform.rotation.w])
		m = numpy.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes
		z_marker = [0,0,1,0] # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
		v2 = numpy.dot(m, z_marker)
		v2_n = v2[0:-1] # Descartamos a ultima posicao
		n2 = v2_n/linalg.norm(v2_n) # Normalizamos o vetor
		x_robo = [1,0,0]
		cosa = numpy.dot(n2, x_robo) # Projecao do vetor normal ao marcador no x do robo
		angulo_marcador_robo = math.degrees(math.acos(cosa))

		# Terminamos
		print("id: {} x {} y {} z {} angulo {} ".format(id, x,y,z, angulo_marcador_robo))



# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    #print("frame")
    global cv_image
    global media
    global centro
    global resultados

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    # print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
        centro, saida_net, resultados =  visao_module.processa(temp_image)        
        for r in resultados:
            # print(r) - print feito para documentar e entender
            # o resultado            
            pass

        depois = time.clock()
        # Desnecessário - Hough e MobileNet já abrem janelas
        cv_image = saida_net.copy()
    except CvBridgeError as e:
        print('ex', e)


# ------------------------------------

def center_of_contour(contorno):
    """ Retorna uma tupla (cx, cy) que desenha o centro do contorno"""
    M = cv2.moments(contorno)
    # Usando a expressão do centróide definida em: https://en.wikipedia.org/wiki/Image_moment
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX = 0
        cY = 0
    return (int(cX), int(cY))
    
def crosshair(img, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,5)
    cv2.line(img,(x,y - size),(x, y + size),color,5)

def calc_rota(frame, sentido):
    # condicoes iniciais de velocidade e rotacao
    velx = 0.0
    rotz = sentido * (math.pi/12)
    if frame is not None:
        # crop na imagem
        (h, w) = frame.shape[:2]
        crop = frame[int(h/2):h, 0:w]

        # converte crop para hsv
        hsv = cv2.cvtColor (crop, cv2.COLOR_BGR2HSV)

        # range de cor para filtrar o tracejado amarelo
        cor1_v2 = np.array([ 28, 80, 80], dtype=np.uint8)
        cor2_v2 = np.array([ 32, 255, 255], dtype=np.uint8)

        # aplica filtro de cor
        mask = cv2.inRange(hsv, cor1_v2, cor2_v2)
        bgr_mask = cv2.bitwise_and(crop, crop, mask = mask)
        gray = cv2.cvtColor (bgr_mask, cv2.COLOR_BGR2GRAY)
        
        # Usando Canny para detectar os contornos
        #min_contrast = 80
        #max_contrast = 255
        #canny_img = cv2.Canny(gray, min_contrast, max_contrast)

        # acha os contornos e define ponto médio
        contornos, arvore = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        pt_count = len(contornos)

        if pt_count > 0:
            # pt_val conta o número de elementos encontrados, e pula o primeiro elemento
            # pt_max define o total de elementos, no caso 3 ou menos
            px, py = 0, 0
            pt_val = 0
            if pt_count > 4:
                pt_max = 4
            else:
                pt_max = pt_count
        
            for c in contornos:
                a = cv2.contourArea(c) # área
                p = center_of_contour(c) # centro de massa
                if pt_val < pt_max:
                    if pt_val > 0:
                        px += p[0]
                        py += p[1]
                        #crosshair(gray, p, 5, (255,255,255))
                    pt_val += 1
                else:
                    break
                
            # calcula pt medio
            if pt_val > 1:
                pt_val -= 1
                px = int(px / pt_val)
                py = int(py / pt_val)
                crosshair(gray, (px, py), 5, (255,255,255))

                # calcula angulo de rotacao
                (h, w) = crop.shape[:2]
                # refx será deslocado para a direita para compensar as curvas
                refx = int(5*w/9)
                refy = h-1
                deltax = px - refx
                deltay = refy - py
                
                ang = math.atan2(deltay, deltax)
                if ang < 0:
                    ang = -math.pi/2 - ang
                else:
                    ang = math.pi/2 - ang

                velx = 0.25 # velx max = 0.5
                # calculo da rotacao depende de velx
                rotz = (0.25/velx) * (-ang/math.pi)
                #print(math.degrees(ang))
                        
        # display img
        cv2.imshow("gray_img", gray)
        #cv2.imshow("canny_img", canny_img)
        
    return velx, rotz
    

# ------------------------------------

if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/raspicam/rgb/image_raw/compressed"

    recebedor1 = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tolerancia = 25

    # Exemplo de categoria de resultados
    # [('chair', 86.965459585189819, (90, 141), (177, 265))]

    try:
        # Inicializando - por default gira no sentido anti-horário
        # vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/10.0))
        recupera = 0
        sentido = 1
        while not rospy.is_shutdown():
            for r in resultados:
                print(r)
            
            # ---------------------------------------------
            if cv_image is not None:
                # calcula ponto futuro da rota
                velx, rotz = calc_rota(cv_image, sentido)
                # sentido rotaciona o robo no sentido a favor da pista
                if velx == 0 and recupera == 0:
                    recupera = 1
                    if rotz < 0:
                        sentido = 1
                    else:
                        sentido = -1
                elif velx > 0 and recupera == 1:
                    recupera = 0

                print(velx, rotz)
                vel = Twist(Vector3(velx,0,0), Vector3(0,0,rotz))
                #vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/10.0))

                # Note que o imshow precisa ficar *ou* no codigo de tratamento de eventos *ou* no thread principal, não em ambos
                #cv2.imshow("cv_image no loop principal", cv_image)
                velocidade_saida.publish(vel)
            # ---------------------------------------------
            
            cv2.waitKey(1)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


