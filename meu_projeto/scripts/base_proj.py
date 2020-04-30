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

# ------------------------------------

# Definindo lista a ser utilizada para os pontos.
lista_pontos = []

# Função que calcula ponto de intersecção.  
def interseccao(x1,y1,x2,y2,x3,y3,x4,y4):
    x1 = x1
    y1 = y1
    x2 = x2
    y2 = y2
    x3 = x3
    y3 = y3
    x4 = x4
    y4 = y4
   
    # definindo coef angulares
    if x2-x1 == 0 or x4-x3 == 0:
        return 0.01, 0.01, 0.01, 0.01, 0.01, 0.01 
    else:
        m1 = (y2-y1)/(x2-x1)
        m2 = (y4-y3)/(x4-x3)
        # definindo coef lineares
        h1 = y1 - m1*x1
        h2 = y3 - m2*x3
        # achando as coordenadas da intersecção
        xi = (h2-h1)/(m1-m2)
        yi = m1*xi+h1
        # ponto de intersecção (xi,yi)
        return xi.round(4), yi.round(4), m1.round(4), m2.round(4), h1.round(4), h2.round(4)


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
        
        while not rospy.is_shutdown():
            for r in resultados:
                print(r)
            #velocidade_saida.publish(vel)

            # ---------------------------------------------
            frame = cv_image

            if frame is not None:
                


                # Our operations on the frame come here
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                hsv = cv2.cvtColor (frame, cv2.COLOR_BGR2HSV)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                # Fazendo a máscara.
                cor1_v2 = np.array([ 225, 225, 225], dtype=np.uint8)
                cor2_v2 = np.array([ 255, 255, 255], dtype=np.uint8)
                mascara = cv2.inRange(rgb, cor1_v2, cor2_v2)
                
                # Usando Canny para detectar todas as linhas da máscara
                min_contrast = 100
                max_contrast = 200
                linhas = cv2.Canny(gray, min_contrast, max_contrast )
                
                hough_img = linhas.copy() # Vamos reusar a imagem de canny


                lines = cv2.HoughLinesP(hough_img, 1, math.pi/180.0, 100, np.array([]), 45, 2.5)
                
                a,b,c = lines.shape
                
                hough_img_rgb = cv2.cvtColor(hough_img, cv2.COLOR_GRAY2BGR)

                # Parâmetros que fazem com que adiconem pontos à lista_pontos apenas 
                # duas vezes por frame.
                line1 = 0
                line2 = 0
                
                for i in range(a):
                    
                    # Pega as coordenadas iniciais e finais do ponto encontrado e calcula
                    # seu coeficiente angular.
                    coordenadas = lines[i][0]
                    x1,y1,x2,y2 = coordenadas
                    m = (y2-y1)/(x2-x1)
                
                    if  0.4 <= m <= 2.7 and line1 == 0:
                        
                        x1 = lines[i][0][0]
                        x2 = lines[i][0][2]
                        y1 = lines[i][0][1]
                        y2 = lines[i][0][3]
                        line1 +=1    
                        lista_pontos.append([x1,y1,x2,y2])
                    
                    elif -0.4 >= m >= -2.7 and line2 == 0:
                        
                        x3 = lines[i][0][0]
                        x4 = lines[i][0][2]
                        y3 = lines[i][0][1]
                        y4 = lines[i][0][3]
                        
                        line2 +=1
                        lista_pontos.append([x3,y3,x4,y4])
                        
                    # Só irá desenhar quando houver os pontos de duas retas em lista_pontos.    
                    if  len(lista_pontos) == 2:
                        
                        # Calcula o ponto de intersecção das duas retas e pega outros 
                        # valores para conseguirmos desenhar as retas na tela.
                        x, y, m1, m2, h1, h2 = interseccao(lista_pontos[0][0], lista_pontos[0][1], lista_pontos[0][2], lista_pontos[0][3],
                                                        lista_pontos[1][0], lista_pontos[1][1], lista_pontos[1][2], lista_pontos[1][3])
                        
                        # Previne erros de floats infinitas.
                        if x == -float('inf') or x == float('inf'):
                            x = 0
                            y = 0
                        
                        # Previne erros de NaN floats.
                        elif math.isnan(x) == True:
                            x = 0
                            y = 0
                        
                        # Transforma em inteiros para que cv2.line possa receber como
                        # parâmetro.
                        else:
                            x = int(x)
                            y = int(y)
                        
                        # Desenha um círculo cujo centro são as coordenadas do ponto
                        # de intersecção.
                        cv2.circle(frame, (x,y), 10, (255, 0, 0), -1 )
                        print("x:{}".format(x))
                        #print("y:{}".format(y))
                        
                        # Desenha as duas retas que vão se inteseccionar.
                        cv2.line(frame, (0, int(h1)), (10000, int(m1*10000 + h1)), (0,0,255), 2, cv2.LINE_AA)
                        cv2.line(frame, (0, int(h2)), (10000, int(m2*10000 + h2)), (0,0,255), 2, cv2.LINE_AA)
                        
                        # Esvazia lista_pontos para o próximo loop.
                        lista_pontos = []
            # ---------------------------------------------

            if cv_image is not None:
                # Note que o imshow precisa ficar *ou* no codigo de tratamento de eventos *ou* no thread principal, não em ambos
                cv2.imshow("cv_image no loop principal", cv_image)
            
                cv2.waitKey(1)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


