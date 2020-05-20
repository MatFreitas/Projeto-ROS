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
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import garra_demo

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
scan_dist = 190 # inicialmente setamos o laser para uma distancia alta

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
		r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
		m = numpy.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes
		z_marker = [0,0,1,0] # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
		v2 = numpy.dot(m, z_marker)
		v2_n = v2[0:-1] # Descartamos a ultima posicao
		n2 = v2_n/linalg.norm(v2_n) # Normalizamos o vetor
		x_robo = [1,0,0]
		cosa = numpy.dot(n2, x_robo) # Projecao do vetor normal ao marcador no x do robo
		angulo_marcador_robo = math.degrees(math.acos(cosa))

		# Terminamos
		#print("id: {} x {} y {} z {} angulo {} ".format(id, x,y,z, angulo_marcador_robo))



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

def checa_laser(scan_data):
	global scan_dist
	scan_dist = scan_data.ranges[0]

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

def cria_mascara(frame, cor_inf, cor_sup):
    if frame is not None:
        # converte crop para hsv
        hsv = cv2.cvtColor (frame, cv2.COLOR_BGR2HSV)
        # aplica filtro de cor
        mask = cv2.inRange(hsv, cor_inf, cor_sup)
        mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3, 3)))
        # display img
        #cv2.imshow("alvo", mask)
        return mask
    else:
        return None
    
def acha_ponto_futuro(mask, cont_ini, cont_final):
    px, py = 0, 0
    if mask is not None:
        # acha os contornos e define ponto médio
        contornos, arvore = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        pt_count = len(contornos)

        if pt_count > 0:
            cont_val = 0 # conta o numero de contornos
            cont_sum = 0 # conta o numero de elementos adicionados

            if cont_ini < 0:
                cont_ini = 0
            elif cont_ini >= pt_count:
                cont_ini = pt_count-1

            if cont_final < 0 or cont_final >= pt_count:
                cont_ini = pt_count-1

            for c in contornos:
                a = cv2.contourArea(c) # área
                p = center_of_contour(c) # centro de massa
                if cont_val >= cont_ini:
                    px += p[0]
                    py += p[1]
                    #crosshair(mask, p, 5, (80, 80, 80))    
                    cont_sum += 1
                    if cont_val >= cont_final:
                        break
                cont_val += 1
                
            # calcula pt medio
            if cont_sum > 0:
                px = int(px / cont_sum)
                py = int(py / cont_sum)
                crosshair(mask, (px, py), 5, (128,128,128))
            else:
                px, py = mask.shape[1]/2, mask.shape[0]/2

            # display img
            cv2.imshow("mask_img", mask)

    return px, py

def calc_velx_rotz(img_w, img_h, px, py):
    velx = 0
    rotz = 0
    if img_w > 0 and img_h > 0:
        # calcula angulo de rotacao
        # refx será deslocado para a direita para compensar as curvas
        refx = int(1*img_w/2)
        refy = img_h-1
        deltax = px - refx
        deltay = refy - py
        
        ang = math.atan2(deltay, deltax)
        if ang < 0:
            ang = -math.pi/2 - ang
        else:
            ang = math.pi/2 - ang

        velx = 0.25 # velx max = 0.5
        # calculo da rotacao depende de velx
        #rotz = (0.25/velx) * (-ang/math.pi)
        rotz = -ang/math.pi
        #print(math.degrees(ang))

    return velx, rotz

def procura_alvo(frame, cor_inf, cor_sup):
    velx = 0
    rotz = 0
    if frame is not None:
        # crop na imagem
        (h, w) = frame.shape[:2]
        crop = frame[int(h/4):int(3*h/4), 0:w]

        mask = cria_mascara(crop, cor_inf, cor_sup)
        px, py = acha_ponto_futuro(mask, 0, 3)
        if px > 0 or py > 0:
            return calc_velx_rotz(w, h, px, py)    

    return velx, rotz


def calc_rota(frame):
    # condicoes iniciais de velocidade e rotacao
    velx = 0.0
    rotz = -(math.pi/12)
    if frame is not None:
        # crop na imagem
        (h, w) = frame.shape[:2]
        crop = frame[int(h/2):h, 0:w]

        # range de cor para filtrar o tracejado amarelo
        pista_inf = np.array([ 28, 80, 80], dtype=np.uint8)
        pista_sup = np.array([ 32, 255, 255], dtype=np.uint8)

        mask = cria_mascara(crop, pista_inf, pista_sup)
        px, py = acha_ponto_futuro(mask, 1, 4)
        if px > 0 or py > 0:
            return calc_velx_rotz(w, h, px, py)  
                    
    return velx, rotz


def controla_garra(valor):

    tutorial = garra_demo.MoveGroupPythonIntefaceTutorial()
    if valor == 1:  # abre garra
        tutorial.open_gripper()

    if valor == 2:  # fecha garra
        tutorial.close_gripper()

    elif valor == 3: #recolhe garra
        tutorial.go_to_home_position_goal()

    else:    # 0 = vai pra posicao inicial garra
        tutorial.go_to_zero_position_goal()
        
# ------------------------------------ 

if __name__=="__main__":
    rospy.init_node("cor", anonymous=True)

    topico_imagem = "/camera/rgb/image_raw/compressed"
    #topico_imagem = "/raspicam/rgb/image_raw/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos
    print("Usando ", topico_imagem)

    # Subscriber do laser que devolve array com as distÂncias para o giro de 360°.
    recebe_scan = rospy.Subscriber("/scan", LaserScan, checa_laser)
    
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    # Exemplo de categoria de resultados
    # [('chair', 86.965459585189819, (90, 141), (177, 265))]

     # Objetivo
    mission_goal = ["blue", 11, "cat"]
    mission_id = mission_goal[1]
    mission_dest = mission_goal[2]
    mission_status = 0
    velx = 0
    rotz = 0  

    # define range de cor para identificacao dos creepers
    if mission_goal[0] == "blue":
        cor_inf = np.array([ 95, 80, 80], dtype=np.uint8)
        cor_sup = np.array([ 110, 255, 255], dtype=np.uint8)
    elif mission_goal[0] == "green":
        cor_inf = np.array([ 55, 80, 80], dtype=np.uint8)
        cor_sup = np.array([ 65, 255, 255], dtype=np.uint8)
    elif mission_goal[0] == "pink":
        cor_inf = np.array([ 145, 80, 80], dtype=np.uint8)
        cor_sup = np.array([ 155, 255, 255], dtype=np.uint8)
    else:
        mission_id = -1

    try:
        # Inicializando - por default gira no sentido anti-horário
        # vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/10.0))

        # zera timer
        robo_time0 = time.clock()
        
        while not rospy.is_shutdown():
            # ---------------------------------------------
            if len(resultados) > 0:
                for r in resultados:
                    estacao_atual = r[0]
                    estacao_x = int((r[2][0] + r[3][0]) / 2)
                    estacao_y = int((r[2][1] + r[3][1]) / 2)
                    print("resultados:", r)
            else:
                estacao_atual = "none"

            if cv_image is not None:

                if mission_status == 0:
                    robo_delay = time.clock() - robo_time0
                    #print("delay:", robo_delay)
                    if robo_delay > 10.0:
                        print("iniciando robot")
                        mission_status = 1
                
                elif mission_status == 1 and mission_id == id:
                    dist_alvo = math.sqrt(x*x + y*y)
                    velx, rotz = procura_alvo(cv_image, cor_inf, cor_sup)
                    print("dist do alvo:", dist_alvo)
                    
                    if dist_alvo < 0.2:
                        velx = 0
                        print("fechando a garra")
                        controla_garra(2)
                        robo_time0 = time.clock()
                        mission_status = 2   

                    elif dist_alvo < 0.4:
                        print("abrindo a garra")
                        controla_garra(1)
                        velx = 0.02

                    elif dist_alvo < 0.6:
                        print("inicia a garra")
                        controla_garra(0)
                        velx = 0.02

                    elif dist_alvo < 0.9:
                        velx = 0.05
                        print("desacelerando")

                elif mission_status == 2:
                    robo_delay = time.clock() - robo_time0
                    #print("delay:", robo_delay)
                    if robo_delay > 10.0:
                        print("move a garra")
                        controla_garra(3)
                        robo_time0 = time.clock()
                        mission_status = 3

                elif mission_status == 3:
                    velx = 0
                    robo_delay = time.clock() - robo_time0
                    #print("delay:", robo_delay)
                    if robo_delay > 5.0:
                        print("acionando ré")
                        robo_time0 = time.clock()
                        mission_status = 4 
                 
                elif mission_status == 4:
                    robo_delay = time.clock() - robo_time0
                    print("delay:", robo_delay)
                    if robo_delay < 25.0:
                        # marcha re depois que encontrar o creeper para evitar colisao
                        rotz = 0
                        velx = -0.20
                    else:
                        velx = 0
                        print("marcha ré finalizada")
                        mission_status = 5
                
                elif mission_status == 5 and mission_dest == estacao_atual and estacao_x > 0 and estacao_y > 0:
                        (h, w) = cv_image.shape[:2]
                        velx, rotz = calc_velx_rotz(w, h, estacao_x, estacao_y)
                        mission_status = 6
                        print("base encontrada")
                        
                elif mission_status == 6:
                    if mission_dest == estacao_atual and estacao_x > 0 and estacao_y > 0:
                        (h, w) = cv_image.shape[:2]
                        velx, rotz = calc_velx_rotz(w, h, estacao_x, estacao_y)
                        if scan_dist < 0.5:
                            velx = 0
                            rotz = 0
                            mission_status = 7
                            
                elif mission_status == 7:
                    # espera por comando para continuar o processo
                    comando = raw_input("digite o comando:")
                    if comando == "g":
                        print("deixou o creeper na estacao")
                        mission_status = 8

                elif mission_status == 8:
                        print("missão dada é missão cumprida!")
                        velx = 0
                        rotz = 0        
                        
                else:
                    velx, rotz = calc_rota(cv_image)

                #print(velx, rotz)
                vel = Twist(Vector3(velx,0,0), Vector3(0,0,rotz))
                    
                # Note que o imshow precisa ficar *ou* no codigo de tratamento de eventos *ou* no thread principal, não em ambos
                #cv2.imshow("cv_image no loop principal", cv_image)
                velocidade_saida.publish(vel)
                cv2.waitKey(1)
            # ---------------------------------------------

            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")