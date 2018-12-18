#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Sergio González"
__copyright__ = "Social Robots Group. Robotics Lab. University Carlos III of Madrid"
__credits__ = ["Sergio González"]
__license__ = "LEUC3M v1.0"
__version__ = "0.0.0"
__maintainer__ = "Sergio González"
__email__ = "serigon@ing.uc3m.es"
__status__ = "Development"

from touch_screen_msgs.msg import ScreenButton, CustomizedTemplate, MultimediaContent, CustomizedMenu, CustomizedText
import rospy

import os

image_path = "image/audiobooks/"
audio_path = "audio/audiobooks/"


#>>>>>>>>>>>>>>>>>>>>>>>>>> CREAR PLANTILLA Y ENVIARLA A LA TABLET <<<<<<<<<<<<<<<<<<<<<<
def show_template(m_type, url, texto):

	# CREACIÓN DE LA PLANTILLA:
	msg = CustomizedTemplate()
	msg.num_template = '5'							# Número de la plantilla a crear

	fondo_url = "image/fondo_blanco.png"			 #URL de la imagen de fondo
	
	########################### IMÁGENES Y/O VÍDEOS ##############################
	#																			 #
	#	Crear un mensaje MultimediaContent() por cada imagen/video				 #

	content = MultimediaContent() 
	content.container = 'c_central_left'				# Posición en la plantilla
	content.type = 'image'								# Image,audio o video
	content.url = url									# url de la imagen o video
	content.msec = 0									# tiempo de inicio 
	msg.multimedia_config.append(content)
	'''
	content = MultimediaContent() 
	content.container = 'c_background'					# Posición en la plantilla
	content.type = 'image'								# Image,audio o video
	content.url = fondo_url								# url de la imagen o video
	content.msec = 0									# tiempo de inicio
	msg.multimedia_config.append(content)
	
	content = MultimediaContent() 
	content.container = 'c_lower'						# Posición en la plantilla
	content.type = 'image'								# Image, audio o video
	content.url = fondo_url					
	content.msec = 0									# url de la imagen o video
	msg.multimedia_config.append(content)
	'''
	
	################################### TEXTO ####################################
	#																			 #
	#	Crear un mensaje CustomizedText() por cada texto				 		 #
	
	content = CustomizedText()
	content.container = 'c_upper'					# Posición
	content.customized_text = texto					# texto
	content.size_text = 115							# tamaño (125 para títulos)
	content.font_text = 'Helvetica'					# tipo de letra 'Helvetica'
	msg.text_config.append(content)
	
	#print(msg)

	return msg


if __name__ == '__main__':
	try:
		print("[show_tablet] __main__")

		rospy.init_node('show_tablet')

		pub_template = rospy.Publisher("/alz4/show_customized_template", CustomizedTemplate, queue_size=1)

		msg = show_template('image', 'image/weather/day/113.png', 'texto de prueba')

		salir = 0
		while salir < 2:

			pub_template.publish(msg)
			salir = salir + 1
			rospy.sleep(1)

		rospy.sleep(60)

	except rospy.ROSInterruptException:
		pass